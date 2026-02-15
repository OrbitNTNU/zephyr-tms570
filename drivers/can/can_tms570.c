
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/can.h>
#include <zephyr/sys/bitarray.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/drivers/clock_control.h>

LOG_MODULE_REGISTER(can_tms570);

#define DT_DRV_COMPAT ti_tms570_can

#define MSG_OBJECT_COUNT (64)
#define MSG_TX_MAX       (CONFIG_CAN_TMS570_TX_MAX)
#define MSG_RX_MAX       (MSG_OBJECT_COUNT - MSG_TX_MAX)

#define BITRATE_MIN (0)
#define BITRATE_MAX (1000000)

#define CTL_OFFSET      (0x00)
#define CTL_SWR_OFFSET  (15)
#define CTL_ABO_OFFSET  (9)
#define CTL_CCE_OFFSET  (6)
#define CTL_EIE_OFFSET  (3)
#define CTL_SIE_OFFSET  (2)
#define CTL_IE0_OFFSET  (1)
#define CTL_INIT_OFFSET (0)

#define ES_OFFSET       (0x04)
#define ES_BOFF_OFFSET  (7)
#define ES_EWARN_OFFSET (6)
#define ES_EPASS_OFFSET (5)

#define ERRC_OFFSET     (0x08)
#define ERRC_REC_WIDTH  (7)
#define ERRC_REC_OFFSET (8)
#define ERRC_TEC_WIDTH  (8)
#define ERRC_TEC_OFFSET (0)

#define BTR_OFFSET       (0x0c)
#define BTR_BRPE_WIDTH   (4)
#define BTR_BRPE_OFFSET  (16)
#define BTR_TSEG2_WIDTH  (3)
#define BTR_TSEG2_OFFSET (12)
#define BTR_TSEG1_WIDTH  (4)
#define BTR_TSEG1_OFFSET (8)
#define BTR_SJW_WIDTH    (2)
#define BTR_SJW_OFFSET   (6)
#define BTR_BRP_WIDTH    (6)
#define BTR_BRP_OFFSET   (0)

#define INT_OFFSET     (0x10)
#define INT_1D_WIDTH   (8)
#define INT_1ID_OFFSET (16)
#define INT_0ID_WIDTH  (16)
#define INT_0ID_OFFSET (0)
#define INT_ESR_SOURCE (0x8000)

#define IF1_OFFSET (0x100)
#define IF1_IDX    (0)
#define IF2_OFFSET (0x120)
#define IF2_IDX    (1)
#define IF_REG_MAX (2) /* IF1/IF2 (excluding IF3) */

#define IF_CMD_OFFSET                 (0x00)
#define IF_CMD_WR_RD_OFFSET           (23)
#define IF_CMD_MASK_BIT               BIT(22)
#define IF_CMD_ARB_BIT                BIT(21)
#define IF_CMD_CONTROL_BIT            BIT(20)
#define IF_CMD_CLRINTPND_BIT          BIT(19)
#define IF_CMD_TXRQST_BIT             BIT(18)
#define IF_CMD_NEWDAT_BIT             IF_CMD_TXRQST_BIT
#define IF_CMD_DATA_A_BIT             BIT(17)
#define IF_CMD_DATA_B_BIT             BIT(16)
#define IF_CMD_BUSY_OFFSET            (15)
#define IF_CMD_DATA_DMA_ACTIVE_OFFSET (14)
#define IF_CMD_MSG_NUM_MAX            (0x40)
#define IF_CMD_MSG_NUM_OFFSET         (0)

#define IF_MASK_OFFSET      (0x04)
#define IF_MASK_MXTD_OFFSET (31)
#define IF_MASK_STD_WIDTH   (11)
#define IF_MASK_STD_OFFSET  (18)
#define IF_MASK_EXT_WIDTH   (29)
#define IF_MASK_EXT_OFFSET  (0)

#define IF_ARB_OFFSET        (0x08)
#define IF_ARB_MSGVAL_OFFSET (31)
#define IF_ARB_XTD_OFFSET    (30)
#define IF_ARB_DIR_OFFSET    (29)
#define IF_ARB_ID_WIDTH      (11)
#define IF_ARB_ID_OFFSET     (18)
#define IF_ARB_IDE_WIDTH     (29)
#define IF_ARB_IDE_OFFSET    (0)

#define IF_MCTL_OFFSET        (0x0c)
#define IF_MCTL_NEWDAT_OFFSET (15)
#define IF_MCTL_UMASK_OFFSET  (12)
#define IF_MCTL_TXIE_OFFSET   (11)
#define IF_MCTL_RXIE_OFFSET   (10)
#define IF_MCTL_RMTEN_OFFSET  (9)
#define IF_MCTL_TXRQST_OFFSET (8)
#define IF_MCTL_EOB_OFFSET    (7)
#define IF_MCTL_DLC_WIDTH     (4)
#define IF_MCTL_DLC_OFFSET    (0)

#define IF_DATA_A_OFFSET (0x10)
#define IF_DATA_B_OFFSET (0x14)

struct tms570_can_msg_object {
        union {
                can_tx_callback_t tx_callback;
                can_rx_callback_t rx_callback;
        };
        void *user_data;
};

struct tms570_can_cfg {
        DEVICE_MMIO_ROM;

        struct can_driver_config can_conf;

        const struct device *clk_ctrl;
        unsigned int clk_domain;

        struct tms570_can_msg_object *tx_objects;
        sys_bitarray_t *tx_bitarray;

        struct tms570_can_msg_object *rx_objects;
        sys_bitarray_t *rx_bitarray;

        void (*irq_connect)(void);
};

struct tms570_can_data {
        DEVICE_MMIO_RAM;

        struct k_spinlock lock;
        struct can_driver_data can_data;
        enum can_state can_state;

        atomic_t ifregs;
        struct k_sem ifsem;

        struct k_sem txsem;
};

static int tms570_can_get_capabilities(const struct device *dev, can_mode_t *cap)
{
        *cap = CAN_MODE_NORMAL;
        return 0;
}

static int tms570_can_set_mode(const struct device *dev, can_mode_t mode)
{
        struct tms570_can_data *data = dev->data;
        int status;
        k_spinlock_key_t key;

        status = 0;

        key = k_spin_lock(&data->lock);

        if (data->can_data.started) {
                status = -EBUSY;
                goto exit;
        }

        if (mode != CAN_MODE_NORMAL) {
                status = -EINVAL;
                goto exit;
        }

        data->can_data.mode = mode;

exit:
        k_spin_unlock(&data->lock, key);

        return status;
}

static int tms570_can_set_timing(const struct device *dev, const struct can_timing *timing)
{
        struct tms570_can_data *data = dev->data;
        uintptr_t ctrl_reg_base;
        uint32_t tseg1;
        uint32_t tseg2;
        uint32_t brp;
        uint32_t sjw;
        uint32_t reg;
        bool started;

        K_SPINLOCK(&data->lock) {
                started = data->can_data.started;
        }

        if (started) {
                LOG_ERR("Can't change timing while running");
                return -EBUSY;
        }

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        tseg1 = (timing->prop_seg + timing->phase_seg1) - 1;
        tseg2 = timing->phase_seg2 - 1;
        brp = timing->prescaler - 1;
        sjw = timing->sjw - 1;

        LOG_DBG("Setting timing: tseg1=%" PRIu32 "(%" PRIu32 " + %" PRIu32 " - 1), tseg2=%" PRIu32
                "(%" PRIu32 "-1), brp=%" PRIu32 "(%" PRIu32 "-1), sjw=%" PRIu32 "(%" PRIu32 "-1)",
                tseg1, timing->prop_seg, timing->phase_seg1, tseg2, timing->phase_seg2, brp,
                timing->prescaler, sjw, timing->sjw);

        reg = (brp & BIT_MASK(BTR_BRP_WIDTH)) << BTR_BRP_OFFSET;
        reg |= ((brp >> BTR_BRP_WIDTH) & BIT_MASK(BTR_BRPE_WIDTH)) << BTR_BRPE_OFFSET;
        reg |= sjw << BTR_SJW_OFFSET;
        reg |= tseg1 << BTR_TSEG1_OFFSET;
        reg |= tseg2 << BTR_TSEG2_OFFSET;

        sys_write32(reg, ctrl_reg_base + BTR_OFFSET);

        LOG_DBG("Timing updated successfully");

        return 0;
}

static int tms570_can_start(const struct device *dev)
{
        struct tms570_can_data *data = dev->data;
        uintptr_t ctrl_reg_base;
        k_spinlock_key_t key;

        key = k_spin_lock(&data->lock);

        if (data->can_data.started) {
                k_spin_unlock(&data->lock, key);
                return -EALREADY;
        }

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        /* SW Reset module */
        sys_set_bit(ctrl_reg_base + CTL_OFFSET, CTL_SWR_OFFSET);
        while (sys_test_bit(ctrl_reg_base + CTL_OFFSET, CTL_SWR_OFFSET)) {
        }

        /* Enable interrupts */
        sys_set_bits(ctrl_reg_base + CTL_OFFSET, BIT(CTL_IE0_OFFSET) | BIT(CTL_SIE_OFFSET) |
                                                         BIT(CTL_EIE_OFFSET) | BIT(CTL_ABO_OFFSET));

        sys_clear_bit(ctrl_reg_base + CTL_OFFSET, CTL_CCE_OFFSET);
        sys_clear_bit(ctrl_reg_base + CTL_OFFSET, CTL_INIT_OFFSET);

        while (sys_test_bit(ctrl_reg_base + CTL_OFFSET, CTL_INIT_OFFSET)) {
        }

        data->can_data.started = true;
        data->can_state = CAN_STATE_ERROR_ACTIVE;

        k_spin_unlock(&data->lock, key);

        return 0;
}

static int tms570_can_stop(const struct device *dev)
{
        struct tms570_can_data *data = dev->data;
        uintptr_t ctrl_reg_base;
        k_spinlock_key_t key;

        key = k_spin_lock(&data->lock);

        if (!data->can_data.started) {
                k_spin_unlock(&data->lock, key);
                return -EALREADY;
        }

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        /* Set device in initialization mode, allow for configuration change */
        sys_set_bit(ctrl_reg_base + CTL_OFFSET, CTL_INIT_OFFSET);
        sys_set_bit(ctrl_reg_base + CTL_OFFSET, CTL_CCE_OFFSET);

        while (!sys_test_bit(ctrl_reg_base + CTL_OFFSET, CTL_INIT_OFFSET)) {
        }

        data->can_data.started = false;

        k_spin_unlock(&data->lock, key);

        return 0;
}

/**
 * @brief Get access a Interface Register set.
 *
 * The returned interface register set index should be released by
 * @ref ifreg_give after being synchronized.
 *
 * @return int
 * @retval <0 Negative errno code
 * @retval >=0 Interface register set index
 */
static int ifreg_take(const struct device *dev, k_timeout_t timeout)
{
        struct tms570_can_data *data = dev->data;
        int status;

        status = k_sem_take(&data->ifsem, timeout);
        if (status < 0) {
                return status;
        }

        if (atomic_test_and_set_bit(&data->ifregs, IF1_IDX)) {
                return IF1_IDX;
        }

        if (atomic_test_and_set_bit(&data->ifregs, IF2_IDX)) {
                return IF2_IDX;
        }

        /* Should not happen, as one of the two should be available if
         * we have successfully taken the semaphore. */
        __ASSERT_NO_MSG(0);
        return -EBUSY;
}

/** @brief Release interface register set */
static void ifreg_give(const struct device *dev, int ifreg)
{
        struct tms570_can_data *data = dev->data;

        atomic_clear_bit(&data->ifregs, ifreg);
        (void)k_sem_give(&data->ifsem);
}

static uintptr_t ifreg_addr(const struct device *dev, int ifreg)
{
        uintptr_t base;

        base = DEVICE_MMIO_GET(dev);
        return base + (ifreg == IF1_IDX ? IF1_OFFSET : IF2_OFFSET);
}

/**
 * @brief Synchronize interface registers and message object with index @p msg
 *
 * @param dev
 * @param msg Index of message object
 * @param if_to_msg Direction of synchronization. When true, synchronize interface register set to
 * message object. When false, synchronize other direction.
 * @param mask Mask of registers to synchronize.
 */
static void ifreg_msgobj_sync(const struct device *dev, int ifreg, uint8_t msg, bool if_to_msg,
                              unsigned int mask)
{
        uintptr_t if_reg_base;
        uint32_t val;

        __ASSERT_NO_MSG(msg <= IF_CMD_MSG_NUM_MAX);

        if_reg_base = ifreg_addr(dev, ifreg);

        val = 0;
        if (if_to_msg) {
                val |= BIT(IF_CMD_WR_RD_OFFSET);
        }

        val |= mask;
        val |= (msg + 1) << IF_CMD_MSG_NUM_OFFSET;

        sys_write32(val, if_reg_base + IF_CMD_OFFSET);

        /* Wait for busy to clear so we can release the IF register */
        while (sys_test_bit(if_reg_base + IF_CMD_OFFSET, IF_CMD_BUSY_OFFSET)) {
        }
}

static void tx_done(const struct device *dev, size_t msg_id, int ifreg, int status)
{
        const struct tms570_can_cfg *cfg = dev->config;
        struct tms570_can_data *data = dev->data;

        if (cfg->tx_objects[msg_id].tx_callback != NULL) {
                cfg->tx_objects[msg_id].tx_callback(dev, status, cfg->tx_objects[msg_id].user_data);
        }

        /* Clear message valid as we do not use object anymore */
        sys_clear_bit(ifreg_addr(dev, ifreg) + IF_ARB_OFFSET, IF_ARB_MSGVAL_OFFSET);
        ifreg_msgobj_sync(dev, ifreg, msg_id, true, IF_CMD_ARB_BIT);

        K_SPINLOCK(&data->lock) {
                (void)sys_bitarray_clear_bit(cfg->tx_bitarray, msg_id);
        }

        (void)k_sem_give(&data->txsem);
}

static int tms570_can_send(const struct device *dev, const struct can_frame *frame,
                           k_timeout_t timeout, can_tx_callback_t callback, void *user_data)
{
        const struct tms570_can_cfg *cfg = dev->config;
        struct tms570_can_data *data = dev->data;
        int if_reg;
        int status;
        uint32_t val;
        uintptr_t if_reg_base;
        size_t msg_id;
        k_timepoint_t expiry;

        if (frame->flags & (CAN_FRAME_FDF | CAN_FRAME_ESI | CAN_FRAME_BRS | CAN_FRAME_RTR)) {
                return -ENOTSUP;
        }

        expiry = sys_timepoint_calc(timeout);

        /* Wait for free TX object */
        status = k_sem_take(&data->txsem, sys_timepoint_timeout(expiry));
        if (status < 0) {
                return status;
        }

        /* Allocate message id. Should never fail, as we have taken the
         * semaphore. */
        K_SPINLOCK(&data->lock) {
                status = sys_bitarray_alloc(cfg->tx_bitarray, 1, &msg_id);
        }

        if (status < 0) {
                (void)k_sem_give(&data->txsem);
                return status;
        }

        /* Get access to one of the interface register sets, so we can
         * indirectly write to the underlying message objects. */
        if_reg = ifreg_take(dev, sys_timepoint_timeout(expiry));
        if (if_reg < 0) {
                K_SPINLOCK(&data->lock) {
                        (void)sys_bitarray_clear_bit(cfg->tx_bitarray, msg_id);
                }

                (void)k_sem_give(&data->txsem);
                return if_reg;
        }

        if_reg_base = ifreg_addr(dev, if_reg);

        cfg->tx_objects[msg_id].user_data = user_data;
        cfg->tx_objects[msg_id].tx_callback = callback;

        /* Valid message, TX direction */
        val = BIT(IF_ARB_MSGVAL_OFFSET) | BIT(IF_ARB_DIR_OFFSET);

        if (frame->flags & CAN_FRAME_IDE) {
                val |= BIT(IF_ARB_XTD_OFFSET); /* Enable extended ID */
                val |= frame->id << IF_ARB_IDE_OFFSET;
        } else {
                val |= frame->id << IF_ARB_ID_OFFSET;
        }
        sys_write32(val, if_reg_base + IF_ARB_OFFSET);

        /* Load data into IF registers. These are in big-endian order */
        sys_write32(sys_le32_to_cpu(frame->data_32[0]), if_reg_base + IF_DATA_A_OFFSET);
        sys_write32(sys_le32_to_cpu(frame->data_32[1]), if_reg_base + IF_DATA_B_OFFSET);

        val = frame->dlc << IF_MCTL_DLC_OFFSET; /* Data length */
        val |= BIT(IF_MCTL_TXRQST_OFFSET);      /* Set TX request */
        val |= BIT(IF_MCTL_TXIE_OFFSET);        /* Tx Interrupt Enable */
        val |= BIT(IF_MCTL_EOB_OFFSET);         /* End of block */
        sys_write32(val, if_reg_base + IF_MCTL_OFFSET);

        ifreg_msgobj_sync(dev, if_reg, msg_id, true,
                          IF_CMD_ARB_BIT | IF_CMD_CONTROL_BIT | IF_CMD_DATA_A_BIT |
                                  IF_CMD_DATA_B_BIT | IF_CMD_TXRQST_BIT);
        ifreg_give(dev, if_reg);

        return 0;
}

static void rx_deliver(const struct device *dev, int ifreg, size_t msg_id)
{
        const struct tms570_can_cfg *cfg = dev->config;
        struct can_frame frame;
        uintptr_t if_addr_base;
        uint32_t val;

        /* Callback not configured, nothing to be done */
        if (cfg->rx_objects[msg_id].rx_callback == NULL) {
                return;
        }

        if_addr_base = ifreg_addr(dev, ifreg);
        val = sys_read32(if_addr_base + IF_ARB_OFFSET);

        frame.flags = 0;

        if (val & BIT(IF_ARB_XTD_OFFSET)) {
                frame.id = (val >> IF_ARB_IDE_OFFSET) & BIT_MASK(IF_ARB_IDE_WIDTH);
                frame.flags |= CAN_FRAME_IDE;
        } else {
                frame.id = (val >> IF_ARB_ID_OFFSET) & BIT_MASK(IF_ARB_ID_WIDTH);
        }

        val = sys_read32(if_addr_base + IF_MCTL_OFFSET);

        frame.dlc = MIN(8, (val >> IF_MCTL_DLC_OFFSET) & BIT_MASK(IF_MCTL_DLC_WIDTH));
        frame.data_32[0] = sys_cpu_to_le32(sys_read32(if_addr_base + IF_DATA_A_OFFSET));
        frame.data_32[1] = sys_cpu_to_le32(sys_read32(if_addr_base + IF_DATA_B_OFFSET));

        cfg->rx_objects[msg_id].rx_callback(dev, &frame, cfg->rx_objects[msg_id].user_data);
}

static int tms570_can_add_rx_filter(const struct device *dev, can_rx_callback_t callback,
                                    void *user_data, const struct can_filter *filter)
{
        const struct tms570_can_cfg *cfg = dev->config;
        struct tms570_can_data *data = dev->data;
        size_t msg_id;
        int status;
        uint32_t mask_reg;
        uint32_t arb_reg;
        uint32_t mctl_reg;
        int ifreg;
        uintptr_t if_addr_base;

        K_SPINLOCK(&data->lock) {
                status = sys_bitarray_alloc(cfg->rx_bitarray, 1, &msg_id);
        }

        if (status != 0) {
                return -EBUSY;
        }

        cfg->rx_objects[msg_id].rx_callback = callback;
        cfg->rx_objects[msg_id].user_data = user_data;

        ifreg = ifreg_take(dev, K_FOREVER);
        if (ifreg < 0) {
                K_SPINLOCK(&data->lock) {
                        (void)sys_bitarray_clear_bit(cfg->rx_bitarray, msg_id);
                }

                return ifreg;
        }

        if_addr_base = ifreg_addr(dev, ifreg);

        /* Ensure MsgVal is cleared */
        sys_write32(0, if_addr_base + IF_ARB_OFFSET);

        mask_reg = 0;
        arb_reg = 0;
        mctl_reg = 0;

        if (filter->flags & CAN_FRAME_IDE) {
                mask_reg |= BIT(IF_MASK_MXTD_OFFSET);
                mask_reg |= (filter->mask & IF_MASK_EXT_WIDTH) << IF_MASK_EXT_OFFSET;

                arb_reg |= BIT(IF_ARB_XTD_OFFSET);
                arb_reg |= (filter->id & IF_ARB_IDE_WIDTH) << IF_ARB_IDE_OFFSET;
        } else {
                mask_reg |= (filter->mask & IF_MASK_STD_WIDTH) << IF_MASK_STD_OFFSET;
                arb_reg |= (filter->id & IF_ARB_ID_WIDTH) << IF_ARB_ID_OFFSET;
        }

        arb_reg |= BIT(IF_ARB_MSGVAL_OFFSET);
        mctl_reg |= BIT(IF_MCTL_EOB_OFFSET) | BIT(IF_MCTL_RXIE_OFFSET) | BIT(IF_MCTL_UMASK_OFFSET);

        sys_write32(mctl_reg, if_addr_base + IF_MCTL_OFFSET);
        sys_write32(mask_reg, if_addr_base + IF_MASK_OFFSET);
        sys_write32(arb_reg, if_addr_base + IF_ARB_OFFSET);

        ifreg_msgobj_sync(dev, ifreg, MSG_TX_MAX + msg_id, true,
                          IF_CMD_CONTROL_BIT | IF_CMD_MASK_BIT | IF_CMD_ARB_BIT);
        ifreg_give(dev, ifreg);

        return msg_id;
}

static void tms570_can_remove_rx_filter(const struct device *dev, int filteridx)
{
        const struct tms570_can_cfg *cfg = dev->config;
        struct tms570_can_data *data = dev->data;
        int ifreg;

        /* Invalid */
        if (filteridx < 0 || filteridx >= MSG_RX_MAX) {
                return;
        }

        ifreg = ifreg_take(dev, K_FOREVER);
        if (ifreg >= 0) {
                /* Clear MsgVal. */
                sys_write32(0, ifreg_addr(dev, ifreg) + IF_ARB_OFFSET);
                ifreg_msgobj_sync(dev, ifreg, MSG_TX_MAX + filteridx, true, IF_CMD_ARB_BIT);
                ifreg_give(dev, ifreg);
        } else {
                /* This should never happen. */
                __ASSERT_NO_MSG(0);
        }

        K_SPINLOCK(&data->lock) {
                (void)sys_bitarray_clear_bit(cfg->rx_bitarray, filteridx);
        }
}

static enum can_state tms570_can_esr_to_state(const struct device *dev)
{
        uintptr_t ctrl_reg_base = DEVICE_MMIO_GET(dev);
        uint32_t esr = sys_read32(ctrl_reg_base + ES_OFFSET);

        if (esr & BIT(ES_BOFF_OFFSET)) {
                return CAN_STATE_BUS_OFF;
        } else if (esr & BIT(ES_EPASS_OFFSET)) {
                return CAN_STATE_ERROR_PASSIVE;
        } else if (esr & BIT(ES_EWARN_OFFSET)) {
                return CAN_STATE_ERROR_WARNING;
        } else {
                return CAN_STATE_ERROR_ACTIVE;
        }

        return CAN_STATE_STOPPED;
}

static struct can_bus_err_cnt tms570_can_get_err_count(const struct device *dev)
{
        uintptr_t ctrl_reg_base;
        uint32_t errc;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);
        errc = sys_read32(ctrl_reg_base + ERRC_OFFSET);

        return (struct can_bus_err_cnt){
                .rx_err_cnt = (errc >> ERRC_REC_OFFSET) & ERRC_REC_WIDTH,
                .tx_err_cnt = (errc >> ERRC_TEC_OFFSET) & ERRC_TEC_WIDTH,
        };
}

static int tms570_can_get_state(const struct device *dev, enum can_state *state,
                                struct can_bus_err_cnt *err)
{
        struct tms570_can_data *data = dev->data;
        bool started;

        *err = tms570_can_get_err_count(dev);

        K_SPINLOCK(&data->lock) {
                started = data->can_data.started;
        }

        if (!started) {
                *state = CAN_STATE_STOPPED;
        } else {
                *state = tms570_can_esr_to_state(dev);
        }

        return 0;
}

static void tms570_can_set_state_change_callback(const struct device *dev,
                                                 can_state_change_callback_t callback,
                                                 void *user_data)
{
        struct tms570_can_data *data = dev->data;

        K_SPINLOCK(&data->lock) {
                if (!data->can_data.started) {
                        data->can_data.state_change_cb = callback;
                        data->can_data.state_change_cb_user_data = user_data;
                }
        }
}

static int tms570_can_get_core_clock(const struct device *dev, uint32_t *rate)
{
        const struct tms570_can_cfg *cfg = dev->config;

        return clock_control_get_rate(cfg->clk_ctrl, (clock_control_subsys_t)&cfg->clk_domain,
                                      rate);
}

static int tms570_can_get_max_filters(const struct device *dev, bool ide)
{
        ARG_UNUSED(dev);
        ARG_UNUSED(ide);

        return MSG_RX_MAX;
}

static void tms570_can_status_update_isr(const struct device *dev)
{
        struct tms570_can_data *data = dev->data;
        enum can_state state;
        struct can_bus_err_cnt err_cnt;
        uintptr_t ctrl_reg_base;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);
        state = tms570_can_esr_to_state(dev);

        if (state != CAN_STATE_STOPPED && state != data->can_state) {
                data->can_state = state;

                if (data->can_data.state_change_cb != NULL) {
                        err_cnt = tms570_can_get_err_count(dev);

                        data->can_data.state_change_cb(dev, data->can_state, err_cnt,
                                                       data->can_data.state_change_cb_user_data);
                }
        }
}

/**
 * @brief Interrupt service routine
 *
 * Note that only interrupt line 0 is used.
 */
static void tms570_can_isr(const struct device *dev)
{
        uintptr_t ctrl_reg_base;
        uint32_t intsrc;
        int ifreg;
        unsigned int sync_mask;
        bool is_tx;
        size_t msg_id;

        ctrl_reg_base = DEVICE_MMIO_GET(dev);
        intsrc = sys_read32(ctrl_reg_base + INT_OFFSET);
        intsrc = (intsrc >> INT_0ID_OFFSET) & BIT_MASK(INT_0ID_WIDTH);

        if (intsrc == INT_ESR_SOURCE) {
                tms570_can_status_update_isr(dev);
                return;
        } else if (intsrc == 0 || intsrc > MSG_OBJECT_COUNT) {
                /* Invalid interrupt source */
                return;
        }

        ifreg = ifreg_take(dev, K_NO_WAIT);
        if (ifreg < 0) {
                return;
        }

        is_tx = intsrc <= MSG_TX_MAX;

        sync_mask = IF_CMD_CLRINTPND_BIT | IF_CMD_ARB_BIT;
        if (!is_tx) {
                sync_mask |= IF_CMD_DATA_A_BIT | IF_CMD_DATA_B_BIT;
                sync_mask |= IF_CMD_CONTROL_BIT | IF_CMD_NEWDAT_BIT;
                msg_id = intsrc - MSG_TX_MAX - 1;
        } else {
                msg_id = intsrc - 1;
        }

        ifreg_msgobj_sync(dev, ifreg, intsrc - 1, false, sync_mask);

        if (is_tx) {
                tx_done(dev, msg_id, ifreg, 0);
        } else {
                rx_deliver(dev, msg_id, ifreg);
        }

        ifreg_give(dev, ifreg);
}

static DEVICE_API(can, tms570_can_api) = {
        .get_capabilities = tms570_can_get_capabilities,
        .set_mode = tms570_can_set_mode,
        .set_timing = tms570_can_set_timing,
        .start = tms570_can_start,
        .stop = tms570_can_stop,
        .send = tms570_can_send,
        .add_rx_filter = tms570_can_add_rx_filter,
        .remove_rx_filter = tms570_can_remove_rx_filter,
        .get_state = tms570_can_get_state,
        .set_state_change_callback = tms570_can_set_state_change_callback,
        .get_core_clock = tms570_can_get_core_clock,
        .get_max_filters = tms570_can_get_max_filters,
        .timing_min =
                {
                        .prop_seg = 1,
                        .phase_seg1 = 1,
                        .phase_seg2 = 1,
                        .sjw = 1,
                        .prescaler = 1,
                },
        .timing_max =
                {
                        .prop_seg = 8,
                        .phase_seg1 = 8,
                        .phase_seg2 = 8,
                        .sjw = 4,
                        .prescaler = 1024,
                },
};

static int tms570_can_init(const struct device *dev)
{
        const struct tms570_can_cfg *cfg = dev->config;
        struct tms570_can_data *data = dev->data;
        struct can_timing timing;
        int status;
        uintptr_t ctrl_reg_base;

        DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);

        (void)k_sem_init(&data->ifsem, IF_REG_MAX, IF_REG_MAX);
        (void)k_sem_init(&data->txsem, MSG_TX_MAX, MSG_TX_MAX);

        cfg->irq_connect();

        ctrl_reg_base = DEVICE_MMIO_GET(dev);

        /* Set device in initialization mode, allow for configuration change */
        sys_set_bit(ctrl_reg_base + CTL_OFFSET, CTL_INIT_OFFSET);
        sys_set_bit(ctrl_reg_base + CTL_OFFSET, CTL_CCE_OFFSET);

        while (!sys_test_bit(ctrl_reg_base + CTL_OFFSET, CTL_INIT_OFFSET)) {
        }

        status = can_calc_timing(dev, &timing, cfg->can_conf.bitrate, cfg->can_conf.sample_point);
        if (status != 0) {
                LOG_ERR("Failed to calculate timing: %i", status);
                return status;
        }

        status = can_set_timing(dev, &timing);
        if (status != 0) {
                LOG_ERR("Failed to set timing: %i", status);
                return status;
        }

        return 0;
}

#define TMS570_CAN_INIT(inst)                                                                      \
        static void tms570_can_##inst##_irq_connect(void)                                          \
        {                                                                                          \
                IRQ_CONNECT(DT_INST_IRQN(inst), 0, tms570_can_isr, DEVICE_DT_INST_GET(inst), 0);   \
                irq_enable(DT_INST_IRQN(inst));                                                    \
        }                                                                                          \
        SYS_BITARRAY_DEFINE_STATIC(tms570_can_##inst##_tx_bitarray, MSG_TX_MAX);                   \
        static struct tms570_can_msg_object tms570_can_##inst##_tx_objects[MSG_TX_MAX];            \
        SYS_BITARRAY_DEFINE_STATIC(tms570_can_##inst##_rx_bitarray, MSG_RX_MAX);                   \
        static struct tms570_can_msg_object tms570_can_##inst##_rx_objects[MSG_RX_MAX];            \
        const struct tms570_can_cfg tms570_can_##inst##_cfg = {                                    \
                DEVICE_MMIO_ROM_INIT(DT_DRV_INST(inst)),                                           \
                .can_conf = CAN_DT_DRIVER_CONFIG_INST_GET(inst, BITRATE_MIN, BITRATE_MAX),         \
                .clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(inst)),                              \
                .clk_domain = DT_INST_CLOCKS_CELL(inst, clk_id),                                   \
                .tx_objects = tms570_can_##inst##_tx_objects,                                      \
                .tx_bitarray = &tms570_can_##inst##_tx_bitarray,                                   \
                .rx_objects = tms570_can_##inst##_rx_objects,                                      \
                .rx_bitarray = &tms570_can_##inst##_rx_bitarray,                                   \
                .irq_connect = tms570_can_##inst##_irq_connect,                                    \
        };                                                                                         \
        struct tms570_can_data tms570_can_##inst##_data;                                           \
        CAN_DEVICE_DT_INST_DEFINE(inst, tms570_can_init, NULL, &tms570_can_##inst##_data,          \
                                  &tms570_can_##inst##_cfg, POST_KERNEL, CONFIG_CAN_INIT_PRIORITY, \
                                  &tms570_can_api);

DT_INST_FOREACH_STATUS_OKAY(TMS570_CAN_INIT);
