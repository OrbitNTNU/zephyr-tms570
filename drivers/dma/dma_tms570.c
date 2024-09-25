
#include <zephyr/drivers/dma.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/device_mmio.h>

LOG_MODULE_REGISTER(dma_tms570);

#define DEV_DMA        DT_INST(0, tms570_dma)
#define DEV_CFG(_dev)  ((const struct dma_tms570_cfg *)(_dev)->config)
#define DEV_DATA(_dev) ((struct dma_tms570_data *)(_dev)->data)

#define NUM_CHANNELS (DT_PROP(DEV_DMA, dma_channels))
#define NUM_REQUESTS (DT_PROP(DEV_DMA, dma_requests))

#define GCTRL_OFFSET      (0x0)   /* Global control register */
#define STAT_OFFSET       (0xc)   /* Channel status offset */
#define HWCHENAS_OFFSET   (0x14)  /* HW Channel enable set */
#define HWCHENAR_OFFSET   (0x1c)  /* HW Channel enable reset */
#define SWCHENAS_OFFSET   (0x24)  /* SW Channel enable set */
#define SWCHENAR_OFFSET   (0x2c)  /* SW Channel enable reset */
#define CHPRIOR_OFFSET    (0x3c)  /* Channel priority */
#define GCHIENAS_OFFSET   (0x44)  /* Global channel interrupt enable */
#define GCHIENAR_OFFSET   (0x4c)  /* Global channel interrupt disable */
#define REQASI_OFFSET     (0x54)  /* Request assignment offset */
#define PAR0_OFFSET       (0x94)  /* Port assignment 0 register */
#define PAR1_OFFSET       (0x98)  /* Port assignment 1 register */
#define FTCINTENAS_OFFSET (0xdc)  /* Frame transfer complete interrupt enable set */
#define FTCINTENAR_OFFSET (0xe4)  /* Frame transfer complete interrupt enable reset */
#define BTCINTENAS_OFFSET (0x10c) /* Block transfer complete interrupt enable set */
#define BTCINTENAR_OFFSET (0x114) /* Block transfer complete interrupt enable reset */
#define FTCFLAG_OFFSET    (0x124) /* Frame transfer complete flag */
#define BTCFLAG_OFFSET    (0x13c) /* Block transfer complete interrupt flag */
#define FTCA_OFFSET       (0x14c) /* Frame transfer complete channel pending interrupt */
#define BTCA_OFFSET       (0x158) /* Block transfer complete channel pending interrupt */
#define PCTRL_OFFSET      (0x178) /* Port control */
#define PBACTC_OFFSET     (0x1a0)

#define DMA_EN_BIT (16)

/* PCTRL bits */
#define BYB_BIT      (18) /* Bypass FIFO */
#define PSFRHQPB_BIT (17) /* Priority scheme for high priority queue */
#define PSFRLQPB_BIT (16) /* Priority scheme for low priority queue */

#define PRIO_SCH_FIXED (0)
#define PRIO_SCH_ROT   (1)

/* Number of channels in each request assignment register */
#define CHAN_PER_REQASI (4)

/* Size, offset of primary control packets */
#define PCP_SIZE   (32)
#define PCP_OFFSET (0x0)

/* Primary control packet offsets.
 * NOTE: These are relative to the address returned by `get_pcp`
 */
#define PCP_ISADDR_OFFSET  (0x00)
#define PCP_IDADDR_OFFSET  (0x04)
#define PCP_ITCOUNT_OFFSET (0x08)
#define PCP_CHCTRL_OFFSET  (0x10)
#define PCP_EIOFF_OFFSET   (0x14)
#define PCP_FIOFF_OFFSET   (0x18)

/* PCP CHCTL Bits */
#define RES_BIT_OFFSET   (14)
#define WES_BIT_OFFSET   (12)
#define TTYPE_BIT_OFFSET (8)
#define ADDMR_BIT_OFFSET (3)
#define ADDMW_BIT_OFFSET (1)
#define AIM_BIT          (0)

/* PCP Itcount bits */
#define IFT_BIT_OFFSET (16)
#define IET_BIT_OFFSET (0)

/* Size, offset of working control packet */
#define WCP_SIZE   (16)
#define WCP_OFFSET (0x800)

/* Working control packet offsets.
 * NOTE: These are relative to the address returned by `get_wcp`. */
#define WCP_CSADDR_OFFSET  (0x0)
#define WCP_CDADDR_OFFSET  (0x4)
#define WCP_CTCOUNT_OFFSET (0x8)

/* WCP CTCOUNT bits */
#define CFT_BIT_OFFSET (16)
#define CET_BIT_OFFSET (0)

struct dma_tms570_cfg {
        DEVICE_MMIO_NAMED_ROM(control);
        DEVICE_MMIO_NAMED_ROM(packets);

        void (*irq_connect)(const struct device *);
};

struct dma_tms570_channel {
        bool used;
        bool cyclic;
        uint8_t elem_size;
        uint8_t slot;
        uint8_t direction;

        dma_callback_t callback;
        void *user_data;
};

struct dma_tms570_data {
        DEVICE_MMIO_NAMED_RAM(control);
        DEVICE_MMIO_NAMED_RAM(packets);

        struct dma_tms570_channel channels[NUM_CHANNELS];
};

static inline uintptr_t get_pcp(uintptr_t reg_base, uint8_t ch)
{
        return reg_base + PCP_OFFSET + ch * PCP_SIZE;
}

static inline uintptr_t get_wcp(uintptr_t reg_base, uint8_t ch)
{
        return reg_base + WCP_OFFSET + WCP_SIZE * ch;
}

/* NOTE: Indexed addressing mode is currently unsupported */
static int get_addr_adj(uint32_t mode, bool mode_read)
{
        int mask;

        switch (mode) {
        case DMA_ADDR_ADJ_DECREMENT:
                LOG_ERR("addr decrement unsupported");
                return -ENOTSUP;
        case DMA_ADDR_ADJ_NO_CHANGE:
                mask = 0;
                break;
        case DMA_ADDR_ADJ_INCREMENT:
                mask = 0x1;
                break;
        default:
                CODE_UNREACHABLE;
        }

        return mask << (mode_read ? ADDMR_BIT_OFFSET : ADDMW_BIT_OFFSET);
}

static bool dir_valid(uint32_t dir)
{
        /* Currently only peripheral to/from memory is supported */
        switch (dir) {
        case PERIPHERAL_TO_MEMORY:
        case MEMORY_TO_PERIPHERAL:
        case MEMORY_TO_MEMORY:
                return true;
        default:
                break;
        }

        return false;
}

static int get_elem_size(uint32_t size, bool dir_read)
{
        if (size > 4) {
                LOG_ERR("element transfer sizes higher than 64 is not supported");
                return -EINVAL;
        }

        return (size - 1) << (dir_read ? RES_BIT_OFFSET : WES_BIT_OFFSET);
}

static int set_pcp_ctrl_mask(const struct device *dev, uint32_t ch_id, struct dma_config *dma_cfg)
{
        int ctrl_mask;
        int addr_adj;
        int elem_size;
        uintptr_t cp_reg_base;

        ctrl_mask = 0;
        cp_reg_base = get_pcp(DEVICE_MMIO_NAMED_GET(dev, packets), ch_id);

        addr_adj = get_addr_adj(dma_cfg->head_block->source_addr_adj, true);
        if (addr_adj < 0) {
                return addr_adj;
        }
        ctrl_mask |= addr_adj;

        addr_adj = get_addr_adj(dma_cfg->head_block->dest_addr_adj, false);
        if (addr_adj < 0) {
                return addr_adj;
        }
        ctrl_mask |= addr_adj;

        elem_size = get_elem_size(dma_cfg->source_data_size, true);
        if (elem_size < 0) {
                return elem_size;
        }
        ctrl_mask |= elem_size;

        elem_size = get_elem_size(dma_cfg->dest_data_size, false);
        if (elem_size < 0) {
                return elem_size;
        }
        ctrl_mask |= elem_size;

        /* Transfer block by block in mem<->mem mode, otherwise transfer frame
         * by frame. */
        switch (dma_cfg->channel_direction) {
        case MEMORY_TO_MEMORY:
                ctrl_mask |= 1 << TTYPE_BIT_OFFSET;
                break;
        default:
                break;
        }

        if (dma_cfg->head_block->source_reload_en || dma_cfg->head_block->dest_reload_en) {
                if (dma_cfg->head_block->source_reload_en != dma_cfg->head_block->dest_reload_en) {
                        LOG_WRN("reloading only source or dest is not supported");
                }

                ctrl_mask |= 1 << AIM_BIT;
        }

        sys_write32(ctrl_mask, cp_reg_base + PCP_CHCTRL_OFFSET);

        return 0;
}

/* We are currently only supporting periph to/from memory, so this only sets
 * the HW enable bit for the channel. The peripheral itself should generate
 * the DMA Request, so this function does not do that from SW. */
static void channel_start(const struct device *dev, uint32_t ch_id, uint32_t dir)
{
        uintptr_t ctrl_reg_base;

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        switch (dir) {
        case MEMORY_TO_MEMORY:
                sys_write32(1 << ch_id, ctrl_reg_base + SWCHENAS_OFFSET);
                break;
        case PERIPHERAL_TO_MEMORY:
        case MEMORY_TO_PERIPHERAL:
                sys_write32(1 << ch_id, ctrl_reg_base + HWCHENAS_OFFSET);
                break;
        default:
                __ASSERT_NO_MSG(0);
        }
}

static void channel_stop(const struct device *dev, uint32_t ch_id, uint32_t dir)
{
        uintptr_t ctrl_reg_base;

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        switch (dir) {
        case MEMORY_TO_MEMORY:
                sys_write32(1 << ch_id, ctrl_reg_base + SWCHENAR_OFFSET);
                break;
        case PERIPHERAL_TO_MEMORY:
        case MEMORY_TO_PERIPHERAL:
                sys_write32(1 << ch_id, ctrl_reg_base + HWCHENAR_OFFSET);
                break;
        default:
                __ASSERT_NO_MSG(0);
        }
}

static void set_addresses(const struct device *dev, uint32_t ch_id, uint32_t src, uint32_t dest)
{
        uintptr_t cp_reg_base;

        cp_reg_base = get_pcp(DEVICE_MMIO_NAMED_GET(dev, packets), ch_id);

        if (src != 0) {
                sys_write32(src, cp_reg_base + PCP_ISADDR_OFFSET);
        }

        if (dest != 0) {
                sys_write32(dest, cp_reg_base + PCP_IDADDR_OFFSET);
        }
}

static void set_size(const struct device *dev, uint32_t ch_id, uint32_t size)
{
        uintptr_t cp_reg_base;
        uint32_t frame_count;
        struct dma_tms570_data *data = dev->data;
        struct dma_tms570_channel *channel = &data->channels[ch_id];

        cp_reg_base = get_pcp(DEVICE_MMIO_NAMED_GET(dev, packets), ch_id);

        /* Set transfer count. This sets the number of frame transfers to the
         * block size / read size, and the number of element transfers to 1. */
        frame_count = size / channel->elem_size;
        sys_write32((frame_count << IFT_BIT_OFFSET) | (1 << IET_BIT_OFFSET),
                    cp_reg_base + PCP_ITCOUNT_OFFSET);
}

static void set_prio(const struct device *dev, uint32_t ch_id, uint8_t prio)
{
        uintptr_t ctrl_reg_base;

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        sys_clear_bits(ctrl_reg_base + CHPRIOR_OFFSET, 1 << ch_id);
        sys_set_bits(ctrl_reg_base + CHPRIOR_OFFSET, 1 << ch_id);
}

static void assign_request(const struct device *dev, uint32_t ch_id, uint8_t slot)
{
        uintptr_t ctrl_reg_base;
        uint32_t offset_bytes;
        uint32_t offset_bits;

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        /* Only 5 bits are used for the request line */
        slot &= (1 << 5) - 1;

        offset_bytes = (ch_id / CHAN_PER_REQASI) * sizeof(uint32_t);
        offset_bits = 32 - (((ch_id % CHAN_PER_REQASI) + 1) * (32 / CHAN_PER_REQASI));

        sys_clear_bits(ctrl_reg_base + REQASI_OFFSET + offset_bytes, 0xff << offset_bits);
        sys_set_bits(ctrl_reg_base + REQASI_OFFSET + offset_bytes, slot << offset_bits);
}

static int dma_tms570_config(const struct device *dev, uint32_t ch_id, struct dma_config *dma_cfg)
{
        int status = 0;
        int lock_key;
        struct dma_block_config *head = dma_cfg->head_block;
        struct dma_tms570_data *data = dev->data;
        struct dma_tms570_channel *channel;

        if (dma_cfg->block_count > 1) {
                LOG_WRN("only one block at a time is currently supported");
        }

        if (dma_cfg->dma_slot >= NUM_REQUESTS) {
                LOG_ERR("dma slot %" PRIu32 " exceeds request line maximum (%u)", dma_cfg->dma_slot,
                        (unsigned int)NUM_REQUESTS);
                return -EINVAL;
        }

        if (dma_cfg->source_chaining_en || dma_cfg->dest_chaining_en) {
                LOG_ERR("chaining is currently unsupported");
                return -EINVAL;
        }

        if (!dir_valid(dma_cfg->channel_direction)) {
                LOG_ERR("only periph to/from memory is supported");
                return -EINVAL;
        }

        if (ch_id >= NUM_CHANNELS) {
                LOG_ERR("invalid channel %" PRIu32 " (max %i)", ch_id, NUM_CHANNELS);
                return -EINVAL;
        }

        lock_key = irq_lock();

        channel = &data->channels[ch_id];
        if (channel->used) {
                LOG_ERR("channel %" PRIu32 " already in use", ch_id);
                status = -EBUSY;
                goto exit;
        }

        channel->callback = dma_cfg->dma_callback;
        channel->user_data = dma_cfg->user_data;
        channel->elem_size = dma_cfg->source_data_size;
        channel->slot = dma_cfg->dma_slot;
        channel->cyclic = dma_cfg->cyclic;
        channel->direction = dma_cfg->channel_direction;

        set_addresses(dev, ch_id, head->source_address, head->dest_address);
        set_size(dev, ch_id, head->block_size);
        set_prio(dev, ch_id, dma_cfg->channel_priority & 1);

        status = set_pcp_ctrl_mask(dev, ch_id, dma_cfg);
        if (status != 0) {
                goto exit;
        }

        assign_request(dev, ch_id, channel->slot);

        channel->used = true;

exit:
        irq_unlock(lock_key);
        return status;
}

static int dma_tms570_start(const struct device *dev, uint32_t ch_id)
{
        uintptr_t ctrl_reg_base;
        struct dma_tms570_data *data = dev->data;
        int key = irq_lock();

        if (ch_id >= NUM_CHANNELS) {
                irq_unlock(key);
                return -EINVAL;
        }

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        sys_write32(1 << ch_id, ctrl_reg_base + GCHIENAS_OFFSET);
        sys_write32(1 << ch_id, ctrl_reg_base + BTCINTENAS_OFFSET);

        if (data->channels[ch_id].cyclic) {
                sys_write32(1 << ch_id, ctrl_reg_base + FTCINTENAS_OFFSET);
        }

        channel_start(dev, ch_id, data->channels[ch_id].direction);
        irq_unlock(key);

        return 0;
}

static int dma_tms570_stop(const struct device *dev, uint32_t ch_id)
{
        uintptr_t ctrl_reg_base;
        struct dma_tms570_data *data = dev->data;
        int key = irq_lock();

        if (ch_id >= NUM_CHANNELS) {
                irq_unlock(key);
                return -EINVAL;
        }

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        channel_stop(dev, ch_id, data->channels[ch_id].direction);
        sys_write32(1 << ch_id, ctrl_reg_base + BTCINTENAR_OFFSET);
        sys_write32(1 << ch_id, ctrl_reg_base + GCHIENAR_OFFSET);

        if (data->channels[ch_id].cyclic) {
                sys_write32(1 << ch_id, ctrl_reg_base + FTCINTENAR_OFFSET);
        }

        data->channels[ch_id].used = false;

        irq_unlock(key);
        return 0;
}

static int dma_tms570_reload(const struct device *dev, uint32_t ch_id, uint32_t src, uint32_t dst,
                             size_t size)
{
        int key = irq_lock();

        if (ch_id >= NUM_CHANNELS) {
                irq_unlock(key);
                return -EINVAL;
        }

        set_addresses(dev, ch_id, src, dst);
        set_size(dev, ch_id, size);

        irq_unlock(key);
        return 0;
}

static int dma_tms570_get_status(const struct device *dev, uint32_t ch_id, struct dma_status *stat)
{
        uintptr_t ctrl_reg_base;
        uintptr_t wcp_reg_base;
        uint32_t frames_remain;
        struct dma_tms570_data *data = dev->data;

        if (ch_id >= NUM_CHANNELS) {
                return -EINVAL;
        }

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);
        wcp_reg_base = get_wcp(DEVICE_MMIO_NAMED_GET(dev, packets), ch_id);

        stat->busy = (sys_read32(ctrl_reg_base + STAT_OFFSET) >> ch_id) & 1;
        stat->dir = data->channels[ch_id].direction;

        /* This assumes that element is always one */
        frames_remain = sys_read32(wcp_reg_base + WCP_CTCOUNT_OFFSET) >> CFT_BIT_OFFSET;
        stat->pending_length = frames_remain * data->channels[ch_id].elem_size;

        return 0;
}

/* The DMA controller of the TMS570 is somewhat lacking in that there is no
 * reliable way to get the count of transferred bytes for the given channel.
 * This is due to the CTCOUNT register of the control packets is only updated
 * when the channel is arbitrated out of the priority queue - but that is not
 * always guaranteed to happen if there is no other work pending.
 *
 * To solve this we use a constantly repeating "dumb" channel, which ensures
 * that there is always work to evict channels from the priority queue. This
 * way, we can read the count from the CTCOUNT register. */
static int force_arbitration(const struct device *dev, uint32_t ch_id)
{
        static uint8_t tmpval;
        static uint8_t buf[10];

        int status;
        uintptr_t ctrl_reg_base;
        struct dma_block_config block = {
                .source_address = (uint32_t)buf,
                .dest_address = (uint32_t)&tmpval,
                .block_size = sizeof(buf),
                .dest_addr_adj = DMA_ADDR_ADJ_NO_CHANGE,
                .source_addr_adj = DMA_ADDR_ADJ_INCREMENT,
                .source_reload_en = 1,
                .dest_reload_en = 1,
        };
        struct dma_config cfg = {
                .head_block = &block,
                .block_count = 1,
                .source_data_size = 1,
                .dest_data_size = 1,
                .channel_direction = MEMORY_TO_MEMORY,
                .channel_priority = 0,
        };

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        status = dma_config(dev, ch_id, &cfg);
        if (status != 0) {
                return status;
        }

        channel_start(dev, ch_id, MEMORY_TO_MEMORY);
        return 0;
}

static int dma_tms570_init(const struct device *dev)
{
        int status;
        uintptr_t ctrl_reg_base;
        const struct dma_tms570_cfg *cfg = dev->config;

        DEVICE_MMIO_NAMED_MAP(dev, control, K_MEM_CACHE_NONE);
        DEVICE_MMIO_NAMED_MAP(dev, packets, K_MEM_CACHE_NONE);

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        cfg->irq_connect(dev);

        sys_write32(1 << DMA_EN_BIT, ctrl_reg_base + GCTRL_OFFSET);
        sys_write32(1 << BYB_BIT | PRIO_SCH_FIXED << PSFRHQPB_BIT | PRIO_SCH_ROT << PSFRLQPB_BIT,
                    ctrl_reg_base + PCTRL_OFFSET);

        sys_write32(UINT32_MAX, ctrl_reg_base + PAR0_OFFSET);
        sys_write32(UINT32_MAX, ctrl_reg_base + PAR0_OFFSET);

        status = force_arbitration(dev, 0);
        if (status != 0) {
                LOG_ERR("force arbitration failed");
                return status;
        }

        return 0;
}

static int dma_tms570_isr(const struct device *dev)
{
        uintptr_t ctrl_reg_base;
        uint32_t btcflag;
        uint32_t ftcflag;
        unsigned int status;
        int i;
        struct dma_tms570_data *data = dev->data;
        struct dma_tms570_channel *channel;

        ctrl_reg_base = DEVICE_MMIO_NAMED_GET(dev, control);

        btcflag = sys_read32(ctrl_reg_base + BTCFLAG_OFFSET);
        ftcflag = sys_read32(ctrl_reg_base + FTCFLAG_OFFSET);

        for (i = 0; (btcflag || ftcflag) && i < NUM_CHANNELS; i++, btcflag >>= 1, ftcflag >>= 1) {
                channel = &data->channels[i];

                if (channel->used && ((ftcflag | btcflag) & 1)) {
                        if (ftcflag & 1) {
                                if (channel->cyclic) {
                                        status = DMA_STATUS_BLOCK;
                                }

                                sys_write32(1 << i, ctrl_reg_base + FTCFLAG_OFFSET);
                        }

                        if (btcflag & 1) {
                                if (channel->cyclic) {
                                        status = DMA_STATUS_COMPLETE;
                                } else {
                                        status = DMA_STATUS_BLOCK;
                                }

                                sys_write32(1 << i, ctrl_reg_base + BTCFLAG_OFFSET);
                        }

                        if (channel->callback != NULL) {
                                channel->callback(dev, channel->user_data, i, status);
                        }
                }
        }

        return 0;
}

static void dma_tms570_irq_connect(const struct device *dev)
{
        ARG_UNUSED(dev);

        IRQ_CONNECT(DT_IRQN(DEV_DMA), 0, dma_tms570_isr, DEVICE_DT_GET(DEV_DMA), 0);
        irq_enable(DT_IRQN(DEV_DMA));

#if DT_NUM_IRQS(DEV_DMA) > 0
        IRQ_CONNECT(DT_IRQN_BY_IDX(DEV_DMA, 1), 0, dma_tms570_isr, DEVICE_DT_GET(DEV_DMA), 0);
        irq_enable(DT_IRQN_BY_IDX(DEV_DMA, 1));
#endif
}

static const struct dma_driver_api dma_tms570_driver_api = {
        .config = dma_tms570_config,
        .reload = dma_tms570_reload,
        .start = dma_tms570_start,
        .stop = dma_tms570_stop,
        .get_status = dma_tms570_get_status,
};

static const struct dma_tms570_cfg dma_tms570_cfg = {
        DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(control, DEV_DMA),
        DEVICE_MMIO_NAMED_ROM_INIT_BY_NAME(packets, DEV_DMA),

        .irq_connect = dma_tms570_irq_connect,
};

static struct dma_tms570_data dma_tms570_data;

DEVICE_DT_DEFINE(DEV_DMA, dma_tms570_init, NULL, &dma_tms570_data, &dma_tms570_cfg, PRE_KERNEL_1,
                 CONFIG_DMA_INIT_PRIORITY, &dma_tms570_driver_api);
