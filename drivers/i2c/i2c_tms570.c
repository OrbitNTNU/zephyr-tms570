
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/sys_clock.h>
#include <zephyr/logging/log.h>

/* in ${ZEPHYR_BASE}/drivers/i2c */
#include "i2c_bitbang.h"

LOG_MODULE_REGISTER(i2c_tms570);

#define DT_DRV_COMPAT tms570_i2c

#define OAR_OFFSET   (0x00)
#define IMR_OFFSET   (0x04)
#define STR_OFFSET   (0x08)
#define CKL_OFFSET   (0x0c)
#define CKH_OFFSET   (0x10)
#define CNT_OFFSET   (0x14)
#define DRR_OFFSET   (0x18)
#define SAR_OFFSET   (0x1c)
#define DXR_OFFSET   (0x20)
#define MDR_OFFSET   (0x24)
#define IVR_OFFSET   (0x28)
#define PSC_OFFSET   (0x30)
#define DMACR_OFFSET (0x3c)
#define PFNC_OFFSET  (0x48)
#define PDIR_OFFSET  (0x4c)
#define DIN_OFFSET   (0x50)
#define DOUT_OFFSET  (0x54)

/* Interrupt mask enable bits */
#define AAS_EN_BIT   (1 << 6)
#define SCD_EN_BIT   (1 << 5)
#define TXRDY_EN_BIT (1 << 4)
#define RXRDY_EN_BIT (1 << 3)
#define IRQ_EN_MASK  (AAS_EN_BIT | SCD_EN_BIT | TXRDY_EN_BIT | RXRDY_EN_BIT)

/* Interrupt values (IVR) */
#define AAS_IRQ   (0x7)
#define SCD_IRQ   (0x6)
#define TXRDY_IRQ (0x5)
#define RXRDY_IRQ (0x4)

/* Status register bits */
#define SDIR_BIT  (1 << 14)
#define BB_BIT    (1 << 12)
#define AAS_BIT   (1 << 9)
#define SCD_BIT   (1 << 5)
#define TXRDY_BIT (1 << 4)
#define RXRDY_BIT (1 << 3)
#define NACK_BIT  (1 << 1)
#define AL_BIT    (1 << 0)

/* Mode register bits */
#define STT_BIT (1 << 13)
#define STP_BIT (1 << 11)
#define MST_BIT (1 << 10)
#define TRX_BIT (1 << 9)
#define IRS_BIT (1 << 5)

/* PFNC register bits */
#define PFNC_BIT (1 << 0)

/* PDIR, DIN and DOUT bits */
#define SDA_BIT (1 << 1)
#define SCL_BIT (1 << 0)

/* From the TRM: module clock frequency must be between 6.7MHz and 13.3 MHz */
#define MOD_CLK_MIN (6700000)
#define MOD_CLK_MAX (13300000)

#define TIMEOUT_MSEC   (K_MSEC(100))
#define RETRY_ATTEMPTS (10)

struct i2c_tms570_cfg {
        DEVICE_MMIO_ROM;

        const struct pinctrl_dev_config *pincfg;
        const struct device *clk_ctrl;
        unsigned int clk_domain;
        void (*irq_connect)(const struct device *dev);

        uint32_t mod_clk_freq;
};

struct i2c_tms570_data {
        DEVICE_MMIO_RAM;

        struct k_mutex lock;

#if defined(CONFIG_I2C_TARGET)
        struct i2c_target_config *target_cfg;
#endif
};

static ALWAYS_INLINE bool is_bus_busy(uintptr_t reg_base)
{
        return sys_read32(reg_base + STR_OFFSET) & BB_BIT;
}

static ALWAYS_INLINE bool is_arb_lost(uintptr_t reg_base)
{
        return sys_read32(reg_base + STR_OFFSET) & AL_BIT;
}

static ALWAYS_INLINE bool is_nack(uintptr_t reg_base)
{
        return sys_read32(reg_base + STR_OFFSET) & NACK_BIT;
}

static ALWAYS_INLINE bool is_controller(uintptr_t reg_base)
{
        return sys_read32(reg_base + MDR_OFFSET) & MST_BIT;
}

static ALWAYS_INLINE bool is_stop(uintptr_t reg_base)
{
        return sys_read32(reg_base + STR_OFFSET) & SCD_EN_BIT;
}

/**
 * @brief Calculate prescaler value for the I2C module clock.
 *
 * Note that this is seperate from the I2C master clock frequency. Per the
 * techincal reference manual, the module clock frequency must be between
 * 6.7MHz and 13.3MHz.
 *
 * @param cfg
 * @param psc
 * @return int
 * @retval 0 Success
 * @retval <0 Negative errno code
 */
static int calc_psc(const struct i2c_tms570_cfg *cfg, uint8_t *psc)
{
        int status;
        uint32_t vclk_freq;

        if (!(cfg->mod_clk_freq > MOD_CLK_MIN && cfg->mod_clk_freq < MOD_CLK_MAX)) {
                return -EINVAL;
        }

        status = clock_control_get_rate(cfg->clk_ctrl, (clock_control_subsys_t)&cfg->clk_domain,
                                        &vclk_freq);
        if (status != 0) {
                return status;
        }

        *psc = vclk_freq / cfg->mod_clk_freq - 1;
        return 0;
}

/**
 * @brief Calculate prescaler for master clock frequency
 *
 * This stores the result in @p ckl and @p ckh, which are the low and
 * high portions of the module clock signal, respectively.
 *
 * @param dev
 * @param nominal
 * @param ckl
 * @param ckh
 * @return int
 * @retval 0 Success
 * @retval <0 Negative errno code
 */
static int calc_mfreq(const struct i2c_tms570_cfg *cfg, uint32_t nominal, uint16_t *ckl,
                      uint16_t *ckh)
{
        int status;
        uint8_t psc;
        unsigned int d;

        status = calc_psc(cfg, &psc);
        if (status != 0) {
                return status;
        }

        switch (psc) {
        case 0:
                d = 7;
                break;
        case 1:
                d = 6;
                break;
        default:
                d = 5;
        }

        *ckl = ((cfg->mod_clk_freq / nominal) - 2 * d) / 2;
        *ckh = *ckl;

        return 0;
}

static uint32_t intr_disable(uintptr_t reg_base)
{
        uint32_t old = sys_read32(reg_base + IMR_OFFSET);
        sys_write32(0, reg_base + IMR_OFFSET);

        return old;
}

static void intr_enable(uintptr_t reg_base, uint32_t mask)
{
        sys_write32(mask, reg_base + IMR_OFFSET);
}

static int xfer_start(uintptr_t reg_base)
{
        unsigned int i;
        k_timepoint_t timeout;

        timeout = sys_timepoint_calc(TIMEOUT_MSEC);

        for (i = 0; i < RETRY_ATTEMPTS; i++) {
                while (is_bus_busy(reg_base)) {
                        if (sys_timepoint_expired(timeout)) {
                                return -ETIMEDOUT;
                        }
                }

                /* Attempt to set device as I2C controller, by setting the
                 * MST (master/controller) bit and STT (START) */
                sys_set_bits(reg_base + MDR_OFFSET, MST_BIT | STT_BIT);

                /* TODO: ARDY may be better suited */
                while (!is_bus_busy(reg_base)) {
                        if (sys_timepoint_expired(timeout)) {
                                return -ETIMEDOUT;
                        }
                }

                if (is_nack(reg_base)) {
                        return -EIO;
                } else if (!is_arb_lost(reg_base)) {
                        /* Successfully taken control of bus */
                        return 0;
                }

                /* Arbitration lost. Clear status flag and try again. */
                sys_set_bits(reg_base + STR_OFFSET, AL_BIT);
        }

        return -EBUSY;
}

/**
 * @brief End transfer, waiting for bus busy bit and MST bit to clear.
 *
 * These must be cleared before initiating another transfer, otherwise the next
 * transfer could be compromised. Note that a stop condition is not manually generated here,
 * as that should already have been done by setting the count and stop bits.
 *
 * @param reg_base
 * @return int
 * @retval 0 Success
 * @retval -ETIMEDOUT Timed out waiting for BB and MST to clear
 */
static int xfer_end(uintptr_t reg_base)
{
        k_timepoint_t timeout = sys_timepoint_calc(TIMEOUT_MSEC);

        while (!is_stop(reg_base) || is_bus_busy(reg_base) || is_controller(reg_base)) {
                if (sys_timepoint_expired(timeout)) {
                        return -ETIMEDOUT;
                }
        }

        /* Clear stop condition */
        sys_set_bits(reg_base + STR_OFFSET, SCD_BIT);

        return 0;
}

/**
 * @brief Set target/slave address of transmission
 *
 * @param dev
 * @param addr
 */
static ALWAYS_INLINE void set_sar(uintptr_t reg_base, uint16_t addr)
{
        sys_write32(addr, reg_base + SAR_OFFSET);
}

static void set_dir(uintptr_t reg_base, bool dir_rx)
{
        if (dir_rx) {
                sys_clear_bits(reg_base + MDR_OFFSET, TRX_BIT);
                return;
        }

        sys_set_bits(reg_base + MDR_OFFSET, TRX_BIT);
}

/**
 * @brief Set the CNT register, and enable the stop condition
 *
 * Setting the count register, and enabling the stop condition, will ensure
 * that the stop condition is generated when the count reaches 0.
 *
 * @param reg_base
 * @param count
 */
static void set_count(uintptr_t reg_base, uint16_t count)
{
        sys_write32(count, reg_base + CNT_OFFSET);

        /* When in non-repeat mode, the stop condition is only generated when
         * CNT reaches 0. The STP bit is therefore set here, to ensure the
         * stop condition is generated later. */
        sys_set_bits(reg_base + MDR_OFFSET, STP_BIT);
}

static void set_pin(uintptr_t reg_base, unsigned int bit, int state)
{
        if (state) {
                sys_set_bits(reg_base + DOUT_OFFSET, bit);
        } else {
                sys_clear_bits(reg_base + DOUT_OFFSET, bit);
        }
}

static void set_sda(void *context, int state)
{
        uintptr_t reg_base = *(uintptr_t *)context;
        set_pin(reg_base, SDA_BIT, state);
};

static void set_scl(void *context, int state)
{
        uintptr_t reg_base = *(uintptr_t *)context;
        set_pin(reg_base, SCL_BIT, state);
};

static int get_sda(void *context)
{
        uintptr_t reg_base = *(uintptr_t *)context;

        return sys_read32(reg_base + DIN_OFFSET) & SDA_BIT;
};

static int rx_msg(const struct device *dev, struct i2c_msg *msg)
{
        uintptr_t reg_base;
        k_timepoint_t timeout;

        reg_base = DEVICE_MMIO_GET(dev);
        timeout = sys_timepoint_calc(TIMEOUT_MSEC);

        for (uint32_t i = 0; i < msg->len; i++) {
                while (!(sys_read32(reg_base + STR_OFFSET) & RXRDY_BIT)) {
                        if (sys_timepoint_expired(timeout)) {
                                return -ETIMEDOUT;
                        }
                }

                msg->buf[i] = (uint8_t)sys_read32(reg_base + DRR_OFFSET);
        }

        return 0;
}

static int tx_msg(const struct device *dev, struct i2c_msg *msg)
{
        uintptr_t reg_base;
        k_timepoint_t timeout;

        reg_base = DEVICE_MMIO_GET(dev);
        timeout = sys_timepoint_calc(TIMEOUT_MSEC);

        for (uint32_t i = 0; i < msg->len; i++) {
                while (!(sys_read32(reg_base + STR_OFFSET) & TXRDY_BIT)) {
                        if (is_nack(reg_base)) {
                                return -EIO;
                        } else if (sys_timepoint_expired(timeout)) {
                                return -ETIMEDOUT;
                        }
                }

                sys_write32(msg->buf[i], reg_base + DXR_OFFSET);
        }

        return 0;
}

static int xfer_msg(const struct device *dev, struct i2c_msg *msg, uint16_t addr)
{
        int status;
        uintptr_t reg_base;

        reg_base = DEVICE_MMIO_GET(dev);

        set_sar(reg_base, addr);
        set_dir(reg_base, msg->flags & I2C_MSG_READ);
        set_count(reg_base, msg->len);

        status = xfer_start(reg_base);
        if (status != 0) {
                goto error;
        }

        if (msg->flags & I2C_MSG_READ) {
                status = rx_msg(dev, msg);
        } else {
                status = tx_msg(dev, msg);
        }

        if (status != 0) {
                goto error;
        }

        status = xfer_end(reg_base);
        if (status != 0) {
                goto error;
        }

        return status;

error:
        (void)i2c_recover_bus(dev);

        return status;
}

static int i2c_tms570_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t count,
                               uint16_t addr)
{
        int status;
        uintptr_t reg_base;
        uint32_t irq_mask;
        struct i2c_tms570_data *data;

        reg_base = DEVICE_MMIO_GET(dev);
        data = dev->data;

        status = k_mutex_lock(&data->lock, K_FOREVER);
        if (status != 0) {
                return status;
        }

        irq_mask = intr_disable(reg_base);

        for (uint8_t i = 0; i < count; i++) {
                status = xfer_msg(dev, &msgs[i], addr);
                if (status != 0) {
                        LOG_ERR("xfer failed with status %i", status);
                        break;
                }
        }

        intr_enable(reg_base, irq_mask);

        (void)k_mutex_unlock(&data->lock);

        return status;
}

static struct i2c_bitbang_io bitbang_io = {
        .set_scl = set_scl,
        .set_sda = set_sda,
        .get_sda = get_sda,
};

static int i2c_tms570_recover_bus(const struct device *dev)
{
        LOG_DBG("attempting to recover bus");

        int status;
        uintptr_t reg_base;
        struct i2c_tms570_data *data = dev->data;
        struct i2c_bitbang bitbang;

        reg_base = DEVICE_MMIO_GET(dev);

        status = k_mutex_lock(&data->lock, K_FOREVER);
        if (status != 0) {
                return status;
        }

        /* Enter reset and enable the pins to be used as GPIO */
        sys_clear_bits(reg_base + MDR_OFFSET, IRS_BIT);
        sys_set_bits(reg_base + PFNC_OFFSET, PFNC_BIT);
        sys_set_bits(reg_base + PDIR_OFFSET, SDA_BIT | SCL_BIT);

        i2c_bitbang_init(&bitbang, &bitbang_io, &reg_base);

        status = i2c_bitbang_recover_bus(&bitbang);
        if (status != 0) {
                LOG_ERR("bus recovery failed: %i", status);
        }

        sys_clear_bits(reg_base + PFNC_OFFSET, PFNC_BIT);
        sys_set_bits(reg_base + MDR_OFFSET, IRS_BIT);

        (void)k_mutex_unlock(&data->lock);

        return status;
}

static int i2c_tms570_configure(const struct device *dev, uint32_t config)
{
        int status;
        uintptr_t reg_base;
        uint32_t speed;
        uint16_t ckl;
        uint16_t ckh;
        struct i2c_tms570_data *data = dev->data;

        reg_base = DEVICE_MMIO_GET(dev);

        speed = I2C_SPEED_GET(config);
        switch (speed) {
        case I2C_SPEED_STANDARD:
                speed = 100000;
                break;
        case I2C_SPEED_FAST:
                speed = 400000;
                break;
        default:
                /* Only up to 400k is supported by this device */
                return -ENOTSUP;
        }

        status = k_mutex_lock(&data->lock, K_FOREVER);
        if (status != 0) {
                return status;
        }

        /* Go into reset */
        sys_clear_bits(reg_base + MDR_OFFSET, IRS_BIT);

        /* Set bus speed */
        status = calc_mfreq(dev->config, speed, &ckl, &ckh);
        if (status != 0) {
                goto exit;
        }

        sys_write32(ckl, reg_base + CKL_OFFSET);
        sys_write32(ckh, reg_base + CKH_OFFSET);

exit:
        /* Go out of reset */
        sys_set_bits(reg_base + MDR_OFFSET, IRS_BIT);

        (void)k_mutex_unlock(&data->lock);

        return status;
}

static int i2c_tms570_init(const struct device *dev)
{
        int status;
        uint8_t psc_value;
        uint32_t reg_base;
        const struct i2c_tms570_cfg *cfg = dev->config;
        struct i2c_tms570_data *data = dev->data;

        DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
        reg_base = DEVICE_MMIO_GET(dev);

        (void)k_mutex_init(&data->lock);

        if (!device_is_ready(cfg->clk_ctrl)) {
                return -ENODEV;
        }

        /* Pinmux is required for the I2C module on the TMS570 */
        status = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
        if (status != 0) {
                return status;
        }

        /* Adjust I2C module clock frequency */
        status = calc_psc(cfg, &psc_value);
        if (status != 0) {
                return status;
        }

        sys_write32(psc_value, reg_base + PSC_OFFSET);

        /* Disable DMA use */
        sys_write32(0, reg_base + DMACR_OFFSET);

        cfg->irq_connect(dev);

        /* Go out of reset */
        sys_set_bits(reg_base + MDR_OFFSET, IRS_BIT);

        return 0;
}

static void i2c_tms570_isr(const struct device *dev)
{
        int status;
        uint8_t byte;
        uint32_t ivr;
        uintptr_t reg_base;
        bool is_tx;
        struct i2c_tms570_data *data;
        const struct i2c_target_callbacks *callbacks;

        reg_base = DEVICE_MMIO_GET(dev);
        data = dev->data;
        callbacks = data->target_cfg->callbacks;

        is_tx = sys_read32(reg_base + STR_OFFSET) & SDIR_BIT;
        ivr = sys_read32(reg_base + IVR_OFFSET);

        if (ivr == TXRDY_IRQ && !is_tx) {
                /* TXRDY is set to 1 at reset */
                return;
        }

        switch (ivr) {
        case AAS_IRQ:
                if (is_tx) {
                        status = callbacks->read_requested(data->target_cfg, &byte);

                        sys_write32(byte, reg_base + DXR_OFFSET);
                } else {
                        /* SDIR=0 means device is either controller
                         * transmitter/receiever or target receiever. As we have
                         * already checked the AAS bit to ensure we are target,
                         * we can safely assume that we are then target receiever.
                         */
                        status = callbacks->write_requested(data->target_cfg);
                        byte = sys_read32(reg_base + DRR_OFFSET);

                        if (status == 0) {
                                status = callbacks->write_received(data->target_cfg, byte);
                        }
                }
                break;
        case SCD_IRQ:
                status = callbacks->stop(data->target_cfg);

                break;
        case TXRDY_IRQ:
                status = callbacks->read_processed(data->target_cfg, &byte);

                sys_write32(byte, reg_base + DXR_OFFSET);
                break;
        case RXRDY_IRQ:
                byte = sys_read32(reg_base + DRR_OFFSET);

                status = callbacks->write_received(data->target_cfg, byte);
                break;
        default:
                CODE_UNREACHABLE;
        }

        if (status != 0) {
                LOG_ERR("irq failed with status %i (mode %" PRIu32 ")", status, ivr);
        }
}

static int i2c_tms570_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
        int status;
        uintptr_t reg_base;
        struct i2c_tms570_data *data;

        reg_base = DEVICE_MMIO_GET(dev);
        data = dev->data;

        status = k_mutex_lock(&data->lock, K_FOREVER);
        if (status != 0) {
                return status;
        }

        if (data->target_cfg != NULL) {
                status = -EBUSY;
                goto exit;
        }

        data->target_cfg = cfg;

        sys_write32(cfg->address, reg_base + OAR_OFFSET);
        intr_enable(reg_base, IRQ_EN_MASK);

exit:
        (void)k_mutex_unlock(&data->lock);

        return status;
}

static int i2c_tms570_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
        int status;
        uintptr_t reg_base;
        struct i2c_tms570_data *data;

        reg_base = DEVICE_MMIO_GET(dev);
        data = dev->data;

        status = k_mutex_lock(&data->lock, K_FOREVER);
        if (status != 0) {
                return status;
        }

        (void)intr_disable(reg_base);
        sys_write32(0, reg_base + OAR_OFFSET);

        data->target_cfg = NULL;

        (void)k_mutex_unlock(&data->lock);

        return status;
}

static const struct i2c_driver_api i2c_tms570_driver_api = {
        .transfer = i2c_tms570_transfer,
        .configure = i2c_tms570_configure,
        .recover_bus = i2c_tms570_recover_bus,

#if defined(CONFIG_I2C_TARGET)
        .target_register = i2c_tms570_target_register,
        .target_unregister = i2c_tms570_target_unregister,
#endif
};

#define I2C_TMS570_INIT(nodeid)                                                                    \
        PINCTRL_DT_INST_DEFINE(nodeid);                                                            \
        static void i2c_tms570_##nodeid##_irq_connect(const struct device *dev)                    \
        {                                                                                          \
                IRQ_CONNECT(DT_INST_IRQN(nodeid), 0, i2c_tms570_isr, DEVICE_DT_INST_GET(nodeid),   \
                            0);                                                                    \
                irq_enable(DT_INST_IRQN(nodeid));                                                  \
        }                                                                                          \
        static const struct i2c_tms570_cfg i2c_tms570_##nodeid##_cfg = {                           \
                DEVICE_MMIO_ROM_INIT(DT_DRV_INST(nodeid)),                                         \
                .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(nodeid),                                  \
                .clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(nodeid)),                            \
                .clk_domain = DT_CLOCKS_CELL(DT_DRV_INST(nodeid), clk_id),                         \
                .irq_connect = i2c_tms570_##nodeid##_irq_connect,                                  \
                .mod_clk_freq = DT_INST_PROP(nodeid, module_clock_frequency),                      \
        };                                                                                         \
        static struct i2c_tms570_data i2c_tms570_##nodeid##_data;                                  \
        I2C_DEVICE_DT_INST_DEFINE(nodeid, i2c_tms570_init, NULL, &i2c_tms570_##nodeid##_data,      \
                                  &i2c_tms570_##nodeid##_cfg, POST_KERNEL,                         \
                                  CONFIG_I2C_INIT_PRIORITY, &i2c_tms570_driver_api);

DT_INST_FOREACH_STATUS_OKAY(I2C_TMS570_INIT);
