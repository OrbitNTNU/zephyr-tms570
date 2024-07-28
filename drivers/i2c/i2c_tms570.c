
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/pinctrl.h>

#define DT_DRV_COMPAT tms570_i2c

#define OAR_OFFSET (0x00)
#define IMR_OFFSET (0x04)
#define STR_OFFSET (0x08)
#define CKL_OFFSET (0x0c)
#define CKH_OFFSET (0x10)
#define CNT_OFFSET (0x14)
#define DRR_OFFSET (0x18)
#define DXR_OFFSET (0x20)
#define MDR_OFFSET (0x24)
#define IVR_OFFSET (0x28)
#define PSC_OFFSET (0x30)
#define SAR_OFFSET (0x1c)

/* Interrupt mask enable bits */
#define AAS_EN_BIT   (1 << 6)
#define SCD_EN_BIT   (1 << 5)
#define TXRDY_EN_BIT (1 << 4)
#define RXRDY_EN_BIT (1 << 3)

/* Interrupt values (IVR) */
#define AAS_IRQ   (0x7)
#define SCD_IRQ   (0x6)
#define TXRDY_IRQ (0x5)
#define RXRDY_IRQ (0x4)

/* Status register bits */
#define SDIR_BIT  (1 << 14)
#define BB_BIT    (1 << 12)
#define TXRDY_BIT (1 << 4)
#define RXRDY_BIT (1 << 3)
#define AL_BIT    (1 << 0)

/* Mode register bits */
#define STT_BIT (1 << 13)
#define STP_BIT (1 << 11)
#define MST_BIT (1 << 10)
#define TRX_BIT (1 << 9)
#define IRS_BIT (1 << 5)

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
};

static ALWAYS_INLINE bool i2c_tms570_bus_busy(uintptr_t reg_base)
{
        return sys_read32(reg_base + STR_OFFSET) & BB_BIT;
}

static bool i2c_tms570_arb_lost(uintptr_t reg_base)
{
        bool result = sys_read32(reg_base + STR_OFFSET) & AL_BIT;

        /* Clear status bit */
        sys_set_bits(reg_base + STR_OFFSET, AL_BIT);

        return result;
}

static ALWAYS_INLINE bool i2c_tms570_is_controller(uintptr_t reg_base)
{
        return sys_read32(reg_base + MDR_OFFSET) & MST_BIT;
}

static int i2c_tms570_start_transfer(uintptr_t reg_base)
{
        unsigned int i;

        for (i = 0; i < RETRY_ATTEMPTS; i++) {
                while (i2c_tms570_bus_busy(reg_base)) {
                }

                /* Attempt to set device as I2C controller, by setting the MST (master/controller)
                 * bit and STT (START) */
                sys_set_bits(reg_base + MDR_OFFSET, MST_BIT | STT_BIT);

                /* Wait to give time to determine if arbitration is lost or not.
                 * TODO: This is quite dirty, should be improved. Is the start
                 * bit even sent here?*/
                k_busy_wait(1);

                /* Arbitration won, which means we are now acting as controller of bus */
                if (!i2c_tms570_arb_lost(reg_base)) {
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
 */
static void i2c_tms570_end_transfer(uintptr_t reg_base)
{
        while (i2c_tms570_bus_busy(reg_base) || i2c_tms570_is_controller(reg_base)) {
        }
}

/**
 * @brief Set target/slave address of transmission
 *
 * @param dev
 * @param addr
 */
static ALWAYS_INLINE void i2c_tms570_set_sar(uintptr_t reg_base, uint16_t addr)
{
        sys_write32(addr, reg_base + SAR_OFFSET);
}

static void i2c_tms570_set_dir(uintptr_t reg_base, bool dir_rx)
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
static void i2c_tms570_set_count(uintptr_t reg_base, uint16_t count)
{
        sys_write16(count, reg_base + CNT_OFFSET);
        sys_set_bits(reg_base + MDR_OFFSET, STP_BIT);
}

static int i2c_tms570_rx(const struct device *dev, struct i2c_msg *msg)
{
        uintptr_t reg_base;

        reg_base = DEVICE_MMIO_GET(dev);

        for (uint32_t i = 0; i < msg->len; i++) {
                while (!(sys_read32(reg_base + STR_OFFSET) & RXRDY_BIT)) {
                }

                msg->buf[i] = sys_read8(reg_base + DRR_OFFSET);
        }
}

static int i2c_tms570_tx(const struct device *dev, struct i2c_msg *msg)
{
        uintptr_t reg_base;

        reg_base = DEVICE_MMIO_GET(dev);

        for (uint32_t i = 0; i < msg->len; i++) {
                while (!(sys_read32(reg_base + STR_OFFSET) & TXRDY_BIT)) {
                }

                sys_write8(reg_base + DXR_OFFSET, msg->buf[i]);
        }
}

static int i2c_tms570_transfer(const struct device *dev, struct i2c_msg *msgs, uint8_t count,
                               uint16_t addr)
{
        int status;
        uintptr_t reg_base;
        struct i2c_tms570_data *data = dev->data;

        reg_base = DEVICE_MMIO_GET(dev);

        /* This only prevents other threads from starting a transfer, however at this point another
         * controller could initiate a transfer. In that case, we either wait for that to complete
         * before continuing, or we lose arbitration, which again forces us to wait for the other
         * transaction to complete. In that sense, the I2C bus itself should act as a "lock" between
         * acting as controller and acting as target. With that, we don't need to perform any
         * locking in the ISR.*/
        status = k_mutex_lock(&data->lock, TIMEOUT_MSEC);
        if (status != 0) {
                return status;
        }

        for (uint8_t i = 0; i < count; i++) {
                status = i2c_tms570_start_transfer(reg_base);
                if (status != 0) {
                        goto exit;
                }
                i2c_tms570_set_sar(reg_base, addr);
                i2c_tms570_set_dir(reg_base, msgs[i].flags & I2C_MSG_READ);
                i2c_tms570_set_count(reg_base, msgs[i].len);

                if (msgs[i].flags & I2C_MSG_READ) {
                        status = i2c_tms570_rx(dev, &msgs[i]);
                } else {
                        status = i2c_tms570_tx(dev, &msgs[i]);
                }

                i2c_tms570_end_transfer(reg_base);

                if (status != 0) {
                        goto exit;
                }
        }

exit:
        (void)k_mutex_unlock(&data->lock);

        return status;
}

static int i2c_tms570_isr(const struct device *dev)
{
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
static int i2c_tms570_calc_psc(const struct i2c_tms570_cfg *cfg, uint8_t *psc)
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
static int i2c_tms570_calc_mfreq(const struct i2c_tms570_cfg *cfg, uint32_t nominal, uint16_t *ckl,
                                 uint16_t *ckh)
{
        int status;
        uint8_t psc;
        unsigned int d;

        status = i2c_tms570_calc_psc(cfg, &psc);
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

static int i2c_tms570_configure(const struct device *dev, uint32_t config)
{
        int status;
        uintptr_t reg_base;
        uint32_t speed;
        uint16_t ckl;
        uint16_t ckh;
        struct i2c_tms570_data *data = dev->data;

        reg_base = DEVICE_MMIO_GET(dev);

        if (config & I2C_ADDR_10_BITS) {
                return -ENOTSUP;
        }

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

        status = k_mutex_lock(&data->lock, TIMEOUT_MSEC);
        if (status != 0) {
                return status;
        }

        /* Go into reset */
        sys_clear_bits(reg_base + MDR_OFFSET, IRS_BIT);

        /* Set bus speed */
        status = i2c_tms570_calc_mfreq(dev->config, speed, &ckl, &ckh);
        if (status != 0) {
                goto exit;
        }

        sys_write16(ckl, reg_base + CKL_OFFSET);
        sys_write16(ckh, reg_base + CKH_OFFSET);

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
        status = i2c_tms570_calc_psc(cfg, &psc_value);
        if (status != 0) {
                return status;
        }

        sys_write8(psc_value, reg_base + PSC_OFFSET);

        cfg->irq_connect(dev);
}

static int i2c_tms570_target_register(const struct device *dev, struct i2c_target_config *cfg)
{
        /* only enable aas interrupts. no interrupt should be generated when
         * we control the bbus. */
}

static int i2c_tms570_target_unregister(const struct device *dev, struct i2c_target_config *cfg)
{
}

static const struct i2c_driver_api i2c_tms570_driver_api = {
        .transfer = i2c_tms570_transfer,
        .configure = i2c_tms570_configure,

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
