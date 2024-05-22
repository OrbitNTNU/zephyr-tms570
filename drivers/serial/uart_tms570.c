
#include <zephyr/arch/cpu.h>
#include <zephyr/device.h>
#include <zephyr/sys/device_mmio.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/irq.h>

#define DT_DRV_COMPAT tms570_uart

#define CGR0_OFFSET     (0x00)
#define CGR1_OFFSET     (0x04)
#define SETINT_OFFSET   (0x0c)
#define CLEARINT_OFFSET (0x10)
#define FLAGS_OFFSET    (0x1c)
#define BRS_OFFSET      (0x2c)
#define FORMAT_OFFSET   (0x28)
#define RDBUF_OFFSET    (0x34)
#define TDBUF_OFFSET    (0x38)
#define PIO0_OFFSET     (0x3c)

/* SCIGCR0 bits */
#define RESET_BIT (1 << 0)

/* SCIGCR1 bits */
#define CLOCK_BIT    (1 << 5)
#define SWnRST_BIT   (1 << 7)
#define LOOPBACK_BIT (1 << 16)
#define CONT_BIT     (1 << 17)
#define RXENA_BIT    (1 << 24)
#define TXENA_BIT    (1 << 25)
#define ASYNC_BIT    (1 << 1)

/* SCIPIO0 bits */
#define RXFUNC_BIT (1 << 1)
#define TXFUNC_BIT (1 << 2)

/* Flags/FLR bits */
#define TXRDY_BIT   (1 << 8)
#define RXRDY_BIT   (1 << 9)
#define TXEMPTY_BIT (1 << 11)

/* SETINT/CLEARINT bits */
#define TXINT_BIT   (1 << 8)
#define RXINT_BIT   (1 << 9)
/* Framing error interrupt, overrun error interrupt and parity error interrupt */
#define ERRINT_BITS ((1 << 26) | (1 << 25) | (1 << 24))

#define FRAME7_BITS (0x7)

struct uart_tms570_cfg {
        DEVICE_MMIO_ROM;
        uint32_t baud;

        const struct pinctrl_dev_config *pincfg;
        const struct device *clk_ctrl;
        unsigned int clk_domain;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        void (*irq_connect)(const struct device *);
#endif
};

struct uart_tms570_data {
        DEVICE_MMIO_RAM;

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        uart_irq_callback_user_data_t cb;
        void *cb_data;
#endif
};

/**
 * @brief Calculate the value for the BAUD register, based on the nominal baud
 * rate @p cfg->baud
 *
 * @param cfg
 * @return uint32_t Converted value, which can be set in the BAUD register
 */
static uint32_t calc_baud_reg(const struct uart_tms570_cfg *cfg)
{
        uint32_t rate;
        int ret;

        ret = clock_control_get_rate(cfg->clk_ctrl, (clock_control_subsys_t)&cfg->clk_domain,
                                     &rate);
        if (ret < 0) {
                __ASSERT(0, "clock control get rate failed");
                return 0;
        }

        return rate / (cfg->baud << 4) - 1;
}

static inline int is_ready(uintptr_t reg_base, uint32_t mask)
{
        return sys_read32(reg_base + FLAGS_OFFSET) & mask;
}

/**
 * @brief Initialize UART device @p dev
 *
 * This implements the steps outlined in TMS570LS1224 technical reference
 * manual, section 30.5.
 *
 * @param dev
 * @return int Always 0
 */
static int uart_tms570_init(const struct device *dev)
{
        int status;
        uint32_t tmp;
        uintptr_t reg_base;
        const struct uart_tms570_cfg *cfg = dev->config;

        DEVICE_MMIO_MAP(dev, K_MEM_CACHE_NONE);
        reg_base = DEVICE_MMIO_GET(dev);

        status = pinctrl_apply_state(cfg->pincfg, PINCTRL_STATE_DEFAULT);
        if (status < 0) {
                return status;
        }

        /* Set RESET bit to 1 */
        sys_set_bits(reg_base + CGR0_OFFSET, RESET_BIT);

        /* Set SWnRST bit to 0. Only when this bit is 0 can the SCI registers
         * be configured. This is set back to 1 at the end of the function */
        sys_clear_bits(reg_base + CGR1_OFFSET, SWnRST_BIT);

        sys_write32(FRAME7_BITS, reg_base + FORMAT_OFFSET);

        /* Configure TX and RX pins */
        sys_set_bits(reg_base + PIO0_OFFSET, TXFUNC_BIT | RXFUNC_BIT);

        /* Set the BAUD register */
        sys_write32(calc_baud_reg(cfg), reg_base + BRS_OFFSET);

        /* Enable clock. Enable CONT bit for emulation environment.
         * Additionally, enable LOOPBACK bit for use in a self test. */
        tmp = CLOCK_BIT | CONT_BIT | ASYNC_BIT;
        tmp |= TXENA_BIT | RXENA_BIT;

        sys_write32(tmp, reg_base + CGR1_OFFSET);

        /* Set the SWnRST bit back to 1. This prevents further configuration
         * of the peripheral. */
        sys_set_bits(reg_base + CGR1_OFFSET, SWnRST_BIT);

        return 0;
}

/**
 * @brief Receive data from the UART peripheral, using the polling method
 *
 * @param dev
 * @param c Buffer where incoming character is stored
 * @return int Always 0
 */
static int uart_tms570_poll_in(const struct device *dev, unsigned char *c)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);

        while (!is_ready(reg_base, RXRDY_BIT)) {
        }

        *c = (unsigned char)sys_read32(reg_base + RDBUF_OFFSET);

        return 0;
}

/**
 * @brief Send data to the UART peripheral, using the polling method
 *
 * @param dev
 * @param c Character to send
 */
static void uart_tms570_poll_out(const struct device *dev, unsigned char c)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);

        while (!is_ready(reg_base, TXRDY_BIT)) {
        }

        sys_write32(c, reg_base + TDBUF_OFFSET);
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
static int uart_tms570_fifo_fill(const struct device *dev, const uint8_t *data, int size)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);
        int i = 0;

        for (; i < size && is_ready(reg_base, TXRDY_BIT); i++) {
                sys_write32(data[i], reg_base + TDBUF_OFFSET);
        }

        return i;
}

static int uart_tms570_fifo_read(const struct device *dev, uint8_t *buf, const int size)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);
        int i = 0;

        for (; i < size && is_ready(reg_base, RXRDY_BIT); i++) {
                buf[i] = sys_read32(reg_base + RDBUF_OFFSET);
        }

        return i;
}

static void uart_tms570_irq_tx_enable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + SETINT_OFFSET, TXINT_BIT);
}

static void uart_tms570_irq_tx_disable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + CLEARINT_OFFSET, TXINT_BIT);
}

static int uart_tms570_irq_tx_ready(const struct device *dev)
{
        return is_ready(DEVICE_MMIO_GET(dev), TXRDY_BIT);
}

static int uart_tms570_irq_tx_complete(const struct device *dev)
{
        return sys_read32(DEVICE_MMIO_GET(dev) + FLAGS_OFFSET) & TXEMPTY_BIT;
}

static void uart_tms570_irq_rx_enable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + SETINT_OFFSET, RXINT_BIT);
}

static void uart_tms570_irq_rx_disable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + CLEARINT_OFFSET, RXINT_BIT);
}

static int uart_tms570_irq_rx_ready(const struct device *dev)
{
        return is_ready(DEVICE_MMIO_GET(dev), RXRDY_BIT);
}

static void uart_tms570_irq_err_enable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + SETINT_OFFSET, ERRINT_BITS);
}

static void uart_tms570_irq_err_disable(const struct device *dev)
{
        sys_set_bits(DEVICE_MMIO_GET(dev) + CLEARINT_OFFSET, ERRINT_BITS);
}

static int uart_tms570_irq_is_pending(const struct device *dev)
{
        uintptr_t reg_base = DEVICE_MMIO_GET(dev);

        return is_ready(reg_base, TXRDY_BIT) || is_ready(reg_base, RXRDY_BIT);
}

static int uart_tms570_irq_update(const struct device *dev)
{
        ARG_UNUSED(dev);
        return 1;
}

static void uart_tms570_irq_callback_set(const struct device *dev, uart_irq_callback_user_data_t cb,
                                         void *user_data)
{
        struct uart_tms570_data *data = dev->data;

        data->cb = cb;
        data->cb_data = user_data;
}

static void uart_tms570_isr(const struct device *dev)
{
        struct uart_tms570_data *data = dev->data;

        if (data->cb != NULL) {
                data->cb(dev, data->cb_data);
        }
}
#endif

static const struct uart_driver_api uart_tms570_driver_api = {
        .poll_in = uart_tms570_poll_in,
        .poll_out = uart_tms570_poll_out,

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
        .fifo_fill = uart_tms570_fifo_fill,
        .fifo_read = uart_tms570_fifo_read,
        .irq_tx_enable = uart_tms570_irq_tx_enable,
        .irq_tx_disable = uart_tms570_irq_tx_disable,
        .irq_tx_ready = uart_tms570_irq_tx_ready,
        .irq_tx_complete = uart_tms570_irq_tx_complete,
        .irq_rx_enable = uart_tms570_irq_rx_enable,
        .irq_rx_disable = uart_tms570_irq_rx_disable,
        .irq_rx_ready = uart_tms570_irq_rx_ready,
        .irq_err_enable = uart_tms570_irq_err_enable,
        .irq_err_disable = uart_tms570_irq_err_disable,
        .irq_is_pending = uart_tms570_irq_is_pending,
        .irq_update = uart_tms570_irq_update,
        .irq_callback_set = uart_tms570_irq_callback_set,
#endif
};

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#define UART_TMS570_IRQ_ENABLE_FN(n)                                                               \
        static void irq_config_func_##n(const struct device *dev)                                  \
        {                                                                                          \
                IRQ_CONNECT(DT_INST_IRQN(n), DT_INST_IRQ(n, priority), uart_tms570_isr,            \
                            DEVICE_DT_INST_GET(n), 0);                                             \
                irq_enable(DT_INST_IRQN(n));                                                       \
        }
#define UART_TMS570_IRQ_CFG(n) .irq_connect = irq_config_func_##n,
#else
#define UART_TMS570_IRQ_ENABLE_FN(n)
#define UART_TMS570_IRQ_CFG(n)
#endif /* CONFIG_UART_INTERRUPT_DRIVEN */

#define UART_TMS570_INIT(node)                                                                     \
        PINCTRL_DT_INST_DEFINE(node);                                                              \
        UART_TMS570_IRQ_ENABLE_FN(node);                                                           \
        static const struct uart_tms570_cfg uart_tms570_##node##_config = {                        \
                DEVICE_MMIO_ROM_INIT(DT_DRV_INST(node)),                                           \
                .baud = DT_INST_PROP(node, current_speed),                                         \
                .pincfg = PINCTRL_DT_INST_DEV_CONFIG_GET(node),                                    \
                .clk_ctrl = DEVICE_DT_GET(DT_INST_CLOCKS_CTLR(node)),                              \
                .clk_domain = DT_CLOCKS_CELL(DT_DRV_INST(node), clk_id),                           \
                UART_TMS570_IRQ_CFG(node)};                                                        \
        static struct uart_tms570_data uart_tms570_##node##_data;                                  \
                                                                                                   \
        DEVICE_DT_INST_DEFINE(node, &uart_tms570_init, NULL, &uart_tms570_##node##_data,           \
                              &uart_tms570_##node##_config, PRE_KERNEL_2,                          \
                              CONFIG_SERIAL_INIT_PRIORITY, &uart_tms570_driver_api)

DT_INST_FOREACH_STATUS_OKAY(UART_TMS570_INIT);
