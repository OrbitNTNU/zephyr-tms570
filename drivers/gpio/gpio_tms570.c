
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#define DT_DRV_COMPAT ti_tms570_gpio

#define GCR0_OFFSET       (0x00)
#define GCR0_RESET_OFFSET (0)

#define INTDET_OFFSET    (0x08)
#define INTPOL_OFFSET    (0x0c)
#define INTENASET_OFFSET (0x10)
#define INTENACLR_OFFSET (0x14)
#define INTLVLSET_OFFSET (0x18)
#define INTLVLCLR_OFFSET (0x1c)
#define INTFLG_OFFSET    (0x20)
#define INTOFF1_OFFSET   (0x24)
#define INTOFF2_OFFSET   (0x28)

#define PORT_A_OFFSET      (0x34)
#define PORT_AB_SIZE       (0x20)
#define PORT_AB_PINS       (8)
#define PORT_AB_COUNT      (2)
#define PORT_DIR_OFFSET    (0x00)
#define PORT_DIN_OFFSET    (0x04)
#define PORT_DOUT_OFFSET   (0x08)
#define PORT_DSET_OFFSET   (0x0c)
#define PORT_DCLR_OFFSET   (0x10)
#define PORT_PDR_OFFSET    (0x14)
#define PORT_PULDIS_OFFSET (0x18)
#define PORT_PSL_OFFSET    (0x1c)

struct gpio_tms570_port_cfg {
        struct gpio_driver_config gpio_cfg;
        uintptr_t reg_base;
        const struct device *parent;
        bool interrupt_capable;
};

struct gpio_tms570_port_data {
        struct gpio_driver_data gpio_data;
        sys_slist_t cb;
};

struct gpio_tms570_ctrl_cfg {
        uintptr_t reg_base;
        void (*irq_connect)(void);

        const struct device **intc_ports;
        size_t num_intc_ports;
};

static int gpio_tms570_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;

        if (flags & GPIO_INPUT) {
                sys_clear_bit(cfg->reg_base + PORT_DIR_OFFSET, pin);

                if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) {
                        if (flags & GPIO_PULL_UP) {
                                sys_set_bit(cfg->reg_base + PORT_PSL_OFFSET, pin);
                        } else if (flags & GPIO_PULL_DOWN) {
                                sys_clear_bit(cfg->reg_base + PORT_PSL_OFFSET, pin);
                        }

                        sys_clear_bit(cfg->reg_base + PORT_PULDIS_OFFSET, pin);
                } else {
                        sys_set_bit(cfg->reg_base + PORT_PULDIS_OFFSET, pin);
                }
        } else if (flags & GPIO_OUTPUT) {
                sys_set_bit(cfg->reg_base + PORT_DIR_OFFSET, pin);

                if (flags & GPIO_SINGLE_ENDED) {
                        if (flags & GPIO_LINE_OPEN_SOURCE) {
                                return -ENOTSUP;
                        }

                        sys_set_bit(cfg->reg_base + PORT_PDR_OFFSET, pin);
                } else {
                        sys_clear_bit(cfg->reg_base + PORT_PDR_OFFSET, pin);
                }

                if (flags & GPIO_OUTPUT_INIT_LOW) {
                        sys_set_bit(cfg->reg_base + PORT_DCLR_OFFSET, pin);
                } else {
                        sys_set_bit(cfg->reg_base + PORT_DSET_OFFSET, pin);
                }
        }

        return 0;
}

static int gpio_tms570_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;

        *value = sys_read32(cfg->reg_base + PORT_DIN_OFFSET);
        return 0;
}

static int gpio_tms570_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
                                           gpio_port_value_t pins)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;
        uint32_t val;

        val = sys_read32(cfg->reg_base + PORT_DOUT_OFFSET);
        sys_write32((val & ~mask) | (pins & mask), cfg->reg_base + PORT_DOUT_OFFSET);
        return 0;
}

static int gpio_tms570_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;

        sys_write32(pins, cfg->reg_base + PORT_DSET_OFFSET);
        return 0;
}

static int gpio_tms570_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;

        sys_write32(pins, cfg->reg_base + PORT_DCLR_OFFSET);
        return 0;
}

static int gpio_tms570_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;
        uint32_t val;

        val = sys_read32(cfg->reg_base + PORT_DOUT_OFFSET);
        sys_write32(val ^ pins, cfg->reg_base + PORT_DOUT_OFFSET);

        return 0;
}

static unsigned int port_to_bit(const struct device *dev, gpio_pin_t pin)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;
        const struct gpio_tms570_ctrl_cfg *ctrl_cfg = cfg->parent->config;

        uintptr_t first = ctrl_cfg->reg_base + PORT_A_OFFSET;

        return ((cfg->reg_base - first) / PORT_AB_SIZE) * PORT_AB_PINS + pin;
}

static int gpio_tms570_manage_callback(const struct device *dev, struct gpio_callback *cb, bool set)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;
        struct gpio_tms570_port_data *data = dev->data;

        if (!cfg->interrupt_capable) {
                return -ENOTSUP;
        }

        return gpio_manage_callback(&data->cb, cb, set);
}

static int gpio_tms570_pin_interrupt_configure(const struct device *dev, gpio_pin_t pin,
                                               enum gpio_int_mode mode, enum gpio_int_trig trig)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;
        const struct gpio_tms570_ctrl_cfg *ctrl_cfg = cfg->parent->config;
        uintptr_t reg_base = ctrl_cfg->reg_base;
        unsigned int bit;
        int status;

        if (!cfg->interrupt_capable) {
                return -ENOTSUP;
        }

        bit = port_to_bit(dev, pin);
        status = 0;

        sys_set_bit(reg_base + INTENACLR_OFFSET, bit);

        if (mode == GPIO_INT_MODE_LEVEL || (trig & GPIO_INT_TRIG_WAKE) != 0) {
                return -ENOTSUP;
        }

        if (mode == GPIO_INT_MODE_DISABLED) {
                return 0;
        }

        if (trig == GPIO_INT_TRIG_BOTH) {
                sys_set_bit(reg_base + INTDET_OFFSET, bit);
        } else {
                sys_clear_bit(reg_base + INTDET_OFFSET, bit);

                if (trig & GPIO_INT_LOW_0) {
                        sys_clear_bit(reg_base + INTPOL_OFFSET, bit);
                } else if (trig & GPIO_INT_HIGH_1) {
                        sys_set_bit(reg_base + INTPOL_OFFSET, bit);
                } else {
                        return -ENOTSUP;
                }
        }

        sys_set_bit(reg_base + INTENASET_OFFSET, bit);
        return status;
}

static DEVICE_API(gpio, gpio_tms570_port_api) = {
        .pin_configure = gpio_tms570_pin_configure,
        .port_get_raw = gpio_tms570_port_get_raw,
        .port_set_masked_raw = gpio_tms570_port_set_masked_raw,
        .port_set_bits_raw = gpio_tms570_port_set_bits_raw,
        .port_clear_bits_raw = gpio_tms570_port_clear_bits_raw,
        .port_toggle_bits = gpio_tms570_port_toggle_bits,
        .manage_callback = gpio_tms570_manage_callback,
        .pin_interrupt_configure = gpio_tms570_pin_interrupt_configure,
};

static int gpio_tms570_port_init(const struct device *dev)
{
        ARG_UNUSED(dev);
        return 0;
}

static void gpio_tms570_ctrl_isr(const struct device *dev)
{
        const struct gpio_tms570_ctrl_cfg *ctrl_cfg = dev->config;
        const struct device *port;
        struct gpio_tms570_port_data *port_data;
        uint32_t reg;
        gpio_pin_t pin;

        /* We currently only use low-priority interrupts. */
        reg = sys_read32(ctrl_cfg->reg_base + INTOFF2_OFFSET);
        if (reg == 0 || reg > PORT_AB_COUNT * PORT_AB_PINS) {
                return;
        }

        pin = (reg - 1) % PORT_AB_PINS;
        port = ctrl_cfg->intc_ports[(reg - 1) / PORT_AB_PINS];
        port_data = port->data;

        gpio_fire_callbacks(&port_data->cb, port, BIT(pin));
}

static int gpio_tms570_ctrl_init(const struct device *dev)
{
        const struct gpio_tms570_ctrl_cfg *cfg = dev->config;

        cfg->irq_connect();

        sys_set_bit(cfg->reg_base + GCR0_OFFSET, GCR0_RESET_OFFSET);
        return 0;
}

#define GPIO_TMS570_PORT_INIT(node)                                                                \
        static const struct gpio_tms570_port_cfg gpio_tms570_port_##node##_cfg = {                 \
                .gpio_cfg.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(node),                   \
                .reg_base = DT_REG_ADDR(node),                                                     \
                .parent = DEVICE_DT_GET(DT_PARENT(node)),                                          \
                .interrupt_capable = DT_PROP(node, interrupt_capable),                             \
        };                                                                                         \
        static struct gpio_tms570_port_data gpio_tms570_port_##node##_data;                        \
        DEVICE_DT_DEFINE(node, gpio_tms570_port_init, NULL, &gpio_tms570_port_##node##_data,       \
                         &gpio_tms570_port_##node##_cfg, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,  \
                         &gpio_tms570_port_api)

#define GPIO_TMS570_CONTROLLER_IRQ_DEFINE(inst)                                                    \
        static void gpio_tms570_ctrl_##inst##_irq_connect(void)                                    \
        {                                                                                          \
                IRQ_CONNECT(DT_INST_IRQN(inst), 0, gpio_tms570_ctrl_isr, DEVICE_DT_INST_GET(inst), \
                            0);                                                                    \
                irq_enable(DT_INST_IRQN(inst));                                                    \
        }

#define GPIO_TMS570_CONTROLLER_IRQ_INIT(inst) .irq_connect = gpio_tms570_ctrl_##inst##_irq_connect,

/* Get the pointer to an interrupt-capable port. If the given node does not refer to an
 * interrupt-capable port,
 * () is returned. If the node does not have status okay, NULL is returned to preserve the order
 * specified in devicetree.
 *
 * This assumes that the GPIO ports in DTS are defined with ascending memory address (A, B, C and so
 * on). */
#define GPIO_TMS570_CONTROLLER_INTC_PORT_ITEM(node)                                                \
        COND_CODE_1(                                                                               \
                DT_PROP(node, interrupt_capable),                                                  \
                (COND_CODE_1(DT_NODE_HAS_STATUS(node, okay), (DEVICE_DT_GET(node), ), (NULL, ))),  \
                ())

#define GPIO_TMS570_CONTROLLER_INIT(inst)                                                          \
        GPIO_TMS570_CONTROLLER_IRQ_DEFINE(inst);                                                   \
        static const struct device *gpio_tms570_ctrl_##inst##_intc_ports[] = {                     \
                DT_INST_FOREACH_CHILD(inst, GPIO_TMS570_CONTROLLER_INTC_PORT_ITEM)};               \
        static const struct gpio_tms570_ctrl_cfg gpio_tms570_ctrl_##inst##_cfg = {                 \
                .reg_base = DT_INST_REG_ADDR(inst),                                                \
                .intc_ports = gpio_tms570_ctrl_##inst##_intc_ports,                                \
                .num_intc_ports = ARRAY_SIZE(gpio_tms570_ctrl_##inst##_intc_ports),                \
                GPIO_TMS570_CONTROLLER_IRQ_INIT(inst)};                                            \
        DEVICE_DT_INST_DEFINE(inst, gpio_tms570_ctrl_init, NULL, NULL,                             \
                              &gpio_tms570_ctrl_##inst##_cfg, PRE_KERNEL_1,                        \
                              CONFIG_GPIO_TMS570_CTRL_INIT_PRIORITY, NULL);                        \
        DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, GPIO_TMS570_PORT_INIT)

DT_INST_FOREACH_STATUS_OKAY(GPIO_TMS570_CONTROLLER_INIT);
