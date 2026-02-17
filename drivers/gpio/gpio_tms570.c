
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>

#define DT_DRV_COMPAT ti_tms570_gpio

#define GCR0_OFFSET       (0x00)
#define GCR0_RESET_OFFSET (0)

#define DIR_OFFSET    (0x00)
#define DIN_OFFSET    (0x04)
#define DOUT_OFFSET   (0x08)
#define DSET_OFFSET   (0x0c)
#define DCLR_OFFSET   (0x10)
#define PDR_OFFSET    (0x14)
#define PULDIS_OFFSET (0x18)
#define PSL_OFFSET    (0x1c)

struct gpio_tms570_port_cfg {
        struct gpio_driver_config gpio_cfg;
        uintptr_t reg_base;
};

struct gpio_tms570_port_data {
        struct gpio_driver_data gpio_data;
};

struct gpio_tms570_ctrl_cfg {
        uintptr_t reg_base;
};

static int gpio_tms570_pin_configure(const struct device *dev, gpio_pin_t pin, gpio_flags_t flags)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;

        if (flags & GPIO_INPUT) {
                sys_clear_bit(cfg->reg_base + DIR_OFFSET, pin);

                if (flags & (GPIO_PULL_UP | GPIO_PULL_DOWN)) {
                        if (flags & GPIO_PULL_UP) {
                                sys_set_bit(cfg->reg_base + PSL_OFFSET, pin);
                        } else if (flags & GPIO_PULL_DOWN) {
                                sys_clear_bit(cfg->reg_base + PSL_OFFSET, pin);
                        }

                        sys_clear_bit(cfg->reg_base + PULDIS_OFFSET, pin);
                } else {
                        sys_set_bit(cfg->reg_base + PULDIS_OFFSET, pin);
                }
        } else if (flags & GPIO_OUTPUT) {
                sys_set_bit(cfg->reg_base + DIR_OFFSET, pin);

                if (flags & GPIO_SINGLE_ENDED) {
                        if (flags & GPIO_OPEN_SOURCE) {
                                return -ENOTSUP;
                        }

                        sys_set_bit(cfg->reg_base + PDR_OFFSET, pin);
                } else {
                        sys_clear_bit(cfg->reg_base + PDR_OFFSET, pin);
                }
        }

        if (flags & GPIO_OUTPUT_INIT_LOGICAL) {
                if (flags & GPIO_OUTPUT_INIT_LOW) {
                        sys_set_bit(cfg->reg_base + DCLR_OFFSET, pin);
                } else {
                        sys_set_bit(cfg->reg_base + DSET_OFFSET, pin);
                }
        }

        return 0;
}

static int gpio_tms570_port_get_raw(const struct device *dev, gpio_port_value_t *value)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;

        *value = sys_read32(cfg->reg_base + DIN_OFFSET);
        return 0;
}

static int gpio_tms570_port_set_masked_raw(const struct device *dev, gpio_port_pins_t mask,
                                           gpio_port_value_t pins)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;
        uint32_t val;

        val = sys_read32(cfg->reg_base + DOUT_OFFSET);
        sys_write32((val & ~mask) | (pins & mask), cfg->reg_base + DOUT_OFFSET);
        return 0;
}

static int gpio_tms570_port_set_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;

        sys_write32(pins, cfg->reg_base + DSET_OFFSET);
        return 0;
}

static int gpio_tms570_port_clear_bits_raw(const struct device *dev, gpio_port_pins_t pins)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;

        sys_write32(pins, cfg->reg_base + DCLR_OFFSET);
        return 0;
}

static int gpio_tms570_port_toggle_bits(const struct device *dev, gpio_port_pins_t pins)
{
        const struct gpio_tms570_port_cfg *cfg = dev->config;
        uint32_t val;

        val = sys_read32(cfg->reg_base + DOUT_OFFSET);
        sys_write32(val ^ pins, cfg->reg_base + DOUT_OFFSET);

        return 0;
}

static DEVICE_API(gpio, gpio_tms570_api) = {
        .pin_configure = gpio_tms570_pin_configure,
        .port_get_raw = gpio_tms570_port_get_raw,
        .port_set_masked_raw = gpio_tms570_port_set_masked_raw,
        .port_set_bits_raw = gpio_tms570_port_set_bits_raw,
        .port_clear_bits_raw = gpio_tms570_port_clear_bits_raw,
        .port_toggle_bits = gpio_tms570_port_toggle_bits,
};

static int gpio_tms570_port_init(const struct device *dev)
{
        ARG_UNUSED(dev);
        return 0;
}

static int gpio_tms570_ctrl_init(const struct device *dev)
{
        const struct gpio_tms570_ctrl_cfg *cfg = dev->config;

        sys_set_bit(cfg->reg_base + GCR0_OFFSET, GCR0_RESET_OFFSET);
        return 0;
}

#define GPIO_TMS570_PORT_INIT(node)                                                                \
        static const struct gpio_tms570_port_cfg gpio_tms570_port_##node##_cfg = {                 \
                .gpio_cfg.port_pin_mask = GPIO_PORT_PIN_MASK_FROM_DT_NODE(node),                   \
                .reg_base = DT_REG_ADDR(node),                                                     \
        };                                                                                         \
        static struct gpio_tms570_port_data gpio_tms570_port_##node##_data;                        \
        DEVICE_DT_DEFINE(node, gpio_tms570_port_init, NULL, &gpio_tms570_port_##node##_data,       \
                         &gpio_tms570_port_##node##_cfg, PRE_KERNEL_1, CONFIG_GPIO_INIT_PRIORITY,  \
                         NULL)

#define GPIO_TMS570_CONTROLLER_INIT(inst)                                                          \
        const struct gpio_tms570_ctrl_cfg gpio_tms570_ctrl_##inst##_cfg = {                        \
                .reg_base = DT_INST_REG_ADDR(inst),                                                \
        };                                                                                         \
        DEVICE_DT_INST_DEFINE(inst, gpio_tms570_ctrl_init, NULL, NULL,                             \
                              &gpio_tms570_ctrl_##inst##_cfg, PRE_KERNEL_1,                        \
                              CONFIG_GPIO_TMS570_CTRL_INIT_PRIORITY, NULL);                        \
        DT_INST_FOREACH_CHILD_STATUS_OKAY(inst, GPIO_TMS570_PORT_INIT)

DT_INST_FOREACH_STATUS_OKAY(GPIO_TMS570_CONTROLLER_INIT);
