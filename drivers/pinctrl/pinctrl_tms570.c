
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/pinctrl.h>

#define DT_DRV_COMPAT tms570_pinctrl
#define DRV_REG_ADDR DT_INST_REG_ADDR(0)

#define KICK0_OFFSET (0x38)
#define KICK1_OFFSET (0x3c)

#define KICK0_VALUE (0x83e70b13) /* Magic value from datasheet */
#define KICK1_VALUE (0x95a4f1e0) /* Magic value from datasheet */

#define PINMMR_OFFSET (0xb10) /* Offset of PINMMR registers */
#define PINMMR_SIZE   (48)    /* Number of PINMMR registers */

#define PIN_REG_OFFSET (10)
#define PIN_BIT(pin)   (1 << (pin & ((1 << PIN_REG_OFFSET) - 1)))
#define PIN_REG(pin)   (pin >> PIN_REG_OFFSET)

/**
 * @brief Helper function to write values @p kick0, @p kick1 to respective
 * KICKn registers. This is necessary to "unlock" access to the IOMM module
 *
 * @param kick0
 * @param kick1
 * @param reg
 */
static void kick_write(uint32_t kick0, uint32_t kick1, uintptr_t reg)
{
        sys_write32(kick0, reg + KICK0_OFFSET);
        sys_write32(kick1, reg + KICK1_OFFSET);
}

/**
 * @brief Unlock IOMM module, enabling writes to PINMMR0-PINMMR47
 */
static void mmr_unlock(void)
{
        kick_write(KICK0_VALUE, KICK1_VALUE, DRV_REG_ADDR);
}

/**
 * @brief Lock MMIO module
 */
static void mmr_lock(void)
{
        kick_write(0, 0, DRV_REG_ADDR);
}

/**
 * @brief Configure pin @pin
 *
 * @param pin
 * @param reg
 */
static int configure_pin(pinctrl_soc_pin_t pin)
{
        uintptr_t addr;
        uint16_t offset = PIN_REG(pin);

        if (offset >= PINMMR_SIZE) {
                return -EINVAL;
        }

        addr = DRV_REG_ADDR + PINMMR_OFFSET + (offset * sizeof(uint32_t));

        sys_write32(sys_read32(addr) | PIN_BIT(pin), addr);

        return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
        ARG_UNUSED(reg);

        mmr_unlock();

        for (uint8_t i = 0; i < pin_cnt; i++) {
                int status = configure_pin(pins[i]);

                if (status != 0) {
                        mmr_lock();
                        return status;
                }
        }

        mmr_lock();

        return 0;
}
