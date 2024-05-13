
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/pinctrl.h>

#define KICK0_OFFSET (0x38)
#define KICK1_OFFSET (0x3c)

#define KICK0_VALUE (0x83e70b13) /* Magic value from datasheet */
#define KICK1_VALUE (0x95a4f1e0) /* Magic value from datasheet */

#define PINMMR_OFFSET (0xb10) /* Offset of PINMMR registers */
#define PINMMR_SIZE   (48)    /* Number of PINMMR registers */

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
 *
 * @param reg Address to IOMM RAM base
 */
static void mmr_unlock(uintptr_t reg)
{
        kick_write(KICK0_VALUE, KICK1_VALUE, reg);
}

/**
 * @brief Lock MMIO module
 *
 * @param reg Address to IOMM RAM base
 */
static void mmr_lock(uintptr_t reg)
{
        kick_write(0, 0, reg);
}

/**
 * @brief Configure pin @pin
 *
 * @param pin
 * @param reg
 */
static int configure_pin(pinctrl_soc_pin_t pin, uintptr_t reg)
{
        uintptr_t addr;

        if (pin.offset >= PINMMR_SIZE) {
                return -EINVAL;
        }

        addr = reg + PINMMR_OFFSET + pin.offset;

        sys_write32(sys_read32(addr) | pin.value, addr);

        return 0;
}

int pinctrl_configure_pins(const pinctrl_soc_pin_t *pins, uint8_t pin_cnt, uintptr_t reg)
{
        mmr_unlock(reg);

        for (uint8_t i = 0; i < pin_cnt; i++) {
                int status = configure_pin(pins[0], reg);

                if (status != 0) {
                        mmr_lock(reg);
                        return status;
                }
        }

        mmr_lock(reg);

        return 0;
}
