
#include <stdint.h>

extern void _errata_CORTEXR4_66_(void);
extern void _errata_CORTEXR4_57_(void);

extern int pcr_init(void);

#define _write32(addr, val)                                                                        \
        do {                                                                                       \
                *(volatile uint32_t *)(addr) = val;                                                \
        } while (0)

#define _read32(addr) *(volatile uint32_t *)(addr)

void z_arm_platform_init(void)
{
        _errata_CORTEXR4_66_();
        _errata_CORTEXR4_57_();

#if 0
        _write32(0xffffff00 + 0x30, 0x42);
        while (((_read32(0xffffff00 + 0x30)) & 0x42) != 0x42) {
        }

        _write32(0xffffff00 + 0xec, 0x0301);
        _write32(0xffffff00 + 0x30, 0x2);
#endif

        pcr_init();
        _write32(0xffffff00 + 0xd0, 1 << 8);     /* PERIENA */
        _write32(0xffffea00 + 0x38, 0x83E70B13); /* Kick 0 */
        _write32(0xffffea00 + 0x3c, 0x95A4F1E0); /* Kick 1 */

        _write32(0xffffea00 + 0xb10 + 8 * 4, 0x1); /* Attempt to write 1 to PINMMR8 */
}
