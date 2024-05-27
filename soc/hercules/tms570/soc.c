
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#define PCR_REG            (0xffffe000)
#define PSPWRDWNCLR_OFFSET (0xa0)
#define PSPWRDWNCLR_COUNT  (4)

extern void _errata_CORTEXR4_66_(void);
extern void _errata_CORTEXR4_57_(void);

static int pcr_init(void)
{
        for (int i = 0; i < PSPWRDWNCLR_COUNT; i++) {
                sys_write32((1ULL << 32) - 1, PCR_REG + PSPWRDWNCLR_OFFSET + i * sizeof(uint32_t));
        }
        return 0;
}

void z_arm_platform_init(void)
{
        _errata_CORTEXR4_66_();
        _errata_CORTEXR4_57_();

        pcr_init();
}
