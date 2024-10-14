
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#define PCR_REG            (0xffffe000)
#define PSPWRDWNCLR_OFFSET (0xa0)
#define PSPWRDWNCLR_COUNT  (4)

#define SYS_REG    (0xffffff00)
#define ECR_OFFSET (0xe0)
#define RESET_BIT  (1 << 15)

extern void _errata_CORTEXR4_66_(void);
extern void _errata_CORTEXR4_57_(void);

static int pcr_init(void)
{
        for (int i = 0; i < PSPWRDWNCLR_COUNT; i++) {
                sys_write32((1ULL << 32) - 1, PCR_REG + PSPWRDWNCLR_OFFSET + i * sizeof(uint32_t));
        }
        return 0;
}

void sys_arch_reboot(int type)
{
        ARG_UNUSED(type);

        sys_set_bits(SYS_REG + ECR_OFFSET, RESET_BIT);
}

void z_arm_platform_init(void)
{
        _errata_CORTEXR4_66_();
        _errata_CORTEXR4_57_();

        pcr_init();
}
