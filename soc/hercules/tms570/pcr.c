
#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>

#define PCR_REG            (0xffffe000)
#define PSPWRDWNCLR_OFFSET (0xa0)
#define PSPWRDWNCLR_COUNT  (4)
#define INIT_PRIO          10

static int pcr_init(void)
{
        for (int i = 0; i < PSPWRDWNCLR_COUNT; i++) {
                sys_write32((1ULL << 32) - 1, PCR_REG + PSPWRDWNCLR_OFFSET + i * sizeof(uint32_t));
        }
        return 0;
}

SYS_INIT(pcr_init, PRE_KERNEL_1, INIT_PRIO);
