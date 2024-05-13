
#ifndef ZEPHYR_SOC_ARM_TMS570_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_TMS570_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

struct pinctrl_soc_pin {
        uint32_t offset;
        uint32_t value;
};

typedef struct pinctrl_soc_pin pinctrl_soc_pin_t;

#ifdef __cplusplus
extern "C" {
#endif

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
        (DT_PROP_BY_IDX(node_id, prop, idx)

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
        {                                                                                          \
                DT_FOREACH_CHILD_VARGS(DT_PHANDLE(node_id, prop), DT_FOREACH_PROP_ELEM, pinmux,    \
                                       Z_PINCTRL_STATE_PIN_INIT)                                   \
        }

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_TMS570_PINCTRL_SOC_H_ */
