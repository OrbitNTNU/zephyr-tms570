
#ifndef ZEPHYR_SOC_ARM_TMS570_PINCTRL_SOC_H_
#define ZEPHYR_SOC_ARM_TMS570_PINCTRL_SOC_H_

#include <zephyr/devicetree.h>
#include <zephyr/types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef uint16_t pinctrl_soc_pin_t;

#ifdef __cplusplus
extern "C" {
#endif

#define TMS570_PIN_INIT(node_id) DT_PROP(node_id, pinmux),

#define Z_PINCTRL_STATE_PIN_INIT(node_id, prop, idx)                                               \
        TMS570_PIN_INIT(DT_PROP_BY_IDX(node_id, prop, idx))

#define Z_PINCTRL_STATE_PINS_INIT(node_id, prop)                                                   \
        {                                                                                          \
                DT_FOREACH_PROP_ELEM(node_id, prop, Z_PINCTRL_STATE_PIN_INIT)                      \
        }

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_SOC_ARM_TMS570_PINCTRL_SOC_H_ */
