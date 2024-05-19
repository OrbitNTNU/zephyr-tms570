
add_subdirectory(timer)
add_subdirectory(interrupt_controller)
add_subdirectory(clock_control)

add_subdirectory_ifdef(CONFIG_PINCTRL pinctrl)
