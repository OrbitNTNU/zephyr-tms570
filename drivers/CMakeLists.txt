
add_subdirectory(timer)
add_subdirectory(interrupt_controller)

add_subdirectory_ifdef(CONFIG_PINCTRL pinctrl)
