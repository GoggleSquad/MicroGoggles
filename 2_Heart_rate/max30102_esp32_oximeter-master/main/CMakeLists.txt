# Component requirements
set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES )

# Source files (removed mqtt_wifi.c)
set(COMPONENT_SRCS "main.c" "i2c_api.c" "max30102_api.c" "algorithm.c")
set(COMPONENT_ADD_INCLUDEDIRS "")

register_component()
