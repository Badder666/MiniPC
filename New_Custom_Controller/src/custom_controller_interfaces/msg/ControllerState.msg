# ROS receive time; device_time_ms is the MCU position snapshot time.
builtin_interfaces/Time stamp
uint16 sequence
uint32 device_time_ms
# Actual motor positions 1-6 in radians, including retained values for invalid axes.
float32[6] positions
# Bit 0 maps to BUTTON_0; one means pressed after debouncing.
uint16 buttons
# Bits 0-5 map to motors 1-6; one means recent valid feedback.
uint16 valid_motor_mask
