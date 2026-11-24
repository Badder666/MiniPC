# 摇杆 ADC 范围 0..4095，中位 2048。
uint16 joystick_x
uint16 joystick_y
int16 mouse_x
int16 mouse_y
int16 mouse_z
# 鼠标按键状态沿用旧协议的 0..3。
uint8 mouse_left
uint8 mouse_right
uint8 mouse_mid
# bit 0..17: W S A D Shift Ctrl Q E R F G Z X C V B 1 2。
uint32 keys
uint8 main_buttons
uint8 handle_buttons
