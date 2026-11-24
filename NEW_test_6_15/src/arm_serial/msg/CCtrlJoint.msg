# CCtrlJoint — 关节角度 + CCtrl 面板数据，用于 ccctrl_receiver → bridge 传输
# 6 关节位置 (rad)
float64[6] position
# 主机按键 (bit0-3: K1-K4)
uint8 main_buttons
# Handle 按键 (bit0: S1, bit1: S2, bit2: S3, bit3: S4)
uint8 handle_buttons
# 摇杆 X 行程 (0-4095, 中心 2048)
uint16 joystick_x
# 摇杆 Y 行程 (0-4095, 中心 2048)
uint16 joystick_y
