# CCtrl Payload 业务数据 — 28 字节 payload 的全部字段
# 对应 CCtrl_串口协议说明.md 第 3 节

# 系统诊断位 (bit0-1: system_status, bit2: timeout_error, bit3-6: warning_flags)
uint8 diagnostic_flags

# 节点有效位 (bit0-5: A1-A6 Encoder, bit6: Handle)
uint8 node_valid_flags

# 主机按键 (bit0-3: K1-K4)
uint8 main_buttons

# Handle 按键 (bit0: S1, bit1: S2, bit2: S3, bit3: S4)
uint8 handle_buttons

# A1-A6 Encoder 状态 (每路: bit0 I2C通讯, bit1 检测磁铁, bit2 磁场过弱, bit3 磁场过强, bit4 数据过期)
uint8[6] encoder_status

# A1-A6 校准后有符号位置计数, 范围 -2048~2047, 一圈 = 4096 计数
int16[6] encoder_value

# 摇杆 X 行程 (0-4095, 中心 2048)
uint16 joystick_x

# 摇杆 Y 行程 (0-4095, 中心 2048, 方向已反转)
uint16 joystick_y

# 扳机行程 (0-4095, 0=松开, 4095=完全按下)
uint16 trigger
