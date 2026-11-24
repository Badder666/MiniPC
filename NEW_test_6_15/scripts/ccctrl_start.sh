#!/bin/bash
set -e

WORKSPACE="/home/hj/NEW_test_6_15_1/NEW_test_6_15"

# ── 1. 禁用 USB 自动挂起 ──────────────────────────────────────────
# 全局关闭内核 USB autosuspend (需要 root/sudo 权限)
if [ -w /sys/module/usbcore/parameters/autosuspend ]; then
    echo -1 > /sys/module/usbcore/parameters/autosuspend 2>/dev/null || true
fi
# 关闭各根 hub 的 autosuspend
for ctrl in /sys/bus/usb/devices/usb[0-9]*/power/control; do
    [ -w "$ctrl" ] && echo on > "$ctrl" 2>/dev/null || true
done
# 关闭所有 USB 设备的 autosuspend (需要 dialout 权限)
for dev in /sys/bus/usb/devices/*/power/control; do
    if [ -w "$dev" ]; then
        echo on > "$dev" 2>/dev/null || true
    fi
done

# ── 2. 等待串口设备就绪 ──────────────────────────────────────────
# 设备路径: 优先使用 udev 规则创建的固定符号链接
SERIAL_TC="/dev/serial_tc"   # CH341 ESP32 遥控接收器
SERIAL_DM="/dev/serial_dm"   # STM32 机械臂桥接
MAX_WAIT=15
ELAPSED=0

log() { echo "[ccctrl_start] $(date +%H:%M:%S) $*"; }

for dev in "$SERIAL_TC" "$SERIAL_DM"; do
    log "等待 $dev 出现..."
    while [ ! -e "$dev" ] && [ $ELAPSED -lt $MAX_WAIT ]; do
        sleep 1
        ELAPSED=$((ELAPSED + 1))
    done
    if [ -e "$dev" ]; then
        log "✓ $dev 已就绪"
    else
        log "✗ $dev 超时未出现，尝试回退设备..."
        # 回退: 使用 /dev/ttyUSB* 或 /dev/ttyACM*
        if [ "$dev" = "$SERIAL_TC" ] && [ -e /dev/ttyUSB0 ]; then
            log "  回退使用 /dev/ttyUSB0"
        elif [ "$dev" = "$SERIAL_DM" ] && [ -e /dev/ttyACM0 ]; then
            log "  回退使用 /dev/ttyACM0"
        fi
    fi
done

# ── 3. 确保设备稳定 (固件启动完成) ──────────────────────────────
# ESP32 和 STM32 上电后固件需要时间初始化，
# 开机自启动时设备可能刚被 USB 枚举，固件仍在启动中
sleep 3

# ── 4. 预检: 确认串口可以正常通信 ──────────────────────────────
for dev in "$SERIAL_TC" "$SERIAL_DM"; do
    if [ -e "$dev" ]; then
        if stty -F "$dev" 9600 2>/dev/null; then
            log "✓ $dev 串口通信正常"
        else
            log "✗ $dev 串口通信测试失败，继续尝试..."
        fi
    fi
done

# ── 5. 启动 ROS2 ──────────────────────────────────────────────────
log "启动 ROS2 CCtrl 节点..."
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"

exec ros2 launch arm_serial ccctrl_all.launch.py
