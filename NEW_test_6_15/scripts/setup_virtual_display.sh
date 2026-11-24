#!/bin/bash
# ============================================================================
# 虚拟显示器配置脚本
# 解决 Nomachine 无物理显示器时卡顿的问题
#
# 原理：
#   1. 切换 GDM 从 Wayland 到 X11（Nomachine 对 X11 兼容性更好）
#   2. 安装 xserver-xorg-video-dummy 虚拟显示驱动
#   3. 配置 Xorg 虚拟显示器（1920x1080@60Hz）
#   4. 创建自动设置脚本，登录后自动激活虚拟显示器
# ============================================================================

set -e

echo "============================================"
echo " Nomachine 无头虚拟显示器配置"
echo "============================================"

# ── 1. 切换 GDM 到 X11 ──────────────────────────────────────────
echo ""
echo "[1/4] 切换 GDM 从 Wayland 到 X11..."

if grep -q "^WaylandEnable=false" /etc/gdm3/custom.conf; then
    echo "  → 已经是 X11 模式，跳过"
else
    sudo sed -i 's/^#WaylandEnable=false/WaylandEnable=false/' /etc/gdm3/custom.conf
    echo "  → 已启用 WaylandEnable=false"
fi

# ── 2. 安装虚拟显示驱动 ──────────────────────────────────────────
echo ""
echo "[2/4] 安装 xserver-xorg-video-dummy..."

if dpkg -l xserver-xorg-video-dummy 2>/dev/null | grep -q '^ii'; then
    echo "  → 已安装，跳过"
else
    sudo apt update -qq
    sudo apt install -y xserver-xorg-video-dummy
    echo "  → 安装完成"
fi

# ── 3. 配置 Xorg 虚拟显示器 ──────────────────────────────────────
echo ""
echo "[3/4] 配置 Xorg 虚拟显示器..."

sudo tee /etc/X11/xorg.conf.d/99-virtual-headless.conf > /dev/null <<'XORGCONF'
# 虚拟显示器配置 - 无物理显示器时提供 GPU 加速显示
# 配合 Intel modesetting 驱动使用

Section "Device"
    Identifier  "IntelGPU"
    Driver      "modesetting"
    Option      "AccelMethod"    "glamor"
    Option      "DRI"            "3"
    Option      "SWCursor"       "on"
EndSection

Section "Monitor"
    Identifier   "HeadlessMonitor"
    HorizSync    28.0 - 83.0
    VertRefresh  48.0 - 75.0
    # 1920x1080 @ 60Hz
    Modeline "1920x1080_60"  173.00  1920 2048 2248 2576  1080 1083 1088 1120 -hsync +vsync
    Option       "PreferredMode" "1920x1080_60"
EndSection

Section "Screen"
    Identifier   "HeadlessScreen"
    Device       "IntelGPU"
    Monitor      "HeadlessMonitor"
    DefaultDepth 24
    SubSection "Display"
        Depth    24
        Modes    "1920x1080_60" "1680x1050" "1280x720"
    EndSubSection
EndSection

Section "ServerLayout"
    Identifier   "HeadlessLayout"
    Screen       "HeadlessScreen"
    Option       "AllowEmptyInitialConfiguration" "true"
EndSection
XORGCONF

echo "  → 已写入 /etc/X11/xorg.conf.d/99-virtual-headless.conf"

# ── 4. 创建用户级自动设置脚本 ─────────────────────────────────────
echo ""
echo "[4/4] 创建登录后自动配置脚本..."

mkdir -p ~/.config/autostart

cat > ~/.config/autostart/virtual-display-setup.desktop <<'DESKTOP'
[Desktop Entry]
Type=Application
Name=Virtual Display Setup
Comment=Setup virtual display for headless Nomachine
Exec=bash -c 'sleep 2 && xrandr --fb 1920x1080 && xrandr --output default --mode 1920x1080 2>/dev/null || xrandr --auto 2>/dev/null || true'
Terminal=false
X-GNOME-Autostart-enabled=true
NoDisplay=true
DESKTOP

# 同时创建 xprofile（在 X 会话启动时生效，比 autostart 更早）
cat > ~/.xprofile <<'XPROFILE'
#!/bin/bash
# 虚拟显示器自动配置
# 在 X11 会话启动时执行

# 等待 X 服务就绪
sleep 1

# 获取第一个可用的输出
OUTPUT=$(xrandr | grep -E "^\S.*connected" | head -1 | awk '{print $1}')

if [ -n "$OUTPUT" ]; then
    # 设置虚拟显示模式
    xrandr --fb 1920x1080
    # 确保有可用的显示模式
    xrandr --output "$OUTPUT" --auto 2>/dev/null || true
fi

# 如果没有任何 connected 输出，创建虚拟显示器
if [ -z "$OUTPUT" ]; then
    # 创建一个虚拟模式
    xrandr --fb 1920x1080
    # 尝试在 HDMI-A-1 上添加模式并强制开启
    xrandr --newmode "1920x1080_60" 173.00 1920 2048 2248 2576 1080 1083 1088 1120 -hsync +vsync 2>/dev/null || true
    xrandr --addmode HDMI-A-1 1920x1080_60 2>/dev/null || true
    xrandr --output HDMI-A-1 --mode 1920x1080_60 2>/dev/null || true
fi

# 确保至少有一个 1920x1080 的 framebuffer
xrandr --fb 1920x1080
XPROFILE

chmod +x ~/.xprofile
echo "  → 已创建 ~/.xprofile 和 ~/.config/autostart/virtual-display-setup.desktop"

# ── 完成 ─────────────────────────────────────────────────────────
echo ""
echo "============================================"
echo " 配置完成！"
echo "============================================"
echo ""
echo "请执行以下操作使配置生效："
echo "  sudo reboot"
echo ""
echo "重启后验证："
echo "  1. echo \$XDG_SESSION_TYPE    # 应该输出 x11"
echo "  2. xrandr                      # 应该能看到 display 输出"
echo "  3. 拔掉显示器，通过 Nomachine 连接，看是否还卡顿"
echo ""
echo "如果仍卡顿，可以尝试在内核参数中强制启用 HDMI 输出："
echo "  编辑 /etc/default/grub，在 GRUB_CMDLINE_LINUX_DEFAULT 中加入："
echo "  video=HDMI-A-1:1920x1080@60e"
echo "  然后执行 sudo update-grub && sudo reboot"
