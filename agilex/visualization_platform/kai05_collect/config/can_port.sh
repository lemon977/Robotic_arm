#!/bin/bash
#===============================================================================#
# gs_usb 修复版脚本 - 同步参考版原子化绑定逻辑
#===============================================================================#

set -e
[ "$EUID" -ne 0 ] && { echo "❌ 必须用 sudo 运行"; exit 1; }

# === 1. 禁用USB省电 ===
echo -1 | tee /sys/module/usbcore/parameters/autosuspend >/dev/null 2>&1 || true

# === 2. 重置驱动 ===
echo "🔄 重置 gs_usb 驱动..."
rmmod gs_usb 2>/dev/null || true
sleep 1.5
modprobe gs_usb
sleep 2.5

# === 3. 检测设备 ===
mapfile -t ifaces < <(ip -br link show type can 2>/dev/null | awk '{print $1}' | sort)
[ "${#ifaces[@]}" -lt 4 ] && {
    echo "❌ 仅检测到 ${#ifaces[@]} 个设备"
    exit 1
}
echo "✅ 检测到 ${#ifaces[@]} 个设备: ${ifaces[*]}"

# === 4. 统一映射 (同步参考版结构) ===
declare -A USB_PORTS
USB_PORTS["1-1:1.0"]="can_l_master:1000000"    # 格式: "目标名:比特率"
USB_PORTS["1-13:1.0"]="can_l_slave:1000000"
USB_PORTS["1-2:1.0"]="can_r_master:1000000"
USB_PORTS["1-12:1.0"]="can_r_slave:1000000"

# === 5. 原子化绑定 (核心同步点) ===
for iface in "${ifaces[@]}"; do
    bus_info=$(timeout 0.5 ethtool -i "$iface" 2>/dev/null | grep "bus-info" | awk '{print $2}')
    [ -z "$bus_info" ] && { echo "⚠️ 跳过 $iface (无bus-info)"; continue; }
    
    target_info="${USB_PORTS[$bus_info]}"
    [ -z "$target_info" ] && { echo "⚠️ 跳过 $iface (未知bus-info: $bus_info)"; continue; }
    
    IFS=':' read -r target_name target_bitrate <<< "$target_info"
    echo "⚙️ 处理 $iface (USB: $bus_info) -> 目标: $target_name, 比特率: ${target_bitrate}"

    # --- 智能状态检查 (同步参考版逻辑) ---
    current_state=$(ip link show "$iface" 2>/dev/null | grep -oP 'state \K\w+' || echo "DOWN")
    current_bitrate=$(ip -details link show "$iface" 2>/dev/null | grep -oP 'bitrate \K\d+' || echo "0")

    # 情况1: 已激活且比特率正确 → 仅需重命名
    if [ "$current_state" = "UP" ] && [ "$current_bitrate" = "$target_bitrate" ]; then
        echo "✅ $iface 已激活且比特率正确 (当前: ${current_bitrate})"
        
        if [ "$iface" != "$target_name" ]; then
            echo "✏️  仅需重命名: $iface → $target_name"
            ip link set "$iface" down
            sleep 0.2
            ip link set "$iface" name "$target_name" || {
                echo "❌ 重命名失败: $iface → $target_name (可能被占用)"
                exit 1
            }
            sleep 0.2
            ip link set "$target_name" up
            sleep 0.3
            echo "  ✓ 重命名完成: $iface → $target_name"
        else
            echo "  ✓ 接口名已匹配: $iface"
        fi
        continue
    fi

    # 情况2: 需要完整重置 (未激活/比特率错误)
    echo "🔧 需要重置配置 (当前状态: $current_state, 比特率: ${current_bitrate})"

    # 严格 DOWN → 配置 → UP 流程
    ip link set "$iface" down 2>/dev/null || true
    sleep 0.5

    # 关键修复: 仅设置必要参数 (移除 berr-reporting)
    if ! ip link set "$iface" type can bitrate "$target_bitrate" restart-ms 100 2>/dev/null; then
        echo "⚠️  restart-ms 不兼容，回退到基础配置"
        ip link set "$iface" type can bitrate "$target_bitrate"
    fi
    sleep 0.3

    ip link set "$iface" up
    sleep 0.8  # gs_usb 固件初始化等待

    # 验证激活
    if ! ip link show "$iface" | grep -q "state UP"; then
        echo "❌ 激活失败: $iface"
        dmesg | tail -10 | grep -i "gs_usb\|$iface"
        exit 1
    fi
    echo "✅ $iface 激活成功 (实际比特率: $(ip -details link show $iface | grep -oP 'bitrate \K\d+' || echo 'unknown'))"

    # --- 立即重命名 (原子化绑定) ---
    if [ "$iface" != "$target_name" ]; then
        echo "✏️  重命名: $iface → $target_name"
        ip link set "$iface" down
        sleep 0.2
        ip link set "$iface" name "$target_name" || {
            echo "❌ 重命名失败: $iface → $target_name"
            exit 1
        }
        sleep 0.2
        ip link set "$target_name" up
        sleep 0.3
        echo "  ✓ 绑定完成: $target_name (USB: $bus_info)"
    else
        echo "  ✓ 接口名已匹配: $iface"
    fi

    sleep 0.5
done

# === 6. 验证 (使用目标名称验证) ===
echo -e "\n📊 最终状态:"
for bus_info in "${!USB_PORTS[@]}"; do
    target_info="${USB_PORTS[$bus_info]}"
    IFS=':' read -r target_name target_bitrate <<< "$target_info"
    
    if ip link show "$target_name" >/dev/null 2>&1; then
        state=$(ip -br link show "$target_name" | awk '{print $2}')
        bitrate=$(ip -details link show "$target_name" 2>/dev/null | grep -oP 'bitrate \K\d+' || echo "0")
        echo "  $target_name: $state, bitrate=${bitrate} (目标: ${target_bitrate})"
    else
        echo "  ❌ $target_name 不存在 (USB: $bus_info)"
        exit 1
    fi
done

echo -e "\n✅ 配置完成! 测试命令: cansend can0 001#1122334455667788"
