#!/bin/sh
# =============================================================================
# selfcheck_eth.sh —— V4 自研板 千兆以太网(RTL8211F-CG)自检   (2026-08-31)
#
# 用法: sh selfcheck_eth.sh [对端IP]
#   给了对端IP → 额外做连通性/丢包/吞吐测试(对端最好开 iperf3 -s)
#   不给 → 只做本地链路层自检(不需要网络环境)
#
# 退出码 = 失败项数(0 = 全过), 方便串进产线脚本
#
# 【自检思路】按"从芯片到线缆"的顺序, 每层独立判定, 坏在哪层一眼看出:
#   ①MDIO 能否读到 PHY  ②接口是否存在  ③载波(网线插了没)
#   ④★协商速率必须 1000Mb/s 全双工★(千兆自检的核心)  ⑤MAC 合法性
#   ⑥跑流量后错误计数必须为 0(反映 PCB 布线/阻抗/EMI 质量)
# =============================================================================
IF=${IF:-eth0}
PEER=$1
FAIL=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $*"; }
info() { echo "  [INFO] $*"; }

echo "======== 千兆以太网自检 (接口 $IF) ========"

# ---------- ① PHY 是否被驱动认到(MDIO 通不通) ----------
echo "[1/7] PHY 探测(MDIO)"
PHYLOG=$(dmesg 2>/dev/null | grep -iE "rtl8211|realtek|stmmac|dwmac|rk_gmac|eth0|phy" | tail -6)
[ -n "$PHYLOG" ] && echo "$PHYLOG" | sed 's/^/         /'
if dmesg 2>/dev/null | grep -qiE "rtl8211|realtek"; then
	pass "识别到 Realtek PHY(RTL8211F 系)"
elif [ -d /sys/class/net/$IF/phydev ]; then
	ID=$(cat /sys/class/net/$IF/phydev/phy_id 2>/dev/null)
	pass "PHY 已绑定, phy_id=$ID   (0x001cc916 = RTL8211F)"
else
	fail "没找到 PHY —— MDIO 不通/PHY 无供电/复位没释放/地址不对"
	info "查: ENET0_MDC(U1.F16) ENET0_MDIO(U1.D13) 焊接与 2.2k 上拉; ENET0_nRST 是否已拉高"
	info "查: 25MHz 晶振 X6 是否起振(PHY 没时钟则 MDIO 全 0xFFFF)"
fi

# ---------- ② 接口存在并能 up ----------
echo "[2/7] 网络接口"
if [ ! -d /sys/class/net/$IF ]; then
	fail "/sys/class/net/$IF 不存在 —— 驱动没加载或 dts 里 gmac 没 enable"
	echo "自检结束: 失败 $FAIL 项"; exit $FAIL
fi
pass "$IF 存在"
ip link set $IF up 2>/dev/null || ifconfig $IF up 2>/dev/null
sleep 3                                  # 等自协商(千兆协商约 2~3s)
OPER=$(cat /sys/class/net/$IF/operstate 2>/dev/null)
[ "$OPER" = "up" ] && pass "operstate=up" || warn "operstate=$OPER"

# ---------- ③ 载波: 网线插了没 ----------
echo "[3/7] 链路载波"
CAR=$(cat /sys/class/net/$IF/carrier 2>/dev/null)
if [ "$CAR" = "1" ]; then
	pass "carrier=1 (网线已连接, 对端已就绪)"
else
	fail "carrier=0 —— 网线没插/对端没通电/变压器或 RJ45 焊接问题"
	info "查: RJ45(RJ1) 与 D9/D10 共模滤波、TRX0~3 四对差分是否都通"
	info "★注意 1000Base-T 必须四对全通; 只通两对时会退化成 100M 或不通★"
fi

# ---------- ④ 协商速率(千兆的核心判定) ----------
echo "[4/7] 协商速率/双工  ★千兆核心★"
SPD=$(cat /sys/class/net/$IF/speed 2>/dev/null)
DUP=$(cat /sys/class/net/$IF/duplex 2>/dev/null)
if [ "$SPD" = "1000" ] && [ "$DUP" = "full" ]; then
	pass "1000Mb/s full —— 千兆链路成立 ✓"
elif [ -n "$SPD" ] && [ "$SPD" != "-1" ]; then
	fail "只协商到 ${SPD}Mb/s $DUP (期望 1000Mb/s full)"
	info "退化到 100M 的典型原因: 四对差分里有一对不通(断线/虚焊/串接件缺失)"
	info "  → 量 RJ45 到 U9 的 TRX0±/1±/2±/3± 八根线, 以及 RN1 端接排阻"
	info "退化原因二: RXC/TXC 时钟相位或 RGMII 延时不对 → 查 dts 的 tx/rx_delay"
else
	warn "读不到速率(没载波时无意义, 先解决第3项)"
fi
command -v ethtool >/dev/null 2>&1 && ethtool $IF 2>/dev/null | \
	grep -iE "speed|duplex|link detected|advertised link modes" | sed 's/^/         /'

# ---------- ⑤ MAC 地址合法性 ----------
echo "[5/7] MAC 地址"
MAC=$(cat /sys/class/net/$IF/address 2>/dev/null)
case "$MAC" in
	00:00:00:00:00:00|ff:ff:ff:ff:ff:ff) fail "MAC=$MAC 非法(efuse/vendor 分区没烧 MAC)" ;;
	"")  fail "读不到 MAC" ;;
	*)   pass "MAC=$MAC" ;;
esac

# ---------- ⑥ 连通性(给了对端IP才做) ----------
echo "[6/7] 连通性"
IPADDR=$(ip addr show $IF 2>/dev/null | grep -o 'inet [0-9.]*' | head -1 | cut -d' ' -f2)
[ -n "$IPADDR" ] && info "本机 IP: $IPADDR" || warn "$IF 还没有 IP(可 udhcpc -i $IF 或手工 ip addr add)"
if [ -n "$PEER" ]; then
	OUT=$(ping -c 20 -i 0.2 -W 1 $PEER 2>&1 | tail -3)
	echo "$OUT" | sed 's/^/         /'
	if echo "$OUT" | grep -q " 0% packet loss"; then
		pass "ping $PEER 20 包零丢包"
	else
		fail "ping $PEER 有丢包 —— 信号完整性/协商/对端问题"
	fi
else
	info "未给对端IP, 跳过(用法: sh $0 192.168.1.100)"
fi

# ---------- ⑦ 吞吐 + 错误计数(最能反映硬件质量) ----------
echo "[7/7] 吞吐与错误计数"
S=/sys/class/net/$IF/statistics
RXE0=$(cat $S/rx_errors 2>/dev/null); TXE0=$(cat $S/tx_errors 2>/dev/null)
RXD0=$(cat $S/rx_dropped 2>/dev/null); RXC0=$(cat $S/rx_crc_errors 2>/dev/null)
if [ -n "$PEER" ] && command -v iperf3 >/dev/null 2>&1; then
	info "iperf3 → $PEER (对端需先跑 iperf3 -s)"
	iperf3 -c $PEER -t 10 -f m 2>&1 | tail -6 | sed 's/^/         /'
	info "千兆参考: TCP 单流 ≥900Mbps 为优, <500Mbps 需查 CPU 占用/中断绑核/RGMII 延时"
elif [ -n "$PEER" ]; then
	info "无 iperf3, 用大包 ping 粗测(只验稳定性不测带宽)"
	ping -c 200 -i 0.01 -s 1400 -W 1 $PEER 2>&1 | tail -2 | sed 's/^/         /'
else
	info "无对端, 跳过吞吐"
fi
RXE=$(cat $S/rx_errors 2>/dev/null); TXE=$(cat $S/tx_errors 2>/dev/null)
RXD=$(cat $S/rx_dropped 2>/dev/null); RXC=$(cat $S/rx_crc_errors 2>/dev/null)
echo "         rx_errors $RXE0→$RXE  tx_errors $TXE0→$TXE  rx_dropped $RXD0→$RXD  rx_crc $RXC0→$RXC"
if [ "$RXE" = "0" ] && [ "$TXE" = "0" ] && [ "${RXC:-0}" = "0" ]; then
	pass "错误计数全 0 —— 物理层干净"
else
	fail "有错误计数 —— 差分对阻抗/长度匹配/EMI/端接 有问题(布线层面的锅)"
fi

echo "======== 以太网自检结束: 失败 $FAIL 项 ========"
exit $FAIL
