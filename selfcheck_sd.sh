#!/bin/sh
# =============================================================================
# selfcheck_sd.sh —— V4 自研板 SD 卡(目标 SD3.0 / UHS-I SDR104)自检  (2026-08-31)
#
# 用法: sh selfcheck_sd.sh [块设备]
#   不给参数 → 自动找 SD 卡(排除承载 rootfs 的 eMMC, 防误写系统盘)
#   例: sh selfcheck_sd.sh /dev/mmcblk1
#
# ★安全★ 读写测试只在【卡上的挂载点或空闲分区】建临时文件, 绝不 dd 裸设备,
#         且会先确认目标不是 rootfs/eMMC。测完自动删除临时文件。
#
# 退出码 = 失败项数(0 = 全过)
#
# 【SD3.0 到底看什么】不是看"能不能读写", 而是三件事:
#   ①信号电压切到 1.8V 了吗   ②timing 是不是 UHS 档(SDR104/DDR50)
#   ③时钟跑到多少(SDR104 = 208MHz)  → 顺序读能否上 60MB/s+
#   只要 timing 还是 "sd high speed"、clock 50MHz, 就说明还在 SD2.0 档,
#   多半是【设备树没开 UHS 能力】, 不是卡或硬件的问题(脚本会明确提示)。
# =============================================================================
FAIL=0
pass() { echo "  [PASS] $*"; }
fail() { echo "  [FAIL] $*"; FAIL=$((FAIL+1)); }
warn() { echo "  [WARN] $*"; }
info() { echo "  [INFO] $*"; }

echo "======== SD 卡自检 (目标 SD3.0/UHS-I) ========"

# ---------- ① 找到 SD 卡, 并确保不是系统盘 ----------
echo "[1/7] 卡检测"
ROOTDEV=$(mount | awk '$3=="/"{print $1}' | sed 's/p\?[0-9]*$//')
DEV=$1
if [ -z "$DEV" ]; then
	for d in /dev/mmcblk1 /dev/mmcblk0 /dev/mmcblk2; do
		[ -b "$d" ] || continue
		case "$ROOTDEV" in *"$(basename $d)"*) continue;; esac   # 跳过承载 rootfs 的
		DEV=$d; break
	done
fi
if [ -z "$DEV" ] || [ ! -b "$DEV" ]; then
	fail "没找到 SD 卡块设备(卡没插? 卡座虚焊? SDMMC0_DET 没接地?)"
	dmesg 2>/dev/null | grep -iE "mmc|sdhci|dwmmc" | tail -8 | sed 's/^/         /'
	info "查: 卡座 U10 的 CLK/CMD/D0~D3 六根线; SDMMC0_PWREN(U1.B13)→ Q2 是否给了 VCC3V3_SD"
	echo "======== 自检结束: 失败 $FAIL 项 ========"; exit $FAIL
fi
case "$ROOTDEV" in *"$(basename $DEV)"*)
	fail "$DEV 是系统盘, 拒绝测试(防误写)"; exit $FAIL;; esac
pass "找到 SD 卡: $DEV (系统盘=$ROOTDEV, 已避开)"
MMCN=$(basename $DEV | sed 's/mmcblk//')

# ---------- ② 卡信息(CID/CSD/SCR) ----------
echo "[2/7] 卡身份信息"
SYSD=/sys/block/$(basename $DEV)/device
for k in name type cid csd scr ocr fwrev hwrev serial date; do
	[ -f "$SYSD/$k" ] && printf "         %-7s %s\n" "$k" "$(cat $SYSD/$k 2>/dev/null)"
done
SZ=$(cat /sys/block/$(basename $DEV)/size 2>/dev/null)
[ -n "$SZ" ] && info "容量 $((SZ/2/1024/1024)) GiB ($SZ 个 512B 扇区)"
[ -f "$SYSD/type" ] && [ "$(cat $SYSD/type)" = "SD" ] && pass "卡类型 = SD" || warn "卡类型不是 SD?"

# ---------- ③ ★总线参数: SD3.0 的判据在这★ ----------
echo "[3/7] 总线参数(SD3.0 判据)"
IOS=/sys/kernel/debug/mmc$MMCN/ios
if [ -f "$IOS" ]; then
	cat $IOS | sed 's/^/         /'
	CLK=$(awk '/^clock:/{print $2}' $IOS)
	TIM=$(grep -i '^timing spec' $IOS | cut -d: -f2- | sed 's/^ *//')
	SIG=$(grep -i '^signal voltage' $IOS | cut -d: -f2- | sed 's/^ *//')
	WID=$(awk '/^bus width:/{print $3}' $IOS)

	[ "${WID:-0}" -ge 2 ] 2>/dev/null && pass "总线宽度 4bit" || warn "总线宽度=$WID (期望 4bit)"

	case "$SIG" in
		*1.8*) pass "信号电压已切到 1.8V (UHS 前提成立)" ;;
		*)     warn "信号电压=$SIG —— 没切 1.8V 就上不了 UHS"
		       info "查: SDMMC0_VOL_CTRL(U1.A12)→Q1 电压切换电路, 以及 dts 的 vqmmc-supply" ;;
	esac
	case "$TIM" in
		*SDR104*|*sdr104*) pass "timing = SDR104 —— ★SD3.0 达标★" ;;
		*DDR50*|*ddr50*)   warn "timing = DDR50 (UHS-I 但非最高档)" ;;
		*)  fail "timing = $TIM —— 还在 SD2.0 档, 没进 UHS"
		    info "★最可能是设备树没开 UHS 能力★ 当前 sdmmc0 只有 cap-sd-highspeed"
		    info "  需在 dts 的 &sdmmc0 里加: sd-uhs-sdr50; sd-uhs-sdr104;"
		    info "  并把 max-frequency 提到 208000000, 确认 vqmmc-supply 能切 1.8V" ;;
	esac
	if [ -n "$CLK" ]; then
		info "实际时钟 $((CLK/1000000)) MHz  (SDR104=208M, SDR50=100M, HS=50M)"
		[ "$CLK" -ge 100000000 ] 2>/dev/null && pass "时钟 ≥100MHz" || warn "时钟偏低, 与上面的 timing 一致"
	fi
else
	warn "没有 $IOS (内核没开 CONFIG_MMC_DEBUG/debugfs 没挂)"
	info "挂 debugfs: mount -t debugfs none /sys/kernel/debug"
fi

# ---------- ④ 分区与挂载 ----------
echo "[4/7] 分区与挂载"
ls /dev/$(basename $DEV)p* 2>/dev/null | sed 's/^/         分区: /'
MP=$(mount | grep "^$DEV" | awk '{print $3}' | head -1)
TMPMP=""
if [ -z "$MP" ]; then
	PART=$(ls /dev/$(basename $DEV)p1 2>/dev/null || echo $DEV)
	TMPMP=/tmp/sdtest_$$
	mkdir -p $TMPMP
	if mount $PART $TMPMP 2>/dev/null; then
		MP=$TMPMP; pass "已临时挂载 $PART → $MP"
	else
		rmdir $TMPMP 2>/dev/null; TMPMP=""
		fail "挂不上(没分区表/文件系统? 先 fdisk + mkfs.vfat)"
	fi
else
	pass "已挂载于 $MP"
fi

# ---------- ⑤⑥ 读写速度 + 数据完整性 ----------
if [ -n "$MP" ] && [ -w "$MP" ]; then
	F=$MP/.sdselftest.bin
	SZMB=${SZMB:-64}
	echo "[5/7] 顺序写/读速度 (${SZMB}MB)"
	# 用 date +%s%N 计时(dash 没有 time 内建); 写源用 /dev/zero(urandom 会被 CPU 卡住测不准)
	T0=$(date +%s%N); dd if=/dev/zero of=$F bs=1M count=$SZMB conv=fsync 2>/dev/null; T1=$(date +%s%N)
	sync; echo 3 > /proc/sys/vm/drop_caches 2>/dev/null      # 清缓存, 否则读到的是内存
	T2=$(date +%s%N); dd if=$F of=/dev/null bs=1M 2>/dev/null; T3=$(date +%s%N)
	WS=$(awk -v s=$SZMB -v a=$T0 -v b=$T1 'BEGIN{t=(b-a)/1e9; printf "%.1f", (t>0)?s/t:0}')
	RS=$(awk -v s=$SZMB -v a=$T2 -v b=$T3 'BEGIN{t=(b-a)/1e9; printf "%.1f", (t>0)?s/t:0}')
	info "写 ${WS} MB/s   读 ${RS} MB/s"
	awk -v r="$RS" 'BEGIN{exit !(r>=60)}' && pass "顺序读 ≥60MB/s —— 符合 UHS-I SDR104 量级" || \
	awk -v r="$RS" 'BEGIN{exit !(r>=35)}' && warn "顺序读 ${RS}MB/s —— SD2.0 高速档量级(约 20~45MB/s), 对照第3项 timing" || \
	fail "顺序读只有 ${RS}MB/s —— 明显偏低, 查 timing/时钟/卡本身速度等级"

	echo "[6/7] 数据完整性(写回校验)"
	if command -v md5sum >/dev/null 2>&1; then
		M1=$(md5sum $F | cut -d' ' -f1)
		sync; echo 3 > /proc/sys/vm/drop_caches 2>/dev/null
		M2=$(md5sum $F | cut -d' ' -f1)
		[ "$M1" = "$M2" ] && pass "两次 md5 一致 ($M1)" || fail "md5 不一致! 数据错误(信号完整性/时序过激进)"
	else
		cmp -s $F $F && pass "cmp 自比一致(无 md5sum, 弱校验)" || fail "读取不稳定"
	fi
	rm -f $F; sync
else
	echo "[5/7] 跳过速度测试(没有可写挂载点)"; echo "[6/7] 跳过完整性测试"
fi

# ---------- ⑦ 内核有没有报错 ----------
echo "[7/7] 内核日志"
ERR=$(dmesg 2>/dev/null | grep -iE "mmc$MMCN|sdhci|dwmmc" | grep -iE "error|timeout|fail|retry|crc|switch to " | tail -8)
if [ -n "$ERR" ]; then
	echo "$ERR" | sed 's/^/         /'
	echo "$ERR" | grep -qiE "error|timeout|fail|crc" && \
		fail "内核有 MMC 错误(时序过激进/信号完整性/供电跌落)" || info "仅有降档/切换提示"
else
	pass "无 MMC 错误日志"
fi

[ -n "$TMPMP" ] && { umount $TMPMP 2>/dev/null; rmdir $TMPMP 2>/dev/null; }
echo "======== SD 自检结束: 失败 $FAIL 项 ========"
exit $FAIL
