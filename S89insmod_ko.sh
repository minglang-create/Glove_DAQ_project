#!/bin/sh
# =============================================================================
# S89insmod_ko.sh —— 开机自启【只加载内核模块】(2026-09-03)
#
# 【为什么要它】相机(rockit)依赖 /dev/mpi/* 与各 sensor .ko, 平时是 rkipc
#   启动时顺带 insmod 的; 我们不启 rkipc → 没人加载 → glove_daq_rv 开相机必失败。
#   本脚本只做 insmod, ★不启动任何应用★ —— 联调阶段用手动跑 glove_daq_rv,
#   自启只负责"把地基铺好", 应用启不启动由人决定。互不干扰。
#
# 【落位】common.mk 约定 S??*.sh 打进 /oem/usr/etc/init.d/, 开机由 RkLunch 环境执行。
#   编号 89 < 99(dualrtsp 点火器), 确保模块先于任何应用加载。
#
# 【逃生】touch /userdata/no_insmod && reboot → 跳过(极少需要)。
# =============================================================================
case "$1" in
start)
	[ -f /userdata/no_insmod ] && { echo "[insmod] no_insmod, 跳过"; exit 0; }
	if [ ! -e /dev/mpi/vsys ]; then
		echo "[insmod] 加载内核模块…"
		sh /oem/usr/ko/insmod_ko.sh
		[ -e /dev/mpi/vsys ] && echo "[insmod] /dev/mpi 就绪 ✓" || echo "[insmod] ★仍无 /dev/mpi★"
	else
		echo "[insmod] /dev/mpi 已在, 跳过"
	fi
	;;
stop) ;;
*) echo "用法: $0 {start|stop}" ;;
esac
