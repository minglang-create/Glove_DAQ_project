#!/bin/sh
# =============================================================================
# RkLunch-GLOVEDAQ.sh —— 上电自启:手套数据采集(glove_daq_rv)
#
# 【怎么被调用】开机 systemd → initd-seq.service → /usr/bin/run_initd.sh
#   → /etc/init.d/S21appinit 读 /etc/.rkapp:
#      内容 = "GLOVEDAQ" → 执行本脚本;官方 rkipc 从此【根本不启动】(不抢相机)。
#   /etc/.rkapp 的内容由 SDK overlay 固化进 rootfs.img(见 board_rootfs_overlay/),
#   所以【重烧固件后依然生效】,不需要手动 echo。
#
# 【为什么要自己 insmod + 解锁】取代 rkipc 后没人替我们跑 insmod_ko.sh,
#   /dev/mpi 不存在 → rockit 起不来;且 clr_unready_dev 不写 → rkaiq 枚举不到
#   sensor(踩过坑, 见 dual_cam/RkLunch-DUALRTSP.sh 同款处理)。两步都必须做。
#
# 【★不碰 USB gadget★】只起采集程序,不建 functionfs gadget → adb 全程常在,
#   永不自锁。自启方案一律不动 gadget(定规, 见 PROJECT_STATE §8)。
#
# 【工人使用】上电 → 本脚本把程序跑起来 → 短按开采 → 长按 2 秒暂停(段收口+fsync, 可直接
#   断电)→ 短按重新开采(新开一段)。按键不会退出程序; 下面的循环只是崩溃兜底。
#
# 日志: /userdata/glove_daq.log
# 逃生口(调试用): touch /userdata/glove_noauto && reboot
#   → 只加载内核模块、不起采集程序,留干净环境给手动跑 glove_daq_rv
# =============================================================================
LOG=/userdata/glove_daq.log
APP=/oem/usr/bin/glove_daq_rv

# 耗时活丢后台:S21appinit 在开机序列里,阻塞会拖慢启动
(
	# 1) 等 userdata 挂好(日志和落盘都在这里)
	cnt=0
	while [ $cnt -lt 50 ]; do
		mount | grep -qw userdata && break
		cnt=$((cnt + 1)); sleep 0.1
	done

	echo "==== RkLunch-GLOVEDAQ 自启 $(date) ====" > $LOG

	# 2) 加载内核模块(/dev/mpi 等) + 解锁 sensor 枚举(滤掉 insmod_ko 探 WiFi 的 sdio 噪音)
	[ -f /oem/usr/ko/insmod_ko.sh ] && ( cd /oem/usr/ko && sh insmod_ko.sh 2>&1 | grep -v sdio ) >> $LOG
	echo 1 > /sys/module/video_rkcif/parameters/clr_unready_dev 2>/dev/null
	echo 1 > /sys/module/video_rkisp/parameters/clr_unready_dev 2>/dev/null
	if [ -e /dev/mpi/vsys ]; then
		echo "[glovedaq] 模块就绪 /dev/mpi ✓ uptime=$(cut -d. -f1 /proc/uptime)s" >> $LOG
	else
		echo "[glovedaq] ★/dev/mpi 缺失, 相机会起不来★" >> $LOG
	fi

	# 2.5) 挂 SD 卡到固定点 /mnt/sd(落盘目标, exFAT 内核内建)。卡不在时这里不报错——
	#      由 glove_daq_rv 的存储自检判定并拒绝采集(不发 0x5501 → STM32 LED 不亮 → 工人可见)。
	#      noatime: 少写元数据。不用 -o sync: 那会把 SD 卡的每次抖动直接暴露给应用。
	mkdir -p /mnt/sd
	if ! mount | grep -q ' /mnt/sd '; then
		if [ -b /dev/mmcblk1p1 ]; then
			if mount -o noatime /dev/mmcblk1p1 /mnt/sd >> $LOG 2>&1; then
				echo "[glovedaq] SD 已挂 /mnt/sd, 可用 $(df -h /mnt/sd | awk 'NR==2{print $4}')" >> $LOG
			else
				echo "[glovedaq] ★SD 挂载失败(文件系统损坏? 需 PC 上检查)★" >> $LOG
			fi
		else
			echo "[glovedaq] ★未检测到 SD 卡(/dev/mmcblk1p1)★ 采集程序自检将拒绝采集" >> $LOG
		fi
	fi

	# 3) 逃生口:调试时不自启程序, 但模块已加载、SD 已挂(手动跑 glove_daq_rv 即可)
	if [ -f /userdata/glove_noauto ]; then
		echo "[glovedaq] glove_noauto 标志存在, 跳过自启(手动模式)" >> $LOG
		exit 0
	fi

	[ -x $APP ] || { echo "[glovedaq] ★$APP 不存在或不可执行★" >> $LOG; exit 1; }

	# 4) 起采集程序。正常情况它永不退出(按键只暂停/开采); 异常退出则自动重起(崩溃兜底)。
	#    异常快退(<5s)时退避到 10s, 防止崩溃循环把日志刷爆。
	while true; do
		t0=$(cut -d. -f1 /proc/uptime)
		echo "[glovedaq] 启动 $APP (uptime ${t0}s)" >> $LOG
		$APP >> $LOG 2>&1
		rc=$?
		t1=$(cut -d. -f1 /proc/uptime)
		echo "[glovedaq] 退出(exit=$rc, 运行 $((t1 - t0))s)" >> $LOG
		if [ $((t1 - t0)) -lt 5 ]; then
			echo "[glovedaq] 快速退出, 退避 10s" >> $LOG; sleep 10
		else
			sleep 3
		fi
	done
) &
