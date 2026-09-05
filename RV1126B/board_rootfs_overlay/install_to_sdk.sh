#!/bin/sh
# =============================================================================
# install_to_sdk.sh —— 把本 overlay 装进 SDK, 使其固化进 rootfs.img
#
# 【官方机制】(来源: Luckfox wiki《SDK 镜像编译》+ build.sh:3023 post_overlay)
#   build.sh 的 build_firmware() 会调 post_overlay():
#     rsync $(dirname .BoardConfig.mk)/overlay/$RK_POST_OVERLAY/* → rootfs 暂存目录
#   所以只要:①把文件按 rootfs 结构放进 overlay/<名字>/  ②板级配置里设
#   RK_POST_OVERLAY=<名字>  ③./build.sh firmware  → 文件就进 rootfs.img,
#   ★重烧固件后依然存在★(不像手动往 /etc 里放, 重烧就被冲掉)。
#
# 【本 overlay 装什么】只有一个文件: etc/.rkapp = "GLOVEDAQ"
#   → 开机 S21appinit 据此执行 /oem/usr/bin/RkLunch-GLOVEDAQ.sh(在 OEM 分区,
#     由 app 的 Makefile 自动打包)→ 加载模块 + 解锁 + 起 glove_daq_rv。
#   官方 rkipc 因此根本不启动, 不会抢相机。
#
# 用法: sh install_to_sdk.sh [SDK根目录]     默认 = 本目录往上 5 层
# =============================================================================
set -e
cd "$(dirname "$0")"                       # 先进脚本所在目录, 相对路径才稳定
SDK=${1:-../../../../..}
NAME=overlay-glove-daq

[ -f "$SDK/build.sh" ] || { echo "★不是 SDK 根目录: $SDK★"; exit 1; }
BC=$(readlink -f "$SDK/.BoardConfig.mk" 2>/dev/null)
[ -f "$BC" ] || { echo "★未找到 .BoardConfig.mk, 请先跑 ./build.sh lunch★"; exit 1; }
DST=$(dirname "$BC")/overlay/$NAME

echo "[1/2] 拷 overlay 文件 → $DST"
mkdir -p "$DST"
tar cf - etc | tar xf - -C "$DST"          # 保持目录结构, 只拷 etc/
find "$DST" -type f | sed "s#^#      #"

echo "[2/2] 在板级配置里启用 RK_POST_OVERLAY=$NAME"
echo "      配置文件: $BC"
if grep -qE "^export RK_POST_OVERLAY=" "$BC"; then
	sed -i "s#^export RK_POST_OVERLAY=.*#export RK_POST_OVERLAY=\"$NAME\"#" "$BC"
	echo "      已更新现有行"
elif grep -qE "^# *export RK_POST_OVERLAY=" "$BC"; then
	sed -i "0,/^# *export RK_POST_OVERLAY=.*/s##export RK_POST_OVERLAY=\"$NAME\"##" "$BC"
	echo "      已取消注释并设值"
else
	printf '\nexport RK_POST_OVERLAY="%s"\n' "$NAME" >> "$BC"
	echo "      已追加到文件末尾"
fi
grep -nE "^export RK_POST_OVERLAY=" "$BC" | sed "s#^#      #"

cat <<TIP

装好了。接下来在 SDK 根目录执行(把 overlay 打进 rootfs.img):
    cd $(cd "$SDK" && pwd)
    sudo ./build.sh firmware
然后烧 output/image/ 下的 update.img(或单独烧 rootfs.img)。
★注意★ 应用与 RkLunch-GLOVEDAQ.sh 在 OEM 分区, 需先 make 编译 app 再打包固件。
TIP
