#!/bin/sh
# =============================================================================
# sync_from_sdk.sh —— 把 SDK 内核树里【本目录已快照的那些文件】的最新内容拷回来
#   用法: sh sync_from_sdk.sh [SDK内核根目录]
#   默认内核根 = ../../../../../sysdrv/source/kernel
#     (本目录 = project/app/Glove_DAQ_RV1126B_SDK/board_kernel_mods)
#   只更新已存在的文件, 不去 SDK 里乱抓新文件。新增文件请先手动 cp --parents 进来。
# =============================================================================
K=${1:-../../../../../sysdrv/source/kernel}
[ -d "$K/arch" ] || { echo "内核目录不对: $K"; exit 1; }
cd "$(dirname "$0")" || exit 1
find . -type f ! -name '*.sh' | while read -r f; do
    src="$K/${f#./}"
    if [ -f "$src" ]; then cp -f "$src" "$f" && echo "  ✓ ${f#./}"
    else echo "  ✗ SDK 里没有 ${f#./}"; fi
done
echo "[*] 同步完成。记得 git add -A && git commit"
