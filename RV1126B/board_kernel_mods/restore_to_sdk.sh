#!/bin/sh
# =============================================================================
# restore_to_sdk.sh —— 把本快照目录的文件写回 SDK 内核树(换机器/重装 SDK 后用)
#   用法: sh restore_to_sdk.sh [SDK内核根目录]
#   ★会覆盖 SDK 里的同名文件★ 执行前确认 SDK 内核树没有你想保留的改动。
# =============================================================================
cd "$(dirname "$0")" || exit 1        # 先进脚本所在目录, 相对路径才稳定(不随调用者CWD变)
K=${1:-../../../../../sysdrv/source/kernel}
[ -d "$K/arch" ] || { echo "内核目录不对: $K"; exit 1; }
printf "将覆盖 %s 下的快照文件, 继续? [y/N] " "$K"; read -r a
[ "$a" = y ] || [ "$a" = Y ] || { echo 取消; exit 0; }
find . -type f ! -name '*.sh' | while read -r f; do
    dst="$K/${f#./}"; mkdir -p "$(dirname "$dst")"
    cp -f "$f" "$dst" && echo "  ✓ → ${f#./}"
done
echo "[*] 写回完成。之后需重编内核: cd ~/Aura-sdk && sudo ./build.sh kernel"
