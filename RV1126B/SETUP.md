# 环境搭建(给第一次接手的团队)

> 本文档解决"从零到能编译、能烧录、能跑起来"这一件事。
> 假设你手上什么都没有,只有这个 git 仓库。
> 日常开发命令(参数、协议、排障)在 [README.md](README.md);出固件的坑在 [AUTOSTART.md](AUTOSTART.md)。

## 0. 这个仓库不是完整 SDK

**本仓库只包含我们自己写的代码**(RV1126B/ 下的应用程序 + 少量内核改动快照),
不包含 Rockchip 官方 SDK(内核源码全树、闭源媒体库、交叉编译工具链)。
原因:官方 SDK 解压后约 **30GB**,其中多个文件单个超过 GitHub 100MB 硬性限制
(编译产物 `vmlinux`、固件镜像 `rootfs.img`/`update.img`、打包压缩包等),
且 `media/`(闭源库)、`tools/`(第三方工具链)分发涉及许可证归属,不该进我们的仓库。

所以搭环境 = **先拿到官方 SDK,再把本仓库"种"进它的 `project/app/` 目录**。

## 0b. 快速路径:直接用打包好的 SDK(Google Drive)——推荐第一次接手的人走这条

我们把**已经配好、已经编译过一次**的整个 SDK 目录打成了一个包放在 Google Drive,拿到后不用做下面 §2–§7 的任何一步。

Drive 上有三样东西:

| 文件 | 内容 | 大小 |
|---|---|---|
| `glove_sdk_20260912.tar.xz`(+ `.sha256`) | 整个 `~/Aura-sdk`:官方 SDK + 本仓库(已在 `project/app/` 里)+ 内核/设备树改动 + rootfs overlay + 全部编译产物 | ≈10 GB |
| `glove_windows_tools_20260912.zip` | Windows 侧工具:RK 驱动 DriverAssitant、烧录工具 SocToolKit、adb | 76 MB |
| `glove_release_20260912_hwsync/` | 现成可烧的 `image/` 目录 + [FLASH_GUIDE.md](FLASH_GUIDE.md)(只想烧不想编译的人用这个就够) | 1.8 GB |

包里**故意没带**两样:Luckfox 出厂镜像目录 `IMAGE/`(想恢复官方系统时去 wiki 下载页取)和原始 SDK 压缩包
`Luckfox_Aura_SDK_260521.tar.gz`(同上)。其余一个文件不少。

在你的 Ubuntu 22.04(WSL2 或虚拟机;别的版本没验证过)里:

```bash
# 1) 装编译依赖(只做一次;= §1 的 SDK 自带清单 + 解压/打包会用到的 xz/rsync/bc/cpio)
sudo apt-get update && sudo apt-get install -y device-tree-compiler texinfo gperf make gzip \
     gcc-multilib g++-multilib xz-utils rsync bc cpio

# 2) 校验 + 解压到家目录(必须是 ~/Aura-sdk 这个相对位置;解压需要 sudo,包里有 root 属主的文件)
sha256sum -c glove_sdk_20260912.tar.xz.sha256
cd ~ && sudo tar -xf /path/to/glove_sdk_20260912.tar.xz      # 解完得到 ~/Aura-sdk,10–20 分钟

# 3) 确认能用(不用重编任何东西)
cd ~/Aura-sdk && ls -l .BoardConfig.mk && ls output/image/boot.img
```

然后直接跳到 **§8 核对** 和 **§10 日常迭代**。改应用代码只需 `sudo ./build.sh app` → 拷 app_out → `sudo ./build.sh firmware`(§10 原文)。
烧录看 [FLASH_GUIDE.md](FLASH_GUIDE.md)(Windows 工具在上面那个 zip 里)。

> 解压路径为什么要求 `~/Aura-sdk`:`media/` 等子模块的 cmake 缓存里记着编译时的绝对路径。上层应用开发不会重编这些模块,
> 所以放在任何用户的家目录下都没事;万一哪天真要重编 `media` 报路径错,删掉 `media/out` 重新 `sudo ./build.sh media` 即可。

## 1. 主机要求

- **Ubuntu 22.04**(或 WSL2 跑 Ubuntu 22.04;本项目实际就是这么开发的);
- 依赖包(SDK 自带的检查清单,`project/scripts/build-depend-tools.txt`):
  ```bash
  sudo apt-get update
  sudo apt-get install -y device-tree-compiler texinfo gperf make gzip \
                           gcc-multilib g++-multilib
  ```
  也可以先跳过,`./build.sh` 跑起来后会自动列出缺什么、给出对应的 `apt-get install` 命令。

## 2. 拿到官方 Luckfox Aura(RV1126B)SDK

从 Luckfox 官方渠道下载(wiki 首页 → 资料下载):
- 概览:https://wiki.luckfox.com/zh/Luckfox-Aura/
- SDK 编译说明(官方原文,和下面步骤对应):https://wiki.luckfox.com/zh/Luckfox-Aura/SDK-Image-Compilation
- 资料下载页:https://wiki.luckfox.com/zh/Luckfox-Aura/Downloads

下载后解压到你想放的目录(下文假设解压出的 SDK 根目录路径是 `~/Aura-sdk`,
**目录名随意,不影响任何编译路径**——所有相对路径都是"离 `project/app/` 多少层",
不依赖 SDK 根目录叫什么):

```bash
tar xf Luckfox_Aura_SDK_*.tar.gz -C ~/
mv ~/<解压出来的目录名> ~/Aura-sdk
cd ~/Aura-sdk
```

## 3. 选板级配置

```bash
ln -sf project/cfg/BoardConfig_Aura/BoardConfig-EMMC-RK801-Luckfox_Aura-Debian.mk .BoardConfig.mk
```

这一行等价于跑官方交互菜单 `./build.sh lunch` 后选"Luckfox_Aura → EMMC → Debian → 64bit"
四步——**我们只用这一个配置**(EMMC 启动、Debian 系统、64 位),直接建符号链接更快更不会选错。
(如果你想用官方菜单核对:`./build.sh lunch`,第一层选 `1`=custom,列出全部 8 个配置后
选 `EMMC-RK801-Luckfox_Aura-Debian.mk` 那一项。)

## 4. 把本仓库放进 SDK

```bash
cd ~/Aura-sdk/project/app
git clone git@github.com:minglang-create/Glove_DAQ_project.git
# 目录名(Glove_DAQ_project)也随意, 只要求"在 project/app/ 下一层"——
# 因为 RV1126B/Makefile 里写的相对路径是"距 app/ 两层"(project/app/<任意名>/RV1126B/)。
```

## 5. 应用内核改动快照

我们对内核 dts/驱动做过一些改动(双摄硬同步、SD 卡 UART0 引脚修复等),
存在 `RV1126B/board_kernel_mods/` 里,需要"倒"回 SDK 的内核树才会生效:

```bash
cd ~/Aura-sdk/project/app/Glove_DAQ_project/RV1126B/board_kernel_mods
sh restore_to_sdk.sh
```

## 6. 应用 rootfs overlay(开机自启配置)

```bash
cd ~/Aura-sdk/project/app/Glove_DAQ_project/RV1126B/board_rootfs_overlay
sh install_to_sdk.sh
```

这一步把 `/etc/.rkapp=GLOVEDAQ` 装进 SDK 的 overlay 机制,并在 `.BoardConfig.mk`
里设好 `RK_POST_OVERLAY`——**必须在第 3 步(选板级配置)之后**执行,
因为它要读 `.BoardConfig.mk` 才知道该改哪个文件。

## 7. 首次完整编译

```bash
cd ~/Aura-sdk
sudo ./build.sh sysdrv    # uboot + kernel + rootfs, 十几分钟起
sudo ./build.sh media     # 官方闭源媒体库(rockit/rkaiq), 已预编译, 很快
sudo ./build.sh app       # 编译我们的应用(含 RV1126B/glove_daq_rv)

# ★必做, SDK 自身的一个断链★:官方 Makefile 里 project/app/Makefile:23 那句
#   $(call MAROC_COPY_PKG_TO_APP_OUTPUT, ...) 调用的宏在整个 SDK 里没有定义,
#   $(call) 展开为空 → build_app 编好的东西不会自动进打包目录 → 必须手动补一步:
sudo cp -rfa project/app/out/* output/out/app_out/

sudo ./build.sh firmware  # 打固件(读 app_out, 上一步没做这里就是旧程序)
```

**为什么每一步都要 `sudo`**:这个 SDK 对 Debian rootfs 目标强制检查
`id -u = 0`(build.sh 源码里写死的),不加 sudo 直接报错退出,不是我们的选择。

编译完成后 `output/image/` 下会有 `boot.img`、`rootfs.img`、`oem.img`、`update.img`。

## 8. 核对(别只信编译日志)

```bash
cat output/out/rootfs_glibc_rv1126b/etc/.rkapp          # 应输出 GLOVEDAQ
ls -l output/out/oem/usr/bin/{glove_daq_rv,RkLunch-GLOVEDAQ.sh}   # 应存在, 时间是刚才
```

## 9. 烧录

**这套 SDK 的 `update.img` 不含 rootfs**(rootfs 是"占满剩余空间"型分区,`tools/linux/Linux_Pack_Firmware/mk-update_pack.sh`
对这类分区一律跳过,所以 `update.img` 只有 ~300MB)。而自启开关 `/etc/.rkapp=GLOVEDAQ` 正好在 rootfs 里——
**只烧 `update.img` 的板子上电跑的还是官方 rkipc,不会自启**。

正确做法:把 `output/image/` 整个目录当"整包",用 SocToolKit 的 **Download(分区下载)** 模式
`Search Path...` 选中该目录 → 8 个分区全勾 → Download。逐步图文见 [FLASH_GUIDE.md](FLASH_GUIDE.md)
(也是发给拿镜像不编译的团队的那份)。进 Loader 模式的按法以官方 wiki 为准:按住 RESET → 按住 BOOT → 松 RESET → 识别后松 BOOT。

烧完上电,不用做任何配置就会自动跑起手套采集程序(参数/排障见 [README.md](README.md)、
[AUTOSTART.md](AUTOSTART.md))。

## 10. 之后的日常迭代

只改了 `RV1126B/*.c`(应用代码),不用走上面全部步骤,直接:

```bash
cd ~/Aura-sdk/project/app/Glove_DAQ_project/RV1126B
make                                          # 本地编译(不进 SDK 打包)
adb push glove_daq_rv /oem/usr/bin/ && adb shell "chmod +x /oem/usr/bin/glove_daq_rv; killall glove_daq_rv"
```

改了内核/dts(`board_kernel_mods/` 下的文件),要重新走 §5→§7(第 7 步只需
`sysdrv`+`app`+补 app_out+`firmware`,不用重跑 `media`)。

## 常见卡点

| 现象 | 原因 |
|---|---|
| 固件里是旧版程序,新功能没生效 | 忘了做第 7 步的 `sudo cp -rfa` 那一行 |
| 开机自启没生效,STM32 没收到 0x5501 | 忘了第 6 步的 overlay,或没走 §3 就跑了 §6 |
| `make` 报找不到 `../../Makefile.param` | 仓库没放在 `project/app/` **正下一层**,层级不对 |
| 相机 dts 改了但不生效 | 忘了第 5 步的 `restore_to_sdk.sh`,或改完没重新 `sudo ./build.sh kernel` |
