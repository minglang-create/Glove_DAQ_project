# 烧录指南(给拿到镜像的团队,不需要懂底层)

> 目标:把"数采手套 RV1126B 固件"烧进板子,上电就自动跑采集程序。
> 全程在 Windows 上操作,只用两个官方工具,10 分钟。

## 0. 你会拿到什么

一个文件夹 `image/`,里面是 8 个分区镜像 + 校验文件(**缺一不可,别改名,别只挑其中几个**):

| 文件 | 内容 | 大小 |
|---|---|---|
| `download.bin` | 烧录引导(工具用) | 0.4 MB |
| `env.img` | 分区表 | 32 KB |
| `idblock.img` | 一级引导 | 0.3 MB |
| `uboot.img` | U-Boot | 0.5 MB |
| `boot.img` | 内核 + 设备树(**相机主从配置在这里:cam0 主 / cam1 从**) | 9 MB |
| `rootfs.img` | Debian 根文件系统(**自启开关 `/etc/.rkapp=GLOVEDAQ` 在这里**) | 1.75 GB |
| `oem.img` | 采集程序 `glove_daq_rv`、相机库、自启脚本 | 250 MB |
| `userdata.img` | 用户数据分区(初始为空) | 35 MB |
| `SHA256SUMS` | 校验值 | — |

为什么不是一个 `update.img`:这套 SDK 打包 `update.img` 时会**跳过 rootfs**(rootfs 是"占满剩余空间"型分区,
打包脚本 `mk-update_pack.sh` 对这类分区一律不打进去),所以只烧 `update.img` 的板子**不会自启**。
用下面的"分区下载"方式一次把 8 个文件全烧进去,最省事也最不容易错。

下载后先校验(PowerShell,在 `image/` 目录里):

```powershell
certutil -hashfile rootfs.img SHA256
```

对照 `SHA256SUMS` 里 `rootfs.img` 那一行,一致即可(其它文件同理,至少查 rootfs/oem/boot 三个大的)。

## 1. 准备(只做一次)

1. 官方资料页 <https://wiki.luckfox.com/zh/Luckfox-Aura/Downloads> 下载两个工具
   (或直接找我们要:`DriverAssitant_v5.13` 和 `SocToolKit_V2.2`):
   - **DriverAssitant**:解压 → 右键 `DriverInstall.exe` **以管理员身份运行** → 安装驱动 → **重启电脑**。
   - **SocToolKit**:解压即用,不用安装。
2. 一根能传数据的 USB Type-C 线(平时 `adb` 用的那根就行),接板子上平时接 adb 的那个 USB 口。

## 2. 烧录(每块板子都这样做)

1. 打开 `SocToolKit.exe` → 弹出芯片选择 → 选 **RV1126B** → OK。
2. 顶部停在 **Download** 页;左上角选 **USB**。
3. 让板子进入烧录模式(官方 wiki 的按法):
   **按住 RESET → 再按住 BOOT → 松开 RESET → 等 USB 下拉框里出现 `Loader` 或 `Maskrom` 字样 → 松开 BOOT。**
   如果下拉框一直是空的:换线、换电脑上的 USB 口、确认驱动装了并重启过电脑,然后重做这一步。
4. 点 **Search Path...** → 选中 `image` 文件夹 → 表格会自动列出
   `DownloadBin / env / idblock / uboot / boot / rootfs / oem / userdata` 8 行(工具按 `env.img` 里的分区表生成)。
5. **把 8 行全部打勾**(左上角表头的总勾选框一次全选),勾上右下角的 **reset**。
6. 点 **Download**。右侧日志会逐行滚 `Download xxx Start / Success`;`rootfs` 1.75 GB 要几分钟,别拔线。
7. 最后看到 `Download Firmware Success` 之类的成功字样、板子自动重启,就完成了。

> 不要用 **Firmware... / Upgrade** 那一行——那是烧单个 `update.img` 用的,我们的包不走这条路。

## 3. 烧完怎么确认(可选,1 分钟)

板子重启后,用 adb(不需要密码,插线即 root):

```bash
adb devices                                   # 能看到一个 device
adb shell cat /etc/.rkapp                     # 期望输出: GLOVEDAQ
adb shell "dmesg | grep 'camera sync mode'"   # 期望: 3-001a ... internal_master ; 4-001a ... slave
adb shell tail -5 /userdata/glove_daq.log     # 期望看到 "模块就绪 /dev/mpi ✓" 和 "启动 /oem/usr/bin/glove_daq_rv"
```

## 4. 上电以后它会做什么(给上层应用同事的一段话)

- 上电约 10 秒后 `glove_daq_rv` 自动运行,等待手套 STM32 就绪、SD 卡就绪(**SD 卡不在会拒绝采集**,LED 不亮)。
- 手套按键:**短按 = 开始采集**;**长按 ≥2 秒 = 结束本段、数据落盘、整机自复位**,看到日志"数据已落盘 ✓"后可直接断电。
- 数据在 SD 卡(exFAT)`/mnt/sd/daq/<年月日时分>_seg_NNN/`,每段 6 个文件(两路视频 + 手套/关节/对齐 CSV),
  格式说明见 [DATA_FORMAT.md](docs/DATA_FORMAT.md),PC 端校验工具 `tools/daq_check.py`。
  取数据:拔卡读,或 `adb pull /mnt/sd/daq/<段目录> D:/somewhere/`(Windows 的 adb 目标路径要写 `D:/...` 形式)。
- 开发调试时不想让它自启(自己手动跑程序):`adb shell "touch /userdata/glove_noauto && reboot"`;
  删掉这个文件再重启就恢复自启。**重新烧录会清空 `/userdata`**,这个开关和"虚拟时间"计数都会归零。
- 更多:参数与排障 [README.md](README.md),自启机制 [AUTOSTART.md](AUTOSTART.md),
  外接关节板串口 [EXT_UART.md](docs/EXT_UART.md)。

## 5. 常见问题

| 现象 | 原因 / 处理 |
|---|---|
| SocToolKit 的 USB 下拉框始终为空 | 驱动没装好(装完必须重启)、线只能充电不能传数据、Loader 按法顺序错了 |
| 只烧了 `update.img`,上电不自启、跑的是官方 rkipc | `update.img` 不含 rootfs,`/etc/.rkapp` 没写进去。按本文第 2 节把 8 个文件全烧一遍 |
| 烧完 `adb devices` 看不到 | 等 30 秒再试;换 USB 口;板子上电后 LED 有没有亮 |
| `dmesg` 里相机 `Unexpected sensor id` | 相机模组硬件问题(和镜像无关),联系硬件同事 |
| 想回到 Luckfox 官方系统 | 从官方 Downloads 页拿官方镜像,同样方法烧回去即可 |

---
版本:2026-09-12 · 分支 `cam-xvs-simple-start` · 相机 hwsync(cam0 主 / cam1 从)· H.264 默认。
从源码重建这套镜像的步骤见 [SETUP.md](SETUP.md)。
