# RV1126B 侧 —— 下位机采集系统

双 IMX415 硬同步采集 + STM32 手套链路 + 帧↔周期号自动对齐 + 分段落盘。
系统全貌见[仓库根 README](../README.md);协议契约见 [docs/PROTOCOL_V2.md](docs/PROTOCOL_V2.md)。

## 快速上手

```bash
# 1) 本目录需放在 Rockchip SDK 的 app 下两层:
#    ~/Aura-sdk/project/app/<clone 目录名>/RV1126B/
#    (Makefile 按"距 app/ 两层"写相对路径, 层级变了要同步改 Makefile 前两行)
# 2) 恢复内核改动到 SDK 并重编内核(换机器/新 SDK 时必做一次)
cd board_kernel_mods && sh restore_to_sdk.sh && cd ..
cd ~/Aura-sdk && sudo ./build.sh kernel     # 之后烧 boot.img
# 3) 交叉编译本程序(产出 ARM64 可执行 glove_daq_rv)
make
# 4) 推到板子并运行
adb push glove_daq_rv /oem/usr/bin/ && adb shell chmod +x /oem/usr/bin/glove_daq_rv
adb shell /oem/usr/bin/glove_daq_rv          # 正式流程(按键触发)
```

常用参数:

| 参数 | 作用 |
|---|---|
| `-X -o none` | 纯手套链路调试(不开相机、不落盘) |
| `-V` | 逐帧校验 STM32 假数据(协议实现验收) |
| `-w imu` / `joint` / `tactile` / `all` | 终端可视化视图 |
| `-A` | 台架直通(STM32 AUTOSTART=1 时) |
| `-o <dir>` | 落盘基目录(默认 `/mnt/sd/daq`,即 SD 卡;`none`=不落盘) |
| `-U <dev>[:baud][:trig\|free]` | 外接转接板 21 路关节 ADC 串口(默认不启用)。例 `-U /dev/ttyS0:460800:trig`,**需 dts 变体让出 UART0**,见 [docs/EXT_UART.md](docs/EXT_UART.md) |
| `-F <sec>` / `-T <min>` / `-M <gb>` | fsync 间隔(5)/自动切段(10 分钟)/SD 剩余空间阈值(2GB;`-M 0`=不检查存储,允许落 eMMC 调试) |
| `-G c:l` / `-K c:l` | PA1 / 按键 GPIO(默认 `0:4` / `0:0`) |

⚠ 烧录固件后**上电即用**:开机自动加载模块 + 自动启动 `glove_daq_rv`,按键即可采集。
机制与出固件流程(含一个必踩的 SDK 坑)见 [AUTOSTART.md](AUTOSTART.md)。
调试时不想让它自启:`touch /userdata/glove_noauto && reboot`。

**数据格式与验收工具**:[docs/DATA_FORMAT.md](docs/DATA_FORMAT.md)、`../tools/daq_check.py`(PC 端,纯 Python 标准库)

## 目录结构

```
main.c              总装(参数/信号/exec 重启), 保持薄
daq_fsm.{c,h}       生命周期状态机: 自检→主从分配→启相机→运行⇄暂停
glove_link.{c,h}    SPI 事务层: 恒定 2690B 全双工 / 小包 / 数据帧 / CRC-16 ARC / PA1 握手
cam_pipeline.{c,h}  双 IMX415 采集管线(ISP→VI→VENC H.265→时间戳配对), dual_cam 血统
align.{c,h}         相机帧 ↔ CYCLE 自动标定(PA1 沿内核时间戳锚定 + 中位数 offset)
ext_uart.{c,h}      外接转接板 21 路关节 ADC(UART0, PA1 沿发 0x00 触发一问一答), 见 docs/EXT_UART.md
recorder.{c,h}      SD 卡分段落盘: 热路径只 fwrite 进页缓存; flush 线程做 fsync/切段/查空间
                    (exFAT 必须周期 fsync 否则拔卡后文件大小不对); 卡不在/空间不足 = 自检失败
glove_view.{c,h}    终端可视化
button.{c,h}        按键(GPIO0_A0 低有效, 30ms 去抖, 2s 长按)
RkLunch-GLOVEDAQ.sh 开机自启入口(打包进 /oem/usr/bin/): 加载模块+解锁 sensor+
                    循环起 glove_daq_rv; 逃生口 /userdata/glove_noauto
selfcheck_*.sh      千兆以太网 / SD3.0 板级自检
docs/               协议契约
board_rootfs_overlay/ rootfs 固化(etc/.rkapp=GLOVEDAQ)+ install_to_sdk.sh
                    → 让自启配置【重烧固件后依然生效】, 见 AUTOSTART.md
board_kernel_mods/  内核改动快照(镜像 SDK 真实路径)+ sync/restore 脚本
  arch/arm64/boot/dts/rockchip/   设备树: 主 dts + 6 个 dtsi
  arch/arm64/configs/             defconfig(含 REALTEK_PHY 等)
  drivers/media/i2c/imx415.c      相机驱动(主从同步 sync_mode 实现)
```

## 内核依赖

```bash
cd board_kernel_mods
sh sync_from_sdk.sh      # 改完 SDK 内核后, 把最新内容拉进快照, 再 git commit
sh restore_to_sdk.sh     # 把快照写回 SDK 内核树, 之后重编内核
```

**设备树组合**(在 `rv1126b-luckfox-aura.dts` 里整块注释切换):

| dtsi | 相机同步拓扑 | 适用 |
|---|---|---|
| `...-dual-cam-daq-hwsync.dtsi` | cam0=主 / cam1=从(相机互同步) | V2/V3,已实测 dpts 恒 −6µs |
| `...-dual-cam-daq-mcusync.dtsi` | 双从(等外部 XVS) | V4 + STM32 供 XVS |
| `...-aura-v4.dtsi` | 板级:千兆 ETH + SD3.0(UHS) | **仅 V4**,V2/V3 必须注释掉 |

## ⚠ 已知硬约束(2026-09-04 实测坐实)

**IMX415 从机模式【启动】时必须同时收到 XVS 和 XHS 两个信号**(手册 slave 模式明文要求,
XHS = 1H 周期行同步 ≈138kHz)。只给 XVS、XHS 静态高 → 从机永不出帧(`frame amount:-1`)。

实验证据:两颗都 slave 时 i2c 把 cam0 切 master 同时输出 XVS+XHS → cam1(仍 slave)
**立即出图且两颗帧数完全相同**;而 STM32 只供 XVS 时从机零帧。

注:早前"XHS 不需要"的结论是误读——那个实验证明的只是"**已锁定后维持**不需要 XHS"。

**对 V4 的影响**:V4 的 XHS 网只在两颗相机间互连、**未引到 STM32**,故 V4 板要么选相机
主从拓扑(cam0 供 XVS+XHS),要么改板把 XHS 引给 STM32。

## 硬件引脚(V4)

| 信号 | RV1126B | 说明 |
|---|---|---|
| SPI0_M2 | J16=CLK / P5=MOSI / T3=MISO / P6=CSN0 | 到 STM32(RV 为主机) |
| PA1(DATA_READY) | 球 K13 = GPIO0_A4 | STM32→RV,高=有包可读 |
| 按键 SW3 | 球 A2 = GPIO0_A0 | 低有效 |
| XVS | 相机 XVS 网 ↔ STM32 PB10(经 TXS0101 电平转换) | 60Hz 时基 |
| UART0(调试口/外接 ADC) | 球 A3=TX / A4=RX = **GPIO0_B3/B4(复用 m2)** | ⚠ 不是 SoC 默认的 m0,m0 与 SDMMC0_D0/D1 冲突 |
| XHS | 仅两相机互连(未接 STM32) | 见上方硬约束 |
