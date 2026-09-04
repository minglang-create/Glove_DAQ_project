# Glove_DAQ_RV1126B_SDK

数据手套采集系统的 **RV1126B(下位机)侧全部代码** —— 双 IMX415 硬同步采集 + STM32G474
手套传感器链路 + 帧↔周期号自动对齐 + 分段落盘。

> 系统构成:每只手套 = **RV1126B**(双相机 + SPI 主机)+ **STM32G474**(传感器主控 +
> SPI 从机 + XVS 时基)。本仓是 RV1126B 侧;STM32 侧固件另仓。

## 快速上手

```bash
# 1) 放进 Rockchip SDK 的 app 目录(本仓即该目录本身)
#    ~/Aura-sdk/project/app/Glove_DAQ_RV1126B_SDK/
# 2) 交叉编译(产出 ARM64 可执行 glove_daq_rv)
make
# 3) 推到板子
adb push glove_daq_rv /oem/usr/bin/ && adb shell chmod +x /oem/usr/bin/glove_daq_rv
# 4) 跑(板上; 每次上电需先有 /dev/mpi, 见 AUTOSTART.md)
glove_daq_rv                 # 正式流程(按键触发)
glove_daq_rv -X -o none      # 纯手套链路调试(不开相机)
glove_daq_rv -w imu          # IMU 数据可视化
```

## 文档

| 文件 | 内容 |
|---|---|
| [ARCHITECTURE.md](ARCHITECTURE.md) | **总体架构**:模块地图、线程模型、生命周期、对齐引擎原理、落盘格式 |
| [docs/PROTOCOL_V2.md](docs/PROTOCOL_V2.md) | **SPI 协议契约**(与 STM32 侧的唯一真理源):事务模型、包表、数据帧字节布局、CRC |
| [AUTOSTART.md](AUTOSTART.md) | 开机自启机制(systemd + run_initd 只扫 `/etc/init.d/`)与干净方案 |

## 目录结构

```
main.c            总装(参数/信号/exec 重启), 保持薄
daq_fsm.{c,h}     生命周期状态机: 自检→主从分配→启相机→运行⇄暂停
glove_link.{c,h}  SPI 事务层: 恒定 2690B 全双工 / 小包 / 数据帧 / CRC-16 ARC / PA1 握手
cam_pipeline.{c,h} 双 IMX415 采集管线(ISP→VI→VENC H.265→配对), dual_cam 血统
align.{c,h}       相机帧 ↔ CYCLE 自动标定(PA1 沿时间戳锚定 + 中位数 offset)
recorder.{c,h}    分段落盘 seg_<n>_<boottime>/{cam0/1.h265, pairs.csv, glove.bin/csv}
glove_view.{c,h}  终端可视化(-w imu/joint/tactile/all)
button.{c,h}      按键(GPIO0_A0 低有效, 30ms 去抖, 2s 长按)
S89insmod_ko.sh   开机自启: 只加载内核模块(放板上 /etc/init.d/)
selfcheck_*.sh    千兆以太网 / SD3.0 板级自检脚本
board_kernel_mods/  内核改动快照(dts/dtsi/imx415.c/defconfig)+ 同步与写回脚本
```

## 内核依赖

本程序依赖内核侧改动(设备树选型 + imx415 主从同步驱动),快照在
`board_kernel_mods/`,镜像 SDK 内的真实路径。

```bash
cd board_kernel_mods
sh sync_from_sdk.sh      # 从 SDK 内核树拉最新内容进快照(改完内核后跑, 再 commit)
sh restore_to_sdk.sh     # 把快照写回 SDK 内核树(换机器/重装 SDK 后跑), 之后重编内核
```

**设备树三选一**(在 `rv1126b-luckfox-aura.dts` 里整块注释切换):

| dtsi | 相机同步拓扑 | 适用 |
|---|---|---|
| `...-dual-cam-daq-hwsync.dtsi` | cam0=主 / cam1=从(相机互同步) | V2/V3 板,已实测 dpts −6µs |
| `...-dual-cam-daq-mcusync.dtsi` | 双从(等外部 XVS) | V4 板 + STM32 供 XVS |
| `...-aura-v4.dtsi` | 板级:千兆 ETH(RTL8211F)+ SD3.0(UHS) | **仅 V4 板**,V2/V3 要注释掉 |

⚠ **已知硬约束(2026-09-04 实测)**:IMX415 从机**启动**时必须同时收到 **XVS 和 XHS**
两个信号(手册 slave 模式要求);只给 XVS、XHS 静态高 → 从机永不出帧。V4 板的 XHS 网
只在两颗相机间互连、未引到 STM32,故 V4 需选相机主从拓扑,或改板把 XHS 引给 STM32。

## 硬件引脚(V4)

| 信号 | RV1126B | 说明 |
|---|---|---|
| SPI0_M2 | J16=CLK / P5=MOSI / T3=MISO / P6=CSN0 | 到 STM32(RV 为主机) |
| PA1 (DATA_READY) | 球 K13 = GPIO0_A4 | STM32→RV,高=有包可读 |
| 按键 SW3 | 球 A2 = GPIO0_A0 | 低有效 |
| XVS | 相机 XVS 网 ↔ STM32 PB10(经 TXS0101 电平转换) | 60Hz 时基 |
