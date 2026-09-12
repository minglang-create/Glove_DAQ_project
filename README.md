# Glove_DAQ_project —— 数据手套采集系统

双相机 + 全手传感器的同步数据采集系统。每只手套由两颗芯片协同:

```
┌─────────────── 一只手套 ───────────────┐
│  RV1126B                STM32G474      │
│  ├ 双 IMX415 硬同步      ├ 5路CAN节点   │
│  ├ SPI 主机(读手套)     ├ I2C IMU      │
│  ├ 帧↔周期号对齐        ├ SPI 从机     │
│  └ 分段落盘             └ XVS 60Hz时基 │
└────────────────────────────────────────┘
      ↕ SPI(2690B事务) + PA1(就绪) + XVS(时基)
```

**核心思想**:XVS 60Hz 方波是全系统唯一时基。每个 XVS 周期,相机曝光一帧、STM32
采一轮全量传感器并打上**周期号(CYCLE)**;RV 侧把相机帧的 PTS 与周期号自动标定对齐,
于是"第 N 帧画面"和"第 N 拍传感器数据"精确成对。两只手套共享同一根 XVS,
CYCLE 同源 → 跨手套直接按 CYCLE 对号。

## 仓库结构

| 目录 | 内容 |
|---|---|
| **[RV1126B/](RV1126B/)** | 下位机(Rockchip RV1126B)侧全部代码 + 内核改动快照 + 板级自检脚本 |
| **[STM32/](STM32/)** | 手套 MCU(STM32G474)侧固件(待推送) |

**协议契约**(两侧唯一真理源):[RV1126B/docs/PROTOCOL_V2.md](RV1126B/docs/PROTOCOL_V2.md)

## 从哪读起

1. 本文件 —— 系统全貌;
2. [RV1126B/README.md](RV1126B/README.md) —— 下位机上手、引脚表、**已知硬约束**
   (第一次搭 SDK 环境?先看 [RV1126B/SETUP.md](RV1126B/SETUP.md));
3. [RV1126B/ARCHITECTURE.md](RV1126B/ARCHITECTURE.md) —— 模块地图、线程模型、对齐引擎原理、落盘格式;
4. [RV1126B/docs/PROTOCOL_V2.md](RV1126B/docs/PROTOCOL_V2.md) —— 字节级协议;
5. [RV1126B/AUTOSTART.md](RV1126B/AUTOSTART.md) —— 板子开机自启机制(踩过坑,已厘清)。

## 硬件版本

| 版本 | 状态 |
|---|---|
| V2 / V3 | 双摄硬同步已验证(cam0 主 / cam1 从,dpts 恒 −6µs) |
| **V4** | 新增千兆 ETH(RTL8211F)+ SD3.0(UHS-I);⚠ RJ45 四对镜像反接需改板;⚠ XHS 未引到 STM32(见 RV1126B/README.md) |
