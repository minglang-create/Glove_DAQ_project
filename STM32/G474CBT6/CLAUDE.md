# 数据手套 DAQ 项目 — 交接/背景文档

> **用途**：本文件是从旧工程（Glove_DAQ_G473_ADC）迁移到新工程（STM32G474CBT6）的项目背景交接。
> 到新工程后，请把本文件复制到**新工程根目录并改名 `CLAUDE.md`**，Claude Code 每次会话会自动加载，
> 等于把整个项目的架构决策和工作方式一次性交给新目录的助手。
> 完整的元器件手册、烧录流程、原理图网表都在 `myDoc/`（请连同本目录一起复制到新工程）。

---

## 0. 与助手协作的约定（重要）

- **需要芯片手册/规格书时，先 `ls myDoc/` 看用户是否已放好**（用户经常提前放好而不说，别自己 WebFetch/curl 抓网页）；没有就开清单让用户下载放进 `myDoc/`。
- 本机已装 **poppler**：`pdftotext`/`pdftoppm` 在
  `C:\Users\jiaqi\AppData\Local\Microsoft\WinGet\Packages\oschwartz10612.Poppler_*\Library\bin`
  （不在 Read 工具 PATH 里，需用完整路径调用；用 `pdftotext -layout xxx.pdf out.txt` 解析手册）。
- 任何不确定的信息**问用户去确认**，不要臆测或上网找。

---

## 1. 系统总览

**目标**：数据采集手套。上位机（也是自研，RV1126B）通过 2 根固定 Type-C 各连一只手套，
一个上位机带 2 只手套，汇总对齐后送 PC。每只手套内：
- RV1126B（MYZR-RV1126B-LB221 核心板模组，吃 5V 出 3.3V/1.8V）跑 2 路硬件同步相机（H.265）；
- STM32G474CBT6 做传感器采集主控，SPI 从机被 RV 随时读取；
- 分布式 CAN FD 节点采集 21 路关节角度（TMR3111）+ 6 路触觉（16×16）+ 主控板 IMU。

**同步脊柱**（2026-08 V4 起反转）：**主手套 STM32 晶振是全系统时间基准**，
PB10 开漏输出自产 60Hz XVS（下降沿=同步沿），相机与从手套跟随；
再经 5 条 CAN SYNC 广播到节点。V3 时代是相机产生 XVS、STM32 捕获（代码留有从机模式）。
双手套互连的主从由按键协商（未实现，当前默认主机）。
60Hz 一帧，固定一帧延迟流水线，所有传感器时间戳可换算到同一时间轴。

**当前阶段（2026-07-20）**：主控板 AIO 原理图 V2.1 审查已过（见 §5），准备改 R1→39k 后画 PCB / 进固件。

---

## 2. 主控板 STM32G474CBT6（LQFP48, 128KB）

- **5 路 CAN FD** = 3 原生 FDCAN + 2×(MCP2518FD 经 SPI)。收发器全部 **MCP2542FD**
  （VIO 脚 1.7~5.5V 接 3.3V；VDD 4.5~5.5V 是 **5V 器件**，接 5V_SYS）。
  弃用过 TCAN3414/TCAN4550/ATA6563。
- **供电链**：Type-C PD（CH224Q，CFG1=56k 请求 15V，PG 悬空不用）→ **AP64500 buck → 5.2V**
  （FB=82.5k/15k，COMP R5=15.8k+C5=2.7nF+C6=39pF）。5V 输入时 buck 进直通模式输出~4.8-4.95V
  （降级/维护模式，满载长链不保证）；15V PD 时才稳 5.2V 全裕量。
  供电等级检测 = VBUS 分压 → PA0 ADC（**R1 待改 56k→39k**，原比例在 15V 太贴 VDDA）。
- **引脚**（最终以新工程 CubeMX .ioc 为准）：
  FDCAN1=PA11/PA12（避开 PB8=BOOT0），FDCAN2=PB12/PB6，FDCAN3=PA8/PA15；
  SPI1 从机(→RV)=PA4/5/6/7，SPI2(→MCP2518 #1)=PB13/14/15+CS PA9，SPI3(→MCP2518 #2)=PB3/4/5+CS PB7；
  MCP INT=PA10/PA3；I2C1=**PB8(BOOT0)/PB9**（温度+IMU）；IMU FSYNC=PC13；
  XVS=PB10（V4 起为开漏**输出**，主手套自产；V3 为输入）；SBU 同步入=PB11；V4 已移除 STM32 双色 LED（原 PB1/PB2 引到排针，其一与 XVS 网络相连，务必保持输入态）；
  握手 GPIO=PA1；SWD=PA13/PA14。
- **⚠️ BOOT0 纪律**：PB8=BOOT0 用作 I2C1，必须写 option bytes（nSWBOOT0=0,nBOOT0=1），
  **首次烧录必须同时写 OB**。完整流程 + 固件自愈代码见 `myDoc/G474_BOOT0_烧录流程.md`。
- MCP2518FD：每颗独立 40MHz **无源晶体**（CL=12pF + 2×15pF，不用 PLL）；GPIO0 配 XSTBY 控收发器待机。

---

## 3. IMU 子系统（主控板 I2C1）

- **ICM-45686**（0x68）+ AUX I2C 挂 **MMC5603NJ 磁力计**（4.7k 上拉）+ **TMP1075**（0x48，板温遥测，非 IMU 温补）。
- I2C1 全域 3.3V，上拉 2.2k。ODR 定 **400Hz LN 模式** + FIFO 批读，60Hz 每帧批量取走全部样本逐个积分。
- **FSYNC**：STM32 GPIO(PC13) 发~100µs 脉冲打标记，芯片记录脉冲↔采样的时间差(TMST_FSYNC)，
  用于把姿态插值到相机曝光时刻。数据集帧里建议同时上报原始 IMU 样本（保值）。
- 陀螺温补用 ICM 片内温度（非 TMP1075）。上电初始姿态：静止检测→估陀螺零偏→加速度定倾角→yaw=0（视觉锚定，不依赖磁罗盘）。

---

## 4. CAN 节点（自研小板，独立工程）

- 每节点 = AT32（型号待定，需带 CAN FD）+ MCP2542FD + TMR3111（SPI 读 23bit 角度）+ LDO(5V→3.3V)。
  触觉也是自研 CAN 节点。按手指菊花链，每总线 4~5 节点，**仲裁 500kbps / 数据 2Mbps**（2026-08 定：5M 台架实测不稳降速，8M 更早已否决），分裂端接。
- **协议**：主控收 XVS→5 总线广播 SYNC（ID 最小/最高优先级）→节点同步采样回帧。
- **节点 ID**：装配时断电逐个插入、上电分配（FH64MA 是 ZIF 不能热插拔），ID 存节点 flash；
  主控存花名册+自动补位+ANNOUNCE 撞 ID 防御；管理帧 HELLO/SET_ID/ACK/ANNOUNCE。
- **CAN bootloader**：节点 flash 头部 8~16KB + app CRC + 配置页，不拆手套走 CAN 升级。
- **线束**：FH64MA-7S-0.25SHW（0.25mm 上接 ZIF，0.2A/pin，寿命仅 10 次），
  7pin=GND/CANH/CANL/GND/5V/5V/GND。节点收发器直接吃线束 5V（不过 LDO），链尾电压硬下限 ≥4.5V。

---

## 5. AIO 主板原理图 V2.1 审查结论（netlist 在 myDoc/）

审查已过。关键确认：RV↔STM32 SPI1 = RV 的 **SPI0_M2**（球 J16=CLK/P5=MOSI/T3=MISO/P6=CSN0，RV 主 STM32 从，两端 3.3V）；
XVS/SBU/握手在 RV **VCCIO5=3.3V** 域、摄像头 VCCIO6=3.3V（已确认）；1.8V 启动脚(B2/L5)正确拉到 VCC_1V8。

- **待改**：VBUS 采样分压 **R1 56k→39k**。
- **建议未做（用户可选）**：MCP nCS 加 10k 上拉；NRST 引测试点；网名 VBAT→VBUS_PD（其实是 5~15V PD 总线）；VCC5V0_SYS 实为 5.2V。
- **用户已否决**：CAN 改分裂端接、FPC 排布重排（暂不改，以后有问题再说）。

---

## 6. myDoc/ 关键文件索引

- `G474_BOOT0_烧录流程.md` — BOOT0/option-byte 首次烧录流程 + 量产日志通道备案（PA14→UART / SPI 转发）
- `Netlist_AIO_SCHV2_1_*.txt` — 主板原理图网表（Protel 格式）
- `MYZR-RV1126B-LB221-REVA-OPEN.pdf` — RV1126B 核心板模组原理图（VCCIO 域/引脚定义权威来源）
- 各芯片手册：MCP2518FDT-E, MCP2542FDT-E, ICM-45686, MMC5603NJ, TMP1075, CH224Q, AP64500SP-13,
  NCP161(节点 LDO), FH64MA(连接器), SN74LV1T34(电平转换), RV1126B_datashell, LED A-SP1922B 等
- `SCM1611_protocol.md` / `TMR3111_PWM_spec.md` — 旧传感器协议（TMR3111 现改 SPI 读）

## 7. 迁移说明
- 本项目对话历史 + 项目记忆绑定在旧文件夹路径，新工程看不到。旧文件夹**保留**可随时 `--resume` 翻历史（完整推理过程）。
- 新工程首次会话：确认本文件已作为 `CLAUDE.md` 放在根目录、`myDoc/` 已复制过来即可。
