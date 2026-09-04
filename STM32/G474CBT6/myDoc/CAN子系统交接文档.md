# CAN 子系统交接文档（数据手套主控 STM32G474）

> 更新：2026-08-20。对应代码：`Core/Src/canfd.c`、`Core/Src/mcp2518.c`、
> `Core/Src/glove_app.c`、`Core/Src/rv_link.c`、`Core/Inc/glove_protocol.h`。
> 读代码前先读本文；代码注释里有更细的寄存器级依据（标了手册章节号）。

---

## 0. 全局图景

```
                        STM32G474CBT6（本工程，唯一的"主"）
                          |
        +--------+--------+--------+-----------+-----------+
        | bus 0  | bus 1  | bus 2  |   bus 3   |   bus 4   |
        | FDCAN1 | FDCAN2 | FDCAN3 | MCP2518#1 | MCP2518#2 |
        | (原生) | (原生) | (原生) | 经 SPI2   | 经 SPI3   |
        +--------+--------+--------+-----------+-----------+
        每条总线：4 个关节节点(AT32+TMR3111) + 将来 1 个触觉节点
```

- **速率**：仲裁段 500kbps，数据段 2Mbps（FD + BRS）。2026-08 从 1M/5M 降速：5Mbps 实测在台架线束上不稳定。
- **角色纪律**：STM32 每帧广播一条 SYNC，节点**只应答、绝不主动发言**。
- **数据去向**：关节角度写进 SPI 上行帧的槽位 0..19，60Hz 被 RV1126B 读走。
- **模块分工**：
  - `canfd.c` —— 3 路原生 FDCAN（bus 0..2）
  - `mcp2518.c` —— 2 颗 MCP2518FD（bus 3..4），寄存器级 SPI 驱动
  - `glove_app.c` —— 唯一的接线处：初始化顺序 + XVS 分发 + 主循环调度
  - `rv_link.c` —— 双缓冲/生产者闸门/心跳（CAN 是它的"生产者"之一）

---

## 1. 三套编号（新人最容易混的地方，先背这张表）

| 编号 | 是什么 | 取值 | 谁用 |
|---|---|---|---|
| **CAN 帧 ID** | 线上传输的东西，每总线内唯一、**跨总线重复** | 0x101~0x104 | 节点固件、验收滤波 |
| **全局编号** | 给人看的（日志、贴标签） | bus×10+node：1..4, 11..14, 21..24, 31..34, 41..44 | 装配、排障 |
| **帧内槽位** | SPI 帧里 20 个连续槽的下标，紧凑无空洞 | bus×4+(node-1) → 0..19 | RV 解析、写帧 |

换算宏都在 `glove_protocol.h` §4b：`GLOVE_JOINT_SLOT(bus, node)`、
`GLOVE_JOINT_GLOBAL_ID(bus, node)`、`GLOVE_JOINT_NODE_FROM_ID(can_id)`。
**RV 侧只认槽位**，前两套是主控内部的事。

---

## 2. 一个 60Hz 周期的完整时间线（核心，务必看懂）

```
t=0        XVS 同步沿（V4 起：主手套自产 60Hz，TIM2 CH3 输出比较驱动 PB10
           开漏，**下降沿=同步沿**；从手套/相机跟随。V3 时代是相机产生、
           STM32 输入捕获，代码里从机模式仍保留）
           → TIM2 中断（NVIC 优先级 1），时间戳=CCR3（硬件精确）
           → FrameSync_Handle → GloveApp_OnFrameSync(ts)，顺序固定：
             ① Canfd_OnFrameSync  3 路原生：SYNC 压入 TX FIFO（~2µs/路）
             ② Mcp_OnFrameSync    2 颗 MCP："点火"——预装好的 SYNC
                                  只写 1 个 TXREQ 位（3 字节 SPI，~3µs/颗）
             ③ RvLink_OnFrameSync 冻结上周期缓冲、交换双缓冲、
                                  复位生产者闸门和心跳位图
             ④ IMU_OnFrameSync    置"该读 FIFO"标志
           （①② 必须在 ③ 前：SYNC 发出时刻 = 全系统采样基准，不能被推迟；
             节点最快 ~100µs 后才回帧，③ 的缓冲交换早已完成，无竞态）

t≈35µs     SYNC 帧上线：ID=0x001（最高仲裁优先级）、FD+BRS、DLC=1、data[0]=0x01
           五条总线偏斜 ~15µs，且恒定（可标定）

t≈0.1~2ms  节点收到 SYNC → 采样 TMR3111 → 回帧：
           ID=0x100+节点号、标准帧、FD+BRS、DLC=4、
           小端 uint32（data[0]=LSB），bit[22:0]=角度，bit[31:23]=预留

           ├─ 原生总线接收：帧过验收滤波（只放行 0x101~0x104）→ RX FIFO0
           │  → FDCANx_IT0 中断（优先级 0，全系统最高）
           │  → HAL_FDCAN_RxFifo0Callback：while 抽干 FIFO、合规校验、
           │    拼 uint32、算槽位、写 send 缓冲 + 标心跳
           │
           └─ MCP 总线接收：帧进芯片内 FIFO2（16 深）→ nINT 拉低
              → EXTI 中断（优先级 1）只置标志
              → 主循环 Mcp_Poll → drain_rx：SPI 读回对象、校验、同样写帧

t=2.5ms    回帧窗口关闭（CANFD_REPLY_WINDOW_US / MCP_REPLY_WINDOW_US，
           用 TIM2 的 1µs 时基计）：
           Canfd_Poll → RvLink_ProducerDone(RV_PRODUCER_CAN_NATIVE)
           Mcp_Poll   → RvLink_ProducerDone(RV_PRODUCER_CAN_MCP)
           没收齐也放行 —— 缺的节点由心跳位（=1）表达

t≈3~5ms    IMU 批量读完成 → ProducerDone(RV_PRODUCER_IMU)
           三个生产者齐了（或 8ms 超时）→ RvLink_Poll 封帧（CRC）
           → 装 SPI DMA → PA1 拉高 → RV 来读

t=16.7ms   下一个 XVS，循环往复
```

**关键语义（背下来）**：节点对第 N 条 SYNC 的回帧，采样时刻 = 第 N 个 XVS 边沿，
在语义上属于**刚结束的周期**，所以写进的是**刚冻结的 send 缓冲**
（`RvLink_GetSendBuffer()`），不是正在采集的 collect 缓冲。
帧封好（CRC 算完）之后才到的帧**必须丢弃**（计 `rx_late`）——
封帧后再写会让内容和 CRC 不一致，RV 整帧作废，损失更大。
写入前的 `RvLink_FramePending()` 检查在 ISR 里天然无竞态
（ISR 相对主循环原子执行）。

---

## 3. canfd.c —— 原生 3 路的实现要点

### 初始化（`Canfd_Init` → 每路 `bus_init`）
1. **代码里覆盖 `StdFiltersNbr=1` 并重新 `HAL_FDCAN_Init`**——.ioc 里是 0，
   在代码里改是有意为之（和滤波器配置强耦合，放同一文件不易改漏），
   **不要去 CubeMX 里改**。
2. 验收滤波：range 0x101~0x104 → FIFO0；`ConfigGlobalFilter` 全拒收非匹配帧
   （G4 的 RX FIFO 硬件只有 **3 深**，一个位置都不给无关帧）。
3. **TDC 发送延迟补偿**（offset=34 mtq = 数据位采样点 425ns）：2Mbps 下不像
   5Mbps 那样不开必炸，但保持正确配置。CubeMX 没有这个选项，只能代码里配，
   必须在 Start 前。
4. **中断分线**：收帧走 IT0（优先级 1），错误全挪 IT1（优先级 2）。
   2026-09-02 起 TIM2（XVS 10µs 脉宽整形）占优先级 0；CAN 收帧 1 仍然
   足够：FIFO 3 深 = ~260µs 容忍度，TIM2 的 ISR 只占 ~30µs；
   错误风暴（如台面无节点时 SYNC 无人 ACK）永远挤不占收帧。
5. `HAL_FDCAN_Start`。单路失败不拖累其他路（`started=0`，后续全跳过）。

### 发送
SYNC 用 `AddMessageToTxFifoQ`，**自动重发是关的**（.ioc `AutoRetransmission=DISABLE`）
——过期的 SYNC 比没有更糟，这是刻意语义，别"修"它。

### 接收（`HAL_FDCAN_RxFifo0Callback`，优先级 0 中断）
- `while` 抽干 FIFO（中断挂起期间可能又进了帧）；
- 合规校验（帧型/DLC=4/ID 精确范围）不合规计 `rx_bad`；
- 4 字节小帧**直接在 ISR 解析写帧**，不走"搬运+主循环解析"两段式
  （拼一个 uint32 比入队出队还便宜；将来触觉 64 字节大帧再考虑队列）；
- `MESSAGE_LOST` 位 → `rx_lost`（一涨就是中断被挡或节点回帧撞车）。

### bus-off 自恢复
`ErrorStatusCallback` 里清 `CCCR.INIT` 发起标准恢复；`Canfd_Poll` 里还有
第二道保险（查协议状态兜底回调丢失）。

---

## 4. mcp2518.c —— SPI 转 CAN 2 路的实现要点

### 硬件
| | 芯片 0（bus 3） | 芯片 1（bus 4） |
|---|---|---|
| SPI | SPI2（PB13/14/15） | SPI3（PB3/4/5） |
| CS | PA9 | PB7 |
| nINT | PA10（EXTI10，下降沿） | PA3（EXTI3，下降沿） |

每颗独立 40MHz 无源晶体，**不用内部 PLL**。
**SPI 时钟上限 17MHz**（手册：FSCK ≤ 0.85×SYSCLK/2），当前配 10MHz。

### 为什么不用 HAL_SPI 而是寄存器级轮询（重要，别改回去）
1. XVS 中断（优先级 1）里要发点火事务，而 SysTick 优先级最低，在那里
   **tick 不走**，HAL 的超时机制失效，SPI 异常会死等。寄存器级用有界自旋
   计数（`MCP_SPI_SPIN_MAX`），任何上下文行为确定。
2. 主循环读回帧和 ISR 点火共用一个 SPI 外设，HAL 的锁会让点火吃 BUSY。
   这里用 `s_spi_busy[]` 显式协调：冲突时点火**放弃**并计 `sync_tx_fail`
   ——宁可丢一拍同步，不在 ISR 里等。

### "预装载 + 点火"机制
- 空闲期（`Mcp_Poll`）：SYNC 报文整个写进芯片内 FIFO1 + UINC（入队不发）；
- XVS 中断：只写 1 个 TXREQ 位（3 字节 SPI ≈ 3µs）。
把 XVS 中断里的 SPI 开销压到最小，SYNC 偏斜恒定可标定。
发完后 `Mcp_Poll` 检查 FIFO1 空了再重新预装载。

### 芯片内资源（2KB 消息 RAM）
- FIFO1 = TX，1 深（SYNC 专用），TXAT=00 不重发（同原生语义）；
- FIFO2 = RX，16 深（比 G4 的 3 深宽裕得多，所以 nINT 不用抢优先级）；
- 滤波器 0：掩码 0x7F8 放行 0x100~0x107（MCP 是掩码式凑不出精确范围，
  多出的 ID 由软件校验挡住）。

### 初始化 12 步（`chip_init`，每步失败即返回）
RESET → **SPI 通信自检**（写读 0x12345678，不对=线/CS/极性问题，直接停）
→ 等 OSCRDY → 开 ECC → **2KB RAM 清零**（不清会触发 ECC 错误）
→ CiCON（ISO CRC / 协议异常关 / 重发策略，逐项和 G4 对齐）
→ 位时序（500k=80tq@87.5%，2M=20tq@85%，与 G4 完全一致）→ TDC 自动模式
→ FIFO → 滤波器 → **IOCON（GPIO0 驱低 = 收发器退出待机）**
→ 中断使能 → 预装载第一条 SYNC → **回读校验位时序** → 切 Normal FD 并确认。

### 收发器待机（STBY）
MCP 的 GPIO0 → MCP2542FD 的 STBY 脚（高=待机）。初始化第 8 步已拉低，
**上电即工作**，平时不用管。`Mcp_SetXcvrStandby()` 留作将来功耗管理。
注意：IOCON 有勘误，**只能 32 位整字写**，字节写会破坏 LAT 位。

---

## 5. 时序契约 —— 节点固件必须遵守的规矩

| 项 | 要求 |
|---|---|
| 位时序 | 仲裁 500kbps / 数据 2Mbps；主控 G4 与 MCP 均为采样点 87.5%/85%（tq 都是 25ns）；节点对齐到该区间 |
| 时钟源 | **必须晶振**（2Mbps 下内部 RC 依然不可靠） |
| TDC | 节点侧也必须开 |
| 发言纪律 | 只在收到 SYNC 后回帧，绝不主动发 |
| 回帧时限 | SYNC 后 **2ms 内**（窗口 2.5ms 关闭，晚到=丢弃） |
| 帧格式 | ID=0x100+节点号、标准帧、FD+BRS、DLC=4、小端、bit[31:23] 填 0 |
| **自动重发** | **数据回帧必须开**！多节点同时回帧靠仲裁排队，仲裁输了必须重试。one-shot 只允许用在主控的 SYNC 上。（2026-08 实测教训：节点 one-shot + 仲裁失败 = ID 大的节点几乎每帧静默丢失） |
| 错峰（建议） | 收到 SYNC 后延迟 (节点号-1)×150µs 再发，消除碰撞、摊平 FIFO 压力 |

---

## 6. 可观测性 —— 每种故障都有专属计数器

Live Expressions 挂 `g_canfd`（原生）和 `g_mcp`（MCP）。

### 丢包定位决策树
| 现象 | 结论 |
|---|---|
| `rx_node[i]` 比例失衡（应各 60/s） | 帧没到主控：节点仲裁+one-shot、或节点没发 |
| `rx_late` 涨 | 节点回帧慢于 2.5ms 窗口 |
| `rx_lost` 涨 | G4 硬件 FIFO 溢出（查中断优先级/节点撞车） |
| `rx_bad` 涨 | 节点帧格式错（DLC/帧型/ID） |
| `err_irq` 持续涨 | 物理层（端接/共地/线长）或位时序不匹配 |
| `sync_tx_fail`(MCP) 涨 | 上条 SYNC 没发出去（总线断/无 ACK）或 SPI 被占 |
| `spi_err`(MCP) 非 0 | SPI 线/CS 问题 |

### 无节点上电的**正常**现象（别当故障修）
SYNC 无人 ACK → 错误计数爬到 error passive 后稳住：
原生 `err_irq` 涨几下停；MCP `trec` 低 8 位（TEC）≈128；
MCP `sync_tx_fail` 持续涨（上条没发完不重装）。接上第一个节点全部消停。

---

## 7. 已知待办 / 边界

1. **触觉节点协议未定**（CAN ID、512 字节怎么分帧）。硬件路由已预留：
   原生走 RX FIFO1、MCP 的掩码已放行到 0x107；定了协议在两个模块各加
   一段接收分支即可。
2. 管理帧（HELLO / SET_ID / ACK / ANNOUNCE，节点 ID 分配）未做。
3. CAN bootloader（节点在线升级）未做。
4. SYNC 的 data[0] 恒为 0x01；将来可扩展为携带周期号低位（需同步改节点）。
5. MCP 的 GPIO1 配成了输入未驱动 —— 板上如果接了东西需要确认。

---

## 8. 十分钟上手路径（给新人的阅读顺序）

1. 本文档 §1（三套编号）和 §2（时间线）；
2. `glove_protocol.h` —— 协议唯一真相源，含编译期断言；
3. `glove_app.c` —— 不到 150 行，所有模块怎么串起来的都在这；
4. `canfd.c` 从 `HAL_FDCAN_RxFifo0Callback` 往回读；
5. `mcp2518.c` 从 `Mcp_OnFrameSync`（点火）和 `drain_rx` 往回读；
6. 挂 Live Expressions 跑一遍 §6 的表。
