/**
  ******************************************************************************
  * @file    mcp2518.h
  * @brief   2 颗 MCP2518FD（SPI 转 CAN FD）：bus 3..4 的同步广播 + 关节接收
  *          + CAN 收发器待机控制
  *
  * ============================================================================
  * 硬件连接
  * ============================================================================
  *   芯片 0（bus 3）：SPI2（PB13/14/15），CS = PA9，nINT = PA10（EXTI10）
  *   芯片 1（bus 4）：SPI3（PB3/4/5），  CS = PB7，nINT = PA3（EXTI3）
  *   每颗独立 40MHz 无源晶体，不用内部 PLL（SYSCLK = 40MHz）。
  *   GPIO0 接本总线 MCP2542FD 收发器的 STBY 脚（高 = 待机，低 = 工作）。
  *
  *   ⚠ SPI 时钟上限：手册 §7（表 7-3 注 3）FSCK ≤ 0.85 × (SYSCLK/2) = 17MHz。
  *     SPI2/3 必须配 10MHz（160MHz / 16），20MHz 超标。
  *
  *   ⚠ nINT 是**低有效**（INTOD=0 推挽输出），EXTI 必须配**下降沿**触发。
  *
  * ============================================================================
  * 每周期的时序（和原生 FDCAN 的 canfd.c 呼应）
  * ============================================================================
  *   空闲期（上一周期读完之后）：把下一条 SYNC 报文预写进 FIFO1 并置 UINC
  *     —— 报文已入队但未请求发送。
  *   XVS 中断：只写一个字节把 TXREQ 置位（3 字节 SPI 事务 ≈ 3us/颗），
  *     报文立即上线。这就是"预装载 + 点火"：把 XVS 中断里的 SPI 开销压到最小，
  *     两颗 MCP 与三路原生 FDCAN 的 SYNC 偏斜控制在 ~15us 内且基本恒定。
  *   节点回帧 → MCP 收进 FIFO2 → nINT 拉低 → EXTI 置标志 →
  *     主循环 Mcp_Poll() 经 SPI 把收到的对象读回、写进 send 缓冲。
  *   回帧窗口关闭 → RvLink_ProducerDone(RV_PRODUCER_CAN_MCP)。
  *
  * ============================================================================
  * 为什么不用 HAL_SPI_*，而是寄存器级轮询
  * ============================================================================
  *   1. XVS 中断（优先级 1）里要发"点火"事务。HAL 的超时机制靠 HAL_GetTick()，
  *      而 SysTick 优先级最低，在优先级 1 的 ISR 里 tick 不走 —— SPI 一旦异常
  *      HAL 会死等永远出不来。寄存器级实现用有界循环计数做超时，任何上下文
  *      行为都确定。
  *   2. 避开 HAL 的锁：主循环批量读和 ISR 点火共用一个 SPI 外设，HAL 的
  *      __HAL_LOCK 会让点火直接吃 HAL_BUSY。这里用自己的 busy 标志显式协调
  *      （ISR 相对主循环是原子的，标志检查无竞态）。
  *   3. 事务都很短（3~20 字节），轮询比 DMA/中断的搭建开销更低。
  *
  * ============================================================================
  * 芯片内资源规划（2KB 消息 RAM，0x400~0xBFF）
  * ============================================================================
  *   TEF/TXQ 不用（CiCON.STEF=0, TXQEN=0）
  *   FIFO1 = TX，1 深 × (8B 头 + 8B 载荷) = 16B   @ 0x400   （SYNC 专用）
  *   FIFO2 = RX，16 深 × (8B 头 + 8B 载荷) = 256B @ 0x410   （关节回帧）
  *   滤波器 0：SID 0x100~0x107（掩码 0x7F8）→ FIFO2，其余帧不收
  ******************************************************************************
  */

#ifndef MCP2518_H
#define MCP2518_H

#include <stdint.h>
#include "glove_protocol.h"

/* ==========================================================================
 * 1. 配置
 * ========================================================================== */

#define MCP_CHIP_CNT                2u

/** 芯片 → 全局总线序号：芯片 0 = bus 3，芯片 1 = bus 4（bus 0..2 是原生 FDCAN） */
#define MCP_BUS_BASE                3u

/** 回帧收集窗口（us），与 canfd.h 的语义一致 */
#define MCP_REPLY_WINDOW_US         2500u

/** SPI 单字节轮询的自旋上限。10MHz 下一个字节 0.8us（约 130 个 CPU 周期），
    给 4000 次自旋 ≈ 25us 都等不到就是硬件出问题了，放弃本事务并计 spi_err。 */
#define MCP_SPI_SPIN_MAX            4000u

/* ==========================================================================
 * 2. 统计（Live Expressions 看 g_mcp）
 * ========================================================================== */

typedef struct
{
  uint8_t  started;         /**< 1 = 初始化成功（含 SPI 通信自检和模式切换）    */
  uint32_t sync_tx;         /**< 点火成功次数                                   */
  uint32_t sync_tx_fail;    /**< 点火失败（SPI 忙 / 上一条还没发出去）          */
  uint32_t restage;         /**< 成功预装载 SYNC 的次数                         */
  uint32_t rx_joint;        /**< 收到的合法关节帧数                             */
  uint32_t rx_bad;          /**< 帧型/DLC/ID 不合规而丢弃                       */
  uint32_t rx_late;         /**< 帧已封才到、被丢弃                             */
  uint32_t rx_ovf;          /**< FIFO2 溢出（RXOVIF）次数                       */
  uint32_t spi_err;         /**< SPI 事务超时/失败次数                          */
  uint32_t trec;            /**< 最近一次读到的 CiTREC（错误计数器，低 16 位：
                                 [15:8]=REC 接收错误计数，[7:0]=TEC 发送错误计数）*/
  uint32_t last_joint_raw[GLOVE_JOINTS_MAX_PER_BUS]; /**< 各节点最近原始值（体检用） */
  uint32_t rx_node[GLOVE_JOINTS_MAX_PER_BUS];        /**< 按节点分的合法帧计数 ——
                                 与 canfd.h 的同名字段语义一致，自检的 CAN 挂载
                                 检测（glove_fsm 的 build_selftest_bitmap）靠它
                                 判断每个节点是否在线                            */
} Mcp_ChipStats_t;

typedef struct
{
  Mcp_ChipStats_t chip[MCP_CHIP_CNT];
  uint32_t        cycles;
} Mcp_Stats_t;

extern Mcp_Stats_t g_mcp;

/* ==========================================================================
 * 3. 接口
 * ========================================================================== */

/**
 * @brief  初始化两颗 MCP2518FD。阻塞式，约 5ms/颗（大头是 2KB RAM 清零）。
 * @return 成功芯片的位掩码（bit0 = 芯片 0）；0 = 全失败
 * @note   初始化序列（每一步的依据见 mcp2518.c）：
 *           RESET → SPI 通信自检（写读寄存器）→ 等 OSCRDY → 使能 ECC →
 *           2KB 消息 RAM 清零（ECC 要求）→ CiCON/位时序/TDC → FIFO/滤波器 →
 *           IOCON（**顺带把收发器 STBY 拉低 = 工作**）→ 中断使能 →
 *           预装载第一条 SYNC → 关键寄存器回读校验 → 切 Normal FD 模式
 * @note   单颗失败不影响另一颗。
 */
uint8_t Mcp_Init(void);

/**
 * @brief  收发器待机控制（经 MCP 的 GPIO0 → MCP2542FD 的 STBY 脚）。
 * @param  chip     0 或 1
 * @param  standby  1 = 收发器进待机（STBY 拉高），0 = 正常工作（拉低）
 * @note   Mcp_Init() 里已调用过 standby=0，上电即工作 —— 按当前调试需求，
 *         这个函数平时不需要再碰；留接口是为将来可能的功耗管理。
 * @note   整字写 IOCON：MCP2518FD 勘误规定对 IOCON 做单字节写会破坏 LAT 位，
 *         必须 32 位整字写。
 */
void Mcp_SetXcvrStandby(uint8_t chip, uint8_t standby);

/**
 * @brief  XVS 到来时调用（TIM2 捕获中断，优先级 1），排在 Canfd_OnFrameSync 之后。
 * @param  ts  TIM2 时间戳（1us）
 * @note   每颗芯片一个 3 字节 SPI 事务置 TXREQ（约 3us/颗，寄存器级轮询）。
 *         上一条 SYNC 还没发出去（预装载标志没恢复）或 SPI 正被主循环占用时
 *         跳过并计 sync_tx_fail —— 宁可丢一拍同步也不能在 ISR 里等。
 */
void Mcp_OnFrameSync(uint32_t ts);

/**
 * @brief  主循环反复调用：读回帧、写 send 缓冲、窗口关闭放行闸门、重新预装载 SYNC。
 */
void Mcp_Poll(void);

#endif /* MCP2518_H */
