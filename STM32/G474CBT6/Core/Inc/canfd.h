/**
  ******************************************************************************
  * @file    canfd.h
  * @brief   3 路原生 FDCAN：XVS 同步广播 + 关节节点数据接收
  *
  * ============================================================================
  * 总线拓扑与编号（见 glove_protocol.h §4b 的三套编号说明）
  * ============================================================================
  *   本模块管 bus 0..2 = FDCAN1 / FDCAN2 / FDCAN3（原生外设）。
  *   bus 3..4 = 2 颗 MCP2518FD（SPI2/SPI3），在第 4 步的独立模块里做。
  *
  *   每条总线上：4 个关节节点（CAN ID 0x101..0x104）+ 将来 1 个触觉节点。
  *   帧内槽位由 GLOVE_JOINT_SLOT() 算。拓扑由 glove_protocol.h 的
  *   GLOVE_SENSOR_MODE 编译期选择：0=正式手套 5x4 / 1=转接板 7+7+6 /
  *   2=碳膜 ADC2CAN（3 路原生总线，21 路 12bit ADC，回帧 0x201..0x203）。
  *
  * ============================================================================
  * 每周期的时序
  * ============================================================================
  *   XVS 上升沿（TIM2 捕获中断，优先级 1）：
  *     1. 本模块最先执行：向 3 条总线的 TX FIFO 压入 SYNC 帧（ID=0x001，
  *        最高优先级，总线此刻空闲，立即上线）。SYNC 的发出时刻就是全系统
  *        的采样基准，所以它排在 XVS 分发链的第一位。
  *     2. rv_link 冻结上一周期缓冲、复位生产者闸门。
  *   之后 ~50us..2ms：节点收到 SYNC，采样并回帧（FD+BRS，DLC=4）。
  *   接收中断（优先级 0）把角度写进**刚冻结的 send 缓冲** —— 这批数据在语义上
  *   属于刚结束的周期（采样时刻 = 本次 XVS 边沿）。
  *   主循环 Canfd_Poll()：回帧窗口（CANFD_REPLY_WINDOW_US）结束后调
  *   RvLink_ProducerDone()，放行封帧。
  *
  * ============================================================================
  * 与 .ioc 的关系
  * ============================================================================
  *   位时序（仲裁 500kbps / 数据 2Mbps @ 80MHz 内核时钟，Prescaler=2 → tq=25ns，
  *   采样点 87.5% / 85%）由 CubeMX 生成。2026-08 从 1M/5M 降速：5Mbps 实测
  *   在台架线束上不稳定。
  *   本模块在 Canfd_Init() 里补上 CubeMX 给不了的部分：
  *     - StdFiltersNbr 覆盖 + 重新 HAL_FDCAN_Init（.ioc 里是 0，改代码比改
  *       CubeMX 少一次生成周期，且重新生成代码也不会丢）
  *     - 验收滤波器 + 全局滤波（非匹配帧全拒收）
  *     - TDC 发送延迟补偿（CubeMX 没有这个选项）
  *     - 中断分线：收帧走 IT0（优先级 0），错误走 IT1（优先级 2）
  ******************************************************************************
  */

#ifndef CANFD_H
#define CANFD_H

#include <stdint.h>
#include "glove_protocol.h"

/* ==========================================================================
 * 1. 配置
 * ========================================================================== */

/** 原生 FDCAN 总线数 */
#define CANFD_BUS_CNT               3u

/** SYNC 广播帧：ID 最小 = 仲裁优先级最高。内容按协议定死。 */
#define CANFD_SYNC_ID               0x001u
#define CANFD_SYNC_DATA0            0x01u

/**
 * 回帧收集窗口（us，用 TIM2 的 1us 时基量）。
 * SYNC 发出后节点应在 1~2ms 内回帧；窗口关闭时不管收没收齐都放行封帧
 * （缺的节点由心跳位表达）。窗口必须 < 闸门超时 8ms，
 * 也不必卡太紧 —— 反正封帧还要等 IMU 的 3~5ms 批量读。
 */
#define CANFD_REPLY_WINDOW_US       2500u

/**
 * TDC 发送延迟补偿。2Mbps 位时间 500ns，收发器环路延迟 100~200ns 已不像
 * 5Mbps 时那样致命，但开着没有坏处，保持正确配置。
 * TdcOffset 单位 = mtq（FDCAN 内核时钟周期 12.5ns），取数据位采样点位置：
 *   (1 + DataTimeSeg1) * DataPrescaler = (1 + 16) * 2 = 34（= 425ns）
 * 实测误码再微调。TdcFilter = 0（默认关窗口滤波）。
 */
#define CANFD_TDC_OFFSET            34u
#define CANFD_TDC_FILTER            0u

/* ==========================================================================
 * 2. 统计（Live Expressions 看 g_canfd）
 * ========================================================================== */

/** 单条总线的统计 */
typedef struct
{
  uint8_t  started;          /**< 1 = 这条总线初始化并启动成功                 */
  uint32_t sync_tx;          /**< 成功压入 TX FIFO 的 SYNC 帧数                */
  uint32_t sync_tx_fail;     /**< 压入失败次数（TX FIFO 满 = 上帧没发出去）    */
  uint32_t rx_joint;         /**< 收到的合法关节帧总数                         */
  uint32_t rx_bad;           /**< ID/DLC/帧型不合规而丢弃的帧数                */
  uint32_t rx_late;          /**< 窗口已关（帧已封）才到、被丢弃的帧数         */
  uint32_t rx_lost;          /**< 硬件 RX FIFO 溢出丢帧数（RF0L/RF1L）         */
  uint32_t err_irq;          /**< 错误中断（IT1）触发次数                      */
  uint32_t busoff;           /**< 进入 bus-off 的次数（已自动发起恢复）        */
  uint32_t rx_node[GLOVE_JOINTS_MAX_PER_BUS];        /**< 按节点分的合法帧计数 ——
                                                      两个节点应该同速增长（各 60/s），
                                                      比例失衡直接暴露"谁在丢"；
                                                      碳膜模式一帧带 7 路，
                                                      7 个计数同步增长 */
  uint8_t  a2c_last_seq;     /**< 碳膜模式：最近一帧的 frame_seq                */
  uint32_t a2c_seq_gap;      /**< 碳膜模式：seq 不连续（板侧丢帧）计数           */
  uint32_t a2c_group_mismatch; /**< 碳膜模式：group 字节与总线不符（接反，已按
                                    group 重映射，数据不丢，只是提示接线）      */
  uint32_t last_joint_raw[GLOVE_JOINTS_MAX_PER_BUS]; /**< 各节点最近一个原始值（体检用） */
} Canfd_BusStats_t;

typedef struct
{
  Canfd_BusStats_t bus[CANFD_BUS_CNT];
  uint32_t         cycles;         /**< 广播过 SYNC 的周期数                   */
  uint32_t         window_open;    /**< 1 = 当前处于回帧收集窗口内             */
} Canfd_Stats_t;

extern Canfd_Stats_t g_canfd;

/* ==========================================================================
 * 3. 接口
 * ========================================================================== */

/**
 * @brief  初始化并启动 3 路 FDCAN。
 * @return 启动成功的总线位掩码（bit0=FDCAN1 … bit2=FDCAN3）；0 = 全失败
 * @note   返回非 0 才把 RV_PRODUCER_CAN_NATIVE 登记进闸门。
 *         单条总线失败不影响其他总线（its started 标志=0，后续所有操作跳过它）。
 */
uint8_t Canfd_Init(void);

/**
 * @brief  XVS 到来时调用（TIM2 捕获中断，优先级 1），必须排在分发链第一位。
 * @param  ts  TIM2 CCR3 硬件时间戳（1us）
 * @note   只做：3 次 HAL_FDCAN_AddMessageToTxFifoQ（每次约 1~2us，写消息 RAM
 *         加置 TXBAR 位，硬件随后自动发送）+ 记录窗口起点。不等发送完成。
 */
void Canfd_OnFrameSync(uint32_t ts);

/**
 * @brief  主循环反复调用：回帧窗口关闭后放行闸门；顺带做 bus-off 恢复检查。
 */
void Canfd_Poll(void);

#endif /* CANFD_H */
