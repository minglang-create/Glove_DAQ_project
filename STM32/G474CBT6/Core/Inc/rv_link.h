/**
  ******************************************************************************
  * @file    rv_link.h
  * @brief   SPI1 从机链路：双缓冲 + PA1 握手 + DMA 装载 + 异常恢复
  *
  *          这是 STM32 唯一的数据出口，也是台面调试时唯一的观测窗口，
  *          所以第一个打通它（用假数据），后面每加一个传感器都能立刻看见结果。
  ******************************************************************************
  */

#ifndef RV_LINK_H
#define RV_LINK_H

#include <stdint.h>
#include "glove_protocol.h"

/* ==========================================================================
 * 配置
 * ========================================================================== */

/**
 * 自检模式开关。
 *
 * 1 = 每帧用 GloveFrame_FillFake() 造假数据，**覆盖掉全部真实数据**。
 *     用途：系统自检 / RV 侧解析器回归测试 / 排查"到底是采集坏了还是链路坏了"。
 *     假数据的规律见 glove_frame.h 的 GloveFrame_FillFake() 注释，
 *     其中触觉阵列每个字等于自己的绝对字偏移，能一眼查出错位。
 * 0 = 正常工作：只发真实采集到的数据，缺的节点填 invalid + 心跳位置 1。
 *
 * 注意：置 1 时假数据是在生产者闸门之后、算 CRC 之前覆盖的，
 *       所以 IMU / CAN 写进去的真值会被盖掉，这是故意的。
 */
#define RV_LINK_FAKE_DATA           0

/** 握手引脚：STM32 输出给 RV1126B，高 = 本周期数据已就绪可读 */
#define RV_READY_GPIO_Port          GPIOA
#define RV_READY_Pin                GPIO_PIN_1

/* ==========================================================================
 * 运行统计（用 CubeIDE 的 Live Expressions 直接看 g_rv_stats）
 * ========================================================================== */

typedef struct
{
  uint32_t frames_armed;        /**< 成功装载进 DMA 并拉高 PA1 的帧数            */
  uint32_t frames_read;         /**< RV 完整读走的帧数（TxRxCplt 回调次数）      */
  uint32_t incomplete_read;     /**< 到下一个 XVS 时上一帧还没被读完的次数        */
  uint32_t tx_words_left;       /**< 最近一次不完整读取时 TX DMA 还剩多少字未发   */
  uint32_t spi_hard_reset;      /**< 因不完整读取而复位 SPI1 外设的次数          */
  uint32_t arm_fail;            /**< HAL_SPI_TransmitReceive_DMA 返回非 OK 的次数 */
  uint32_t spi_error;           /**< HAL_SPI_ErrorCallback 触发次数              */
  uint32_t last_spi_error_code; /**< 最近一次的 hspi1.ErrorCode                   */
  uint16_t last_pkt_hdr;        /**< RV 发来的最近一个合法小包的帧头               */
  uint32_t ctrl_reads;          /**< 小事务完成次数                                */
  uint32_t inbound_pkts;        /**< 收到的合法入站小包数                          */
  uint32_t producer_timeout;    /**< 等生产者超时的次数（超时后照样发帧）          */
  uint32_t cycle;               /**< 当前周期号                                   */
} RvLink_Stats_t;

extern RvLink_Stats_t g_rv_stats;

/* ==========================================================================
 * 接口
 * ========================================================================== */

/**
 * @brief  初始化链路：清双缓冲、CRC 建表 + 自检、PA1 拉低。
 * @note   必须在 FrameSync_Init() 之前调用（否则第一个 XVS 可能撞上未初始化的缓冲）。
 *         CRC 自检失败会直接进 Error_Handler()。
 * @note   本函数不启动 SPI，第一次装载发生在第一个 XVS 之后的 RvLink_Poll()。
 */
void RvLink_Init(void);

/**
 * @brief  XVS 到来时调用（在 TIM2 中断里，优先级 1）。必须保持很短。
 * @param  capture_ts  TIM2 CCR3 锁存的硬件时间戳（1us 单位）
 * @param  cycle       连续节拍号（= g_sync_stats.sync_cnt，XVS 常开方案下
 *                     暂停期间照样递增，保证与相机帧号的偏移恒定）
 *
 * @note   做四件事，都是寄存器级操作，约 5~10us：
 *           1. PA1 拉低，告诉 RV "别读了，正在重建"
 *           2. 硬关 SPI1 和它的两个 DMA 通道（确保 DMA 不再碰内存）
 *           3. 清出下一周期的采集缓冲，然后原子交换 collect / send 索引
 *           4. 置 pending 标志，剩下的重活交给主循环
 *         这里**不算 CRC、不装 DMA**，因为那要 100us 量级，
 *         放在优先级 1 会挡住 DMA/SPI(2) 和 TIM6(3) 的中断。
 */
void RvLink_OnFrameSync(uint32_t capture_ts, uint32_t cycle);

/**
 * @brief  在主循环里反复调用。看到 pending 就封帧、装 DMA、拉高 PA1。
 * @note   耗时约 100us（CRC 83us + 装载）。期间可被任何中断抢占，没有临界区。
 */
void RvLink_Poll(void);

/**
 * @brief  取当前正在采集的那块缓冲，供 CAN / IMU 模块写入传感器数据。
 * @return 指向 GLOVE_FRAME_WORDS 个 uint16 的缓冲
 * @note   返回值每个 XVS 周期会变，**不要缓存这个指针**，每次写之前重新取。
 */
uint16_t *RvLink_GetCollectBuffer(void);

/**
 * @brief  标记某个节点"本待封帧有数据"（把心跳位图对应位清 0）。
 * @param  bit  GLOVE_HB_xxx_BIT 之一
 * @note   任意上下文可调（内部有临界区）。心跳在每个 XVS 复位成全 1，
 *         封帧时读走 —— 所以必须在 RvLink_ProducerDone() 之前调用才算数。
 */
void RvLink_MarkNodeAlive(uint32_t bit);


/* ==========================================================================
 * 生产者闸门
 * ==========================================================================
 *
 * 【为什么需要这个】
 *   XVS 到来时冻结的 send 缓冲里装的是"刚结束的那个周期"的数据。IMU 的 FIFO
 *   批量读要 3~5ms（I2C DMA），CAN 节点回帧要 1~2ms，它们读回来的样本都属于
 *   这个刚冻结的周期，所以必须写进 send 缓冲、而且要在算 CRC 之前写完。
 *
 *   如果让它们写进 collect 缓冲（省事做法），数据就会比关节晚整整一个周期
 *   （16.7ms）—— 对一个以时间同步为核心的系统这是硬缺陷。
 *
 *   所以 RvLink_Poll() 会等：所有"预期的生产者"都调用过 RvLink_ProducerDone()
 *   之后才封帧。带超时兜底，任何一个生产者挂掉都不会卡死 SPI 链路。
 *
 * 【代价】
 *   PA1 拉高从 XVS 后约 100us 推迟到 3~5ms，RV 的读窗口从 16.5ms 缩到约 11ms。
 */

#define RV_PRODUCER_IMU             (1uL << 0)   /**< IMU 四元数            */
#define RV_PRODUCER_CAN_NATIVE      (1uL << 1)   /**< 第 2 步：3 路原生 FDCAN */
#define RV_PRODUCER_CAN_MCP         (1uL << 2)   /**< 第 4 步：2 颗 MCP2518FD */

/** 等待生产者的超时（毫秒）。超时后照样封帧发出去，缺的节点心跳位保持 1。 */
#define RV_FINALIZE_TIMEOUT_MS      8u

/**
 * @brief  声明本次运行有哪些生产者需要等待。
 * @param  mask  RV_PRODUCER_xxx 的按位或；传 0 表示不等任何人（退回原来的行为）
 * @note   在 RvLink_Init() 之后、FrameSync_Init() 之前调用。
 *         各模块初始化失败时（比如 IMU 没焊上、WHO_AM_I 读不对）就不要把它
 *         算进来，否则每帧都要白等一个超时。
 */
void RvLink_SetExpectedProducers(uint32_t mask);

/**
 * @brief  取本周期已冻结、即将发给 RV 的那块缓冲，供生产者写入数据。
 * @return 指向 GLOVE_FRAME_WORDS 个 uint16 的缓冲
 * @note   只能在"收到本周期的 pending 之后、调用 RvLink_ProducerDone() 之前"
 *         这个窗口里写。返回值每个周期都变，不要缓存指针。
 */
uint16_t *RvLink_GetSendBuffer(void);

/**
 * @brief  生产者宣告"本周期我写完了"。
 * @param  mask  自己那一位 RV_PRODUCER_xxx
 * @note   即使本周期一个样本都没拿到也要调用（缺数据由心跳位表达），
 *         否则会白等一个超时。
 */
void RvLink_ProducerDone(uint32_t mask);

/**
 * @brief  本周期是否有待封的帧（生产者用它判断该不该干活）。
 * @return 1 = 有，且还没封帧；0 = 没有
 */
uint8_t RvLink_FramePending(void);

/* ==========================================================================
 * 出站内容模式（方案 A：事务长度恒为 GLOVE_FRAME_WORDS，只切换内容）
 * ==========================================================================
 *   CTRL 内容：小包占开头 6 字 + 全 0 填充（自检/角色分配/就绪/暂停）。
 *              装载即拉高 PA1，被读走一次后拉低（同包可重复读，CRC 恒有效）。
 *   DATA 内容：XVS 驱动的数据帧流水线（运行态）。
 *   两种内容下，入站（MOSI 前 6 字）都被截存，FSM 用 RvLink_FetchInbound()
 *   取走判析 —— RV 的控制包任何时候都送得进来。
 */

/** 切到小包内容（默认上电模式）。硬停当前 SPI 并重新装载。 */
void RvLink_SetModeCtrl(void);

/** 切到数据帧内容。清空双缓冲，等下一个 XVS 启动流水线（其间 PA1 = 低）。 */
void RvLink_SetModeData(void);

/**
 * @brief  装载/替换出站小包并拉高 PA1。
 * @param  pkt  完整 6 字（含帧头与 CRC，用 GloveFrame_BuildSmallPkt 生成）
 * @note   仅 CTRL 模式有效；主循环上下文调用。替换期间 PA1 短暂拉低，
 *         若 RV 恰在读旧包会读到撕裂内容 —— 由 CRC 兜底，RV 重读即可。
 */
void RvLink_LoadCtrlPacket(const uint16_t pkt[GLOVE_SPKT_WORDS]);

/**
 * @brief  取最近一次事务里 RV 发来的 MOSI 前 6 字（未校验的原始内容）。
 * @return 1 = 有新内容并已拷出；0 = 没有
 * @note   单槽缓存：两次 Fetch 之间若来了多个事务，只保留最后一个。
 *         校验（帧头合法性 + CRC）由调用方（FSM）做。
 */
uint8_t RvLink_FetchInbound(uint16_t out[GLOVE_SPKT_WORDS]);
#endif /* RV_LINK_H */
