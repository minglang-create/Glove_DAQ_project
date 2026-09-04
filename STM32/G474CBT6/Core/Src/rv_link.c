/**
  ******************************************************************************
  * @file    rv_link.c
  * @brief   SPI1 从机链路实现
  *
  * ============================================================================
  * 双缓冲模型
  * ============================================================================
  *   s_frame[0] 和 s_frame[1] 轮流扮演两个角色：
  *     collect  —— 正在被 CAN / IMU 中断写入的那块
  *     send     —— 已冻结、正挂在 SPI DMA 上等 RV 来读的那块
  *   XVS 到来时两者互换。这样 RV 读到的永远是一整个周期的一致快照，
  *   不会出现"前半帧是第 N 周期、后半帧是第 N+1 周期"的撕裂。
  *   代价是固定一帧延迟：第 N 个 XVS 发出去的是第 N-1 周期的数据。
  *
  * ============================================================================
  * 为什么 XVS 里要硬关 SPI 和 DMA
  * ============================================================================
  *   上一周期结束时可能有两种状态：
  *     (a) 正常：RV 把 1336 个字读完了，TX DMA 计数归零，回调已触发；
  *     (b) 异常：RV 少读了、或者根本没读，DMA 停在半路，TXFIFO 里还压着旧字。
  *   不管哪种，我们都要在换缓冲之前确保 DMA 不再访问那块内存，
  *   否则 memset 会和 DMA 抢同一片地址。
  *   情况 (b) 还额外需要复位 SPI1 外设 —— 因为 STM32 的 SPI 没有单独清 TXFIFO
  *   的手段，残留的字会在下一次传输开头被先发出去，把整帧顶偏。
  ******************************************************************************
  */

#include "main.h"
#include "rv_link.h"
#include "glove_frame.h"
#include <string.h>

/* main.c 里由 CubeMX 生成的句柄 */
extern SPI_HandleTypeDef hspi1;

/* ==========================================================================
 * 私有数据
 * ========================================================================== */

RvLink_Stats_t g_rv_stats;

/* 双缓冲。4 字节对齐是为了让 memset/memcpy 走 32 位访问，快一些。 */
static uint16_t s_frame[2][GLOVE_FRAME_WORDS] __attribute__((aligned(4)));

/* RV 打时钟时 MOSI 上的数据落在这里。第一个字是命令字，其余忽略。
   必须给它一块真实的接收缓冲：SPI1 是全双工从机，如果只开 TX DMA 不收，
   RXFIFO 会被填满并置起 OVR 错误标志。 */
static uint16_t s_rx[GLOVE_FRAME_WORDS] __attribute__((aligned(4)));

static volatile uint8_t  s_collect_idx      = 0u;   /* 采集端缓冲索引 */
static volatile uint8_t  s_send_idx         = 1u;   /* 冻结/发送端缓冲索引 */
static volatile uint8_t  s_finalize_pending = 0u;   /* 主循环待办标志 */
static volatile uint8_t  s_xfer_done        = 0u;   /* 上一帧是否被完整读走 */
static volatile uint8_t  s_need_hard_reset  = 0u;   /* 需要复位 SPI1 外设 */

/* 生产者闸门。s_producers_expected 在初始化时定好，之后只读；
   s_producers_done 每个 XVS 清零，生产者干完活各自置位。 */
static uint32_t          s_producers_expected = 0u;
static volatile uint32_t s_producers_done     = 0u;
static volatile uint32_t s_finalize_t0_ms     = 0u;  /* 本周期开始等生产者的时刻 */

static volatile uint32_t s_cycle            = 0u;   /* XVS 周期号，从 1 开始 */

/* 待封帧的心跳位图。
   闸门架构下所有生产者（IMU 批量读、CAN 节点回帧）都是 XVS **之后**才把
   刚冻结那个周期的数据写进 send 缓冲的，所以心跳也只有这一份：
   XVS 时复位成全 1，生产者写完数据就清自己的位，封帧时读走。
   （最初设计有"采集中/已冻结"两份影子，那是为"XVS 之前写 collect 缓冲"
   的生产者准备的 —— 现在没有那种生产者，两份影子反而会让心跳错位一帧。） */
static volatile uint32_t s_hb_pending       = GLOVE_HB_ALL_MISSING;

/* ---- 出站内容模式（方案 A：事务长度恒为 GLOVE_FRAME_WORDS）----
   模式只在主循环里切换（FSM 上下文），ISR 只读。 */
#define RVL_MODE_CTRL               0u
#define RVL_MODE_DATA               1u
static volatile uint8_t  s_mode             = RVL_MODE_CTRL;

/* 出站小包的载体：整帧长度（方案 A 恒定事务长），小包在头 6 字，其余全 0。
   独立于双缓冲，避免与数据帧流水线的任何别名纠缠。 */
static uint16_t          s_ctrl_tx[GLOVE_FRAME_WORDS] __attribute__((aligned(4)));
static volatile uint8_t  s_ctrl_fresh       = 0u;   /* 出站包尚未被读走 */
static volatile uint8_t  s_ctrl_rearm       = 0u;   /* 需要重新装载小事务 */

static uint16_t          s_inbound[GLOVE_SPKT_WORDS];          /* 入站小包 */
static volatile uint8_t  s_inbound_flag     = 0u;

/* ==========================================================================
 * 私有函数
 * ========================================================================== */

/**
 * @brief 把帧装载进 SPI1 的 TX/RX DMA，之后只要 RV 一拉低 NSS 就会自动开始发。
 *
 * @note  HAL_SPI_TransmitReceive_DMA() 会做这些事（都是"打开中断"的动作，记录在此）：
 *          - 使能 SPI1 的 TXDMAEN / RXDMAEN（CR2）
 *          - 使能 SPI1 的 ERRIE（CR2）→ 出现 OVR / MODF / FRE 时进 SPI1_IRQHandler
 *            （NVIC 优先级 1），HAL 再回调 HAL_SPI_ErrorCallback()
 *          - 启动 DMA1_Channel1（SPI1_TX）和 DMA1_Channel6（SPI1_RX），
 *            两个通道的 TC/TE 中断都打开 → 进 DMA1_Channel1/6_IRQHandler（优先级 2）
 *          - RX 通道传输完成时 HAL 回调 HAL_SPI_TxRxCpltCallback()
 *            （TransmitReceive 模式下以 RX 完成为准，因为 RX 完成才代表时钟真的走完了）
 *
 * @note  从机模式下 DMA 一装载就会立刻把头几个字预填进 TXFIFO 和移位寄存器，
 *        所以 NSS 拉低后的第一个 SCK 边沿就能吐出正确的第一个字。
 *        这也是"必须先装 DMA、再拉高 PA1"的原因。
 */
static void RvLink_ArmSpi(uint16_t *frame, uint16_t words)
{
  /* 让 HAL 的状态机规规矩矩回到 READY。
     XVS 中断里只是硬关了外设和 DMA 通道，HAL 内部的 State 还停在 BUSY_TX_RX，
     不走这一步的话下面的 TransmitReceive_DMA 会直接返回 HAL_BUSY。
     HAL_SPI_DMAStop() 内部会 HAL_DMA_Abort 两个通道、清 TXDMAEN/RXDMAEN、
     并把 State 置回 READY，不会打开任何中断。 */
  (void)HAL_SPI_DMAStop(&hspi1);

  if (s_need_hard_reset != 0u)
  {
    s_need_hard_reset = 0u;

    /* 上一帧没被完整读走 → TXFIFO 里可能残留 1~2 个旧字。
       STM32 的 SPI 没有"清空 TXFIFO"的位，唯一确定的办法是复位整个外设。
       复位只清寄存器，不动 RCC 的时钟使能位，所以之后直接重新 Init 即可。 */
    __HAL_RCC_SPI1_FORCE_RESET();
    __HAL_RCC_SPI1_RELEASE_RESET();

    /* 置成 RESET 状态，HAL_SPI_Init() 才会重新走 HAL_SPI_MspInit()
       （重新使能时钟、重配 GPIO、重新 HAL_DMA_Init 两个通道并 LINKDMA）。
       这些动作都是幂等的，重复执行安全。 */
    hspi1.State = HAL_SPI_STATE_RESET;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
      Error_Handler();
    }
    g_rv_stats.spi_hard_reset++;
  }
  else
  {
    /* 正常路径：只需要把 RXFIFO 里可能剩下的字读干净、清掉错误标志。
       FRLVL 是 SR 里的 2 位字段，不能用 __HAL_SPI_GET_FLAG（那个是按位比较）。 */
    while ((hspi1.Instance->SR & SPI_SR_FRLVL) != 0u)
    {
      (void)hspi1.Instance->DR;
    }
    __HAL_SPI_CLEAR_OVRFLAG(&hspi1);
    __HAL_SPI_CLEAR_MODFFLAG(&hspi1);
  }

  s_xfer_done = 0u;

  /* Size 的单位是"数据帧个数"，因为 DataSize = 16BIT，所以就是 uint16 的个数 */
  if (HAL_SPI_TransmitReceive_DMA(&hspi1,
                                  (uint8_t *)frame,
                                  (uint8_t *)s_rx,
                                  words) != HAL_OK)
  {
    g_rv_stats.arm_fail++;
  }
}

/* ==========================================================================
 * 公共接口
 * ========================================================================== */

void RvLink_Init(void)
{
  memset(&g_rv_stats, 0, sizeof(g_rv_stats));
  memset(s_rx, 0, sizeof(s_rx));

  GloveFrame_CrcInit();
  if (GloveFrame_CrcSelfTest() == 0u)
  {
    /* CRC 实现不是 CRC-16/ARC，带着这个去联调只会浪费时间 */
    Error_Handler();
  }

  GloveFrame_ClearForCollect(s_frame[0]);
  GloveFrame_ClearForCollect(s_frame[1]);

  s_collect_idx      = 0u;
  s_send_idx         = 1u;
  s_finalize_pending = 0u;
  s_xfer_done        = 1u;   /* 开机第一帧不算"没读完" */
  s_need_hard_reset  = 0u;
  s_cycle            = 0u;
  s_hb_pending       = GLOVE_HB_ALL_MISSING;
  s_producers_done   = 0u;
  s_finalize_t0_ms   = 0u;
  /* s_producers_expected 不在这里清 —— 各模块初始化成功后自己调
     RvLink_SetExpectedProducers() 登记，顺序上晚于本函数。 */

  s_mode         = RVL_MODE_CTRL;
  s_ctrl_fresh   = 0u;
  s_ctrl_rearm   = 0u;
  s_inbound_flag = 0u;
  memset(s_ctrl_tx, 0, sizeof(s_ctrl_tx));   /* 头 6 字为 0 = "没有包" */

  /* PA1 = 低：还没有任何数据可读 */
  HAL_GPIO_WritePin(RV_READY_GPIO_Port, RV_READY_Pin, GPIO_PIN_RESET);
}

/** 硬停 SPI1 + 两个 DMA 通道（模式切换/换包时用，主循环上下文）。
    停在半路的事务会在 TXFIFO 留残渣，置 need_hard_reset 让下次装载走复位路径。 */
static void RvLink_HardStop(void)
{
  HAL_GPIO_WritePin(RV_READY_GPIO_Port, RV_READY_Pin, GPIO_PIN_RESET);

  __HAL_SPI_DISABLE(&hspi1);
  CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  if (hspi1.hdmatx != NULL) { __HAL_DMA_DISABLE(hspi1.hdmatx); }
  if (hspi1.hdmarx != NULL) { __HAL_DMA_DISABLE(hspi1.hdmarx); }

  if (s_xfer_done == 0u)
  {
    s_need_hard_reset = 1u;
  }
}

void RvLink_SetModeCtrl(void)
{
  RvLink_HardStop();
  s_mode             = RVL_MODE_CTRL;
  s_finalize_pending = 0u;      /* 丢弃可能挂着的封帧待办 */
  s_ctrl_rearm       = 1u;      /* 主循环会用当前 s_ctrl_tx 重新装载 */
}

void RvLink_SetModeData(void)
{
  RvLink_HardStop();
  s_mode = RVL_MODE_DATA;

  /* 双缓冲里是上一段运行/停机期间的陈旧数据，清干净再开跑：
     否则第一帧会把旧关节值当新数据发出去（心跳虽然=缺席，但数值有误导性）。 */
  GloveFrame_ClearForCollect(s_frame[0]);
  GloveFrame_ClearForCollect(s_frame[1]);
  s_finalize_pending = 0u;
  s_xfer_done        = 1u;
  /* 第一个 XVS 到来前 PA1 保持低，流水线由 XVS 启动 */
}

void RvLink_LoadCtrlPacket(const uint16_t pkt[GLOVE_SPKT_WORDS])
{
  if (s_mode != RVL_MODE_CTRL)
  {
    return;
  }
  /* 换包必须先停：从机 TX DMA 装载瞬间就把头 1~2 个字预填进了 TXFIFO，
     直接改内存不会更新已预填的字，会发出撕裂包。 */
  RvLink_HardStop();
  memcpy(s_ctrl_tx, pkt, GLOVE_SPKT_WORDS * sizeof(uint16_t));
  s_ctrl_fresh = 1u;
  s_ctrl_rearm = 1u;
}

uint8_t RvLink_FetchInbound(uint16_t out[GLOVE_SPKT_WORDS])
{
  if (s_inbound_flag == 0u)
  {
    return 0u;
  }
  /* 拷贝期间新事务可能覆盖 s_inbound —— 6 字的窗口极小，且 FSM 每圈都取，
     真撞上也只是那一包按最后到达的算，CRC 校验兜底。 */
  memcpy(out, (const void *)s_inbound, sizeof(s_inbound));
  s_inbound_flag = 0u;
  return 1u;
}

void RvLink_OnFrameSync(uint32_t capture_ts, uint32_t cycle)
{
  (void)capture_ts;   /* 时间戳目前只在 frame_sync 里用于测周期，暂不进帧 */

  if (s_mode != RVL_MODE_DATA)
  {
    return;   /* 标定期的 XVS 节拍不驱动数据流水线（FSM 也在上游拦了一道） */
  }

  /* ---- 1. 立刻通知 RV：本周期正在重建，不要发起读 ---- */
  HAL_GPIO_WritePin(RV_READY_GPIO_Port, RV_READY_Pin, GPIO_PIN_RESET);

  /* ---- 2. 硬关 SPI1 与它的两个 DMA 通道 ----
     纯寄存器写，几十个周期，中断里安全。
     顺序：先停外设产生 DMA 请求的能力，再关通道。 */
  __HAL_SPI_DISABLE(&hspi1);
  CLEAR_BIT(hspi1.Instance->CR2, SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
  if (hspi1.hdmatx != NULL)
  {
    __HAL_DMA_DISABLE(hspi1.hdmatx);
  }
  if (hspi1.hdmarx != NULL)
  {
    __HAL_DMA_DISABLE(hspi1.hdmarx);
  }

  /* ---- 3. 判断上一帧是否被完整读走 ---- */
  if (s_xfer_done == 0u)
  {
    g_rv_stats.incomplete_read++;
    g_rv_stats.tx_words_left =
        (hspi1.hdmatx != NULL) ? __HAL_DMA_GET_COUNTER(hspi1.hdmatx) : 0u;
    /* TXFIFO 里可能有残留 → 下次装载前必须复位外设 */
    s_need_hard_reset = 1u;
  }

  /* ---- 4. 清出下一周期的采集缓冲 ----
     要清的是"刚刚发完的那块"（s_send_idx）。第 2 步已经确保 DMA 不再碰它了。
     约 4us @160MHz。 */
  GloveFrame_ClearForCollect(s_frame[s_send_idx]);

  /* ---- 5. 交换索引 ----
     s_collect_idx 是单字节 volatile，Cortex-M 上单次写是原子的。
     即便更高优先级的 CAN 接收中断正好在这两行之间插进来，
     它读到的也一定是完整的旧值或新值，最坏情况只是一个采样落到了上一帧。 */
  {
    const uint8_t old_collect = s_collect_idx;
    s_collect_idx = s_send_idx;
    s_send_idx    = old_collect;
  }

  /* ---- 6. 周期号 + 心跳快照 ----
     周期号 = 连续节拍号（XVS 常开方案）：暂停期间节拍照走、号照涨，
     只是那些周期没有对应的数据帧。这样"周期号 ↔ 相机帧号"的偏移
     从放号那一刻起恒定，跨暂停段配对不用重新对齐。 */
  s_cycle = cycle;
  g_rv_stats.cycle = s_cycle;

  s_hb_pending = GLOVE_HB_ALL_MISSING;     /* 本周期的心跳从"全部缺席"开始 */

  /* ---- 7. 开闸：等生产者把刚冻结那个周期的数据写进 send 缓冲 ---- */
  s_producers_done = 0u;
  s_finalize_t0_ms = HAL_GetTick();        /* 超时基准，1ms 分辨率够用 */

  /* ---- 8. 剩下的重活交给主循环 ---- */
  s_finalize_pending = 1u;
}

void RvLink_Poll(void)
{
  uint16_t *frame;
  uint32_t  cycle;
  uint32_t  heartbeat;

  /* ---- CTRL 模式：只负责保持小事务常备 ----
     出站包被读走（或刚切换/换包）后重新装载同一内容；
     PA1 只在包是"新鲜"（装载后还没被读过）时为高。 */
  if (s_mode == RVL_MODE_CTRL)
  {
    if (s_ctrl_rearm != 0u)
    {
      s_ctrl_rearm = 0u;
      /* 方案 A：小包也走整帧长度的事务，RV 侧长度永远不变 */
      RvLink_ArmSpi(s_ctrl_tx, (uint16_t)GLOVE_FRAME_WORDS);
      if (s_ctrl_fresh != 0u)
      {
        HAL_GPIO_WritePin(RV_READY_GPIO_Port, RV_READY_Pin, GPIO_PIN_SET);
      }
    }
    return;
  }

  if (s_finalize_pending == 0u)
  {
    return;
  }

  /* ---- 生产者闸门 ----
     等 IMU / CAN / MCP 把这一周期的数据写进 send 缓冲。它们的数据属于刚冻结
     的这个周期，必须在算 CRC 之前落地，否则 CRC 会和内容不符。
     超时兜底：任何生产者卡住或没装，最多白等 RV_FINALIZE_TIMEOUT_MS，
     之后照样把帧发出去 —— 缺的节点由心跳位表达，SPI 链路绝不因此停摆。 */
  if ((s_producers_done & s_producers_expected) != s_producers_expected)
  {
    if ((uint32_t)(HAL_GetTick() - s_finalize_t0_ms) < RV_FINALIZE_TIMEOUT_MS)
    {
      return;   /* 继续等，下一次 Poll 再看 */
    }
    g_rv_stats.producer_timeout++;
  }

  s_finalize_pending = 0u;

  frame     = s_frame[s_send_idx];
  cycle     = s_cycle;
  heartbeat = s_hb_pending;

#if (RV_LINK_FAKE_DATA != 0)
  /* 第一步：没有任何真实传感器，造一帧可校验的假数据 */
  GloveFrame_FillFake(frame, cycle, &heartbeat);
#endif

  /* 填帧头/长度/周期/心跳，最后算 CRC（约 83us） */
  GloveFrame_Finalize(frame, cycle, heartbeat);

  /* 装载 DMA。必须在拉高 PA1 之前完成。 */
  RvLink_ArmSpi(frame, (uint16_t)GLOVE_FRAME_WORDS);

  /* PA1 拉高 = 本周期数据就绪，RV 可以读了。保持高到下一个 XVS。 */
  HAL_GPIO_WritePin(RV_READY_GPIO_Port, RV_READY_Pin, GPIO_PIN_SET);
  g_rv_stats.frames_armed++;
}

uint16_t *RvLink_GetCollectBuffer(void)
{
  return s_frame[s_collect_idx];
}

void RvLink_SetExpectedProducers(uint32_t mask)
{
  s_producers_expected = mask;
}

uint16_t *RvLink_GetSendBuffer(void)
{
  return s_frame[s_send_idx];
}

void RvLink_ProducerDone(uint32_t mask)
{
  s_producers_done |= mask;
}

uint8_t RvLink_FramePending(void)
{
  return s_finalize_pending;
}

void RvLink_MarkNodeAlive(uint32_t bit)
{
  if (bit < 32u)
  {
    /* 读-改-写必须加临界区：CAN 接收中断（优先级 0）和主循环里的 IMU 都会
       调这个函数，主循环的 RMW 被中断打断会丢标记（节点被误报缺席一帧）。
       关中断只有两条指令的窗口，代价可忽略。 */
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_hb_pending &= ~(1uL << bit);
    __set_PRIMASK(primask);
  }
}

/* ==========================================================================
 * HAL 回调（弱符号覆盖，不需要改 stm32g4xx_it.c）
 * ========================================================================== */

/**
 * @brief SPI1 一帧收发完成。
 * @note  调用链：RV 打完第 1336 个字的时钟
 *          → DMA1_Channel6（SPI1_RX）传输完成 → DMA1_Channel6_IRQHandler（优先级 2）
 *          → HAL_DMA_IRQHandler → HAL 内部 SPI_DMATransmitReceiveCplt
 *          → 本回调
 *        用 RX 完成而不是 TX 完成判断，是因为从机的 TX DMA 只要把数据推进 FIFO
 *        就算完成，而 RX 完成才代表 RV 真的把 1336 个时钟全打完了。
 */
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI1)
  {
    return;
  }

  s_xfer_done = 1u;

  /* 两种模式下 MOSI 前 6 字都可能是 RV 的控制小包，截存给 FSM 判析。
     全 0 / 全 0xFFFF 的填充会在 FSM 的 CRC 校验处被自然丢弃。 */
  memcpy((void *)s_inbound, s_rx, sizeof(s_inbound));
  s_inbound_flag = 1u;

  if (s_mode == RVL_MODE_CTRL)
  {
    /* 出站包被读走一次：PA1 拉低（不再"新鲜"），重新装载同一内容
       （RV 允许重复读，CRC 恒有效）。装载动作交回主循环。 */
    g_rv_stats.ctrl_reads++;
    s_ctrl_fresh = 0u;
    s_ctrl_rearm = 1u;
    HAL_GPIO_WritePin(RV_READY_GPIO_Port, RV_READY_Pin, GPIO_PIN_RESET);
  }
  else
  {
    g_rv_stats.frames_read++;
  }
}

/**
 * @brief SPI 出错。
 * @note  调用链：SPI1 的 SR 出现 OVR / MODF / FRE
 *          → SPI1_IRQHandler（优先级 1，ERRIE 由 TransmitReceive_DMA 打开）
 *          → HAL_SPI_IRQHandler → 本回调
 *        典型原因：RV 的时钟数和我们装载的字数不一致、或者 NSS 抖动。
 *        处理：只记账 + 标记需要硬复位，实际恢复统一放到下一个 XVS，
 *        避免在中断里做重活。
 */
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance != SPI1)
  {
    return;
  }

  g_rv_stats.spi_error++;
  g_rv_stats.last_spi_error_code = hspi->ErrorCode;
  s_need_hard_reset = 1u;
}
