/**
  ******************************************************************************
  * @file    canfd.c
  * @brief   3 路原生 FDCAN 实现
  ******************************************************************************
  */

#include "main.h"
#include "canfd.h"
#include "rv_link.h"
#include "glove_frame.h"
#include "frame_sync.h"
#include <string.h>

/* main.c 里由 CubeMX 生成的句柄 */
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/* ==========================================================================
 * 私有数据
 * ========================================================================== */

Canfd_Stats_t g_canfd;

/** 总线序号 → HAL 句柄。序号约定见 canfd.h 头注释。 */
static FDCAN_HandleTypeDef *const s_bus[CANFD_BUS_CNT] =
{
  &hfdcan1, &hfdcan2, &hfdcan3
};

static volatile uint32_t s_window_t0   = 0u;   /* 本周期 SYNC 的发出时刻（1us） */
static volatile uint8_t  s_window_open = 0u;   /* 1 = 回帧窗口开着 */

/** SYNC 帧头。所有字段固定，做成常量免得每周期重填。 */
static const FDCAN_TxHeaderTypeDef s_sync_hdr =
{
  .Identifier          = CANFD_SYNC_ID,
  .IdType              = FDCAN_STANDARD_ID,
  .TxFrameType         = FDCAN_DATA_FRAME,
  .DataLength          = FDCAN_DLC_BYTES_1,
  .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
  .BitRateSwitch       = FDCAN_BRS_ON,        /* 数据段切 2Mbps，与节点约定一致 */
  .FDFormat            = FDCAN_FD_CAN,
  .TxEventFifoControl  = FDCAN_NO_TX_EVENTS,
  .MessageMarker       = 0u,
};

/* ==========================================================================
 * 私有函数
 * ========================================================================== */

/** HAL 句柄 → 总线序号（回调里用）。找不到返回 CANFD_BUS_CNT。 */
static uint32_t bus_index(const FDCAN_HandleTypeDef *hfdcan)
{
  uint32_t i;
  for (i = 0u; i < CANFD_BUS_CNT; i++)
  {
    if (s_bus[i] == hfdcan)
    {
      return i;
    }
  }
  return CANFD_BUS_CNT;
}

/**
 * @brief 初始化单条总线。失败返回 0（该总线不参与后续任何操作）。
 */
static uint8_t bus_init(uint32_t idx)
{
  FDCAN_HandleTypeDef *h = s_bus[idx];
  FDCAN_FilterTypeDef  f;

  /* ---- 1. 覆盖滤波器数量并重新初始化 ----
     .ioc 里 StdFiltersNbr = 0，配滤波器会被 HAL 断言拒绝。在这里改比在
     CubeMX 里改省一轮生成，而且这个值和本文件的滤波器配置强耦合，
     放在同一个文件里不容易改漏。
     HAL_FDCAN_Init 从 READY 状态重入是安全的：不会重跑 MspInit（时钟/GPIO/
     NVIC 不动），只是重进外设的 INIT+CCE 配置模式把 Init 结构重写一遍。 */
  h->Init.StdFiltersNbr = 1u;   /* 目前只有关节一条 range 滤波；触觉协议定了再加 */
  if (HAL_FDCAN_Init(h) != HAL_OK)
  {
    return 0u;
  }

  /* ---- 2. 验收滤波：只收本总线关节节点 0x101..0x104 → RX FIFO0 ----
     触觉节点将来路由到 FIFO1（凑出 3+3 深度），管理帧（HELLO/ANNOUNCE）
     到时候一并规划。 */
  /* 节点数按拓扑走（GLOVE_BUS_JOINT_CNT）。没有节点的总线也必须写一个
     DISABLE 滤波元素 —— StdFiltersNbr=1 告诉外设"滤波 RAM 里有 1 条规则"，
     而那块 RAM 上电是随机值，不显式写入等于装了一条内容随机的规则。 */
#if (GLOVE_SENSOR_MODE == 2)
  /* 碳膜 ADC2CAN：三路原生总线各收回帧 0x201..0x203。放行整个范围而不是
     本路的单个 ID，是为了容忍接线接反 —— 按帧里的 group 字节重映射。 */
  f.IdType       = FDCAN_STANDARD_ID;
  f.FilterIndex  = 0u;
  f.FilterType   = FDCAN_FILTER_RANGE;
  f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  f.FilterID1    = GLOVE_A2C_ID(1u);
  f.FilterID2    = GLOVE_A2C_ID(GLOVE_A2C_GROUPS);
#else
  f.IdType       = FDCAN_STANDARD_ID;
  f.FilterIndex  = 0u;
  f.FilterType   = FDCAN_FILTER_RANGE;
  f.FilterID1    = GLOVE_JOINT_CAN_ID_BASE;
  if (GLOVE_BUS_JOINT_CNT(idx) != 0u)
  {
    f.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    f.FilterID2    = GLOVE_BUS_JOINT_ID_MAX(idx);
  }
  else
  {
    f.FilterConfig = FDCAN_FILTER_DISABLE;     /* 这条总线没有节点，不收数据帧 */
    f.FilterID2    = GLOVE_JOINT_CAN_ID_BASE;
  }
#endif /* GLOVE_SENSOR_MODE */
  if (HAL_FDCAN_ConfigFilter(h, &f) != HAL_OK)
  {
    return 0u;
  }

  /* 不匹配任何滤波器的帧直接拒收（不占 FIFO），远程帧拒收。
     G4 的 RX FIFO 只有 3 深，一个字节都不能浪费在无关帧上。 */
  if (HAL_FDCAN_ConfigGlobalFilter(h, FDCAN_REJECT, FDCAN_REJECT,
                                   FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE) != HAL_OK)
  {
    return 0u;
  }

  /* ---- 3. TDC 发送延迟补偿 ----
     2Mbps 位时间 500ns，MCP2542FD 收发器环路延迟 100~200ns。虽然不像
     5Mbps 时（200ns 位时间）那样不开必炸，但正确配置只有好处。
     必须在 Start 之前（外设还在配置模式）调用。 */
  if (HAL_FDCAN_ConfigTxDelayCompensation(h, CANFD_TDC_OFFSET,
                                          CANFD_TDC_FILTER) != HAL_OK)
  {
    return 0u;
  }
  if (HAL_FDCAN_EnableTxDelayCompensation(h) != HAL_OK)
  {
    return 0u;
  }

  /* ---- 4. 中断分线 ----
     把错误类中断组挪到中断线 1（NVIC 里 FDCANx_IT1_IRQn，优先级 2），
     收帧类留在线 0（FDCANx_IT0_IRQn，优先级 1，仅次于 TIM2/XVS）。
     这样错误风暴（比如台面上没接任何节点时 SYNC 帧无人 ACK）永远
     不会挤占收帧中断的时间。 */
  if (HAL_FDCAN_ConfigInterruptLines(h,
        FDCAN_IT_GROUP_BIT_LINE_ERROR | FDCAN_IT_GROUP_PROTOCOL_ERROR |
        FDCAN_IT_GROUP_MISC | FDCAN_IT_GROUP_SMSG | FDCAN_IT_GROUP_TX_FIFO_ERROR,
        FDCAN_INTERRUPT_LINE1) != HAL_OK)
  {
    return 0u;
  }

  /* ---- 5. 打开具体中断源 ----
     HAL_FDCAN_ActivateNotification 置的是外设 IE 寄存器位；NVIC 的
     FDCANx_IT0/IT1_IRQn 已由 CubeMX 使能（优先级 0 / 2）。
     触发路径：
       新帧进 RX FIFO0        → FDCANx_IT0_IRQHandler（优先级 1）
                              → HAL_FDCAN_IRQHandler → HAL_FDCAN_RxFifo0Callback
       RX FIFO0 溢出（RF0L）   → 同上，RxFifo0ITs 里带 MESSAGE_LOST 位
       bus-off / 协议错误      → FDCANx_IT1_IRQHandler（优先级 2）
                              → HAL_FDCAN_ErrorStatusCallback / ErrorCallback */
  if (HAL_FDCAN_ActivateNotification(h,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST, 0u) != HAL_OK)
  {
    return 0u;
  }
  if (HAL_FDCAN_ActivateNotification(h,
        FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE | FDCAN_IT_ERROR_WARNING,
        0u) != HAL_OK)
  {
    return 0u;
  }

  /* ---- 6. 启动 ----
     HAL_FDCAN_Start 清 CCCR.INIT，外设开始参与总线。从这一刻起就能收帧。 */
  if (HAL_FDCAN_Start(h) != HAL_OK)
  {
    return 0u;
  }

  return 1u;
}

/* ==========================================================================
 * 公共接口
 * ========================================================================== */

uint8_t Canfd_Init(void)
{
  uint8_t  ok_mask = 0u;
  uint32_t i;

  memset(&g_canfd, 0, sizeof(g_canfd));
  s_window_open = 0u;

  for (i = 0u; i < CANFD_BUS_CNT; i++)
  {
    if (bus_init(i) != 0u)
    {
      g_canfd.bus[i].started = 1u;
      ok_mask |= (uint8_t)(1u << i);
    }
  }

  return ok_mask;
}

void Canfd_OnFrameSync(uint32_t ts)
{
  uint8_t  data[1] = { CANFD_SYNC_DATA0 };
  uint32_t i;

  /* 向所有活着的总线广播 SYNC。AddMessageToTxFifoQ 只是把帧写进消息 RAM
     并置 TXBAR 请求位（约 1~2us），实际发送由硬件完成 —— SYNC ID 最小
     且此刻总线空闲（节点只在收到 SYNC 后才发言），所以三条总线上的
     SYNC 几乎同时上线，偏斜在一个位时间量级。 */
  for (i = 0u; i < CANFD_BUS_CNT; i++)
  {
    if (g_canfd.bus[i].started == 0u)
    {
      continue;
    }
    if (HAL_FDCAN_AddMessageToTxFifoQ(s_bus[i], &s_sync_hdr, data) == HAL_OK)
    {
      g_canfd.bus[i].sync_tx++;
    }
    else
    {
      /* TX FIFO 满：只可能是前几帧都没发出去（总线断/无 ACK 一直重发？
         我们关了自动重发，所以更可能是 bus-off）。记账，等恢复逻辑处理。 */
      g_canfd.bus[i].sync_tx_fail++;
    }
  }

  g_canfd.cycles++;
  s_window_t0   = ts;
  s_window_open = 1u;
}

void Canfd_Poll(void)
{
  uint32_t i;

  /* ---- 回帧窗口关闭 → 放行闸门 ----
     用 TIM2 的 1us 时基而不是 HAL_GetTick，2.5ms 的窗口用 1ms 粒度太粗。 */
  if (s_window_open != 0u)
  {
    if ((uint32_t)(FrameSync_Now() - s_window_t0) >= CANFD_REPLY_WINDOW_US)
    {
      s_window_open = 0u;
      g_canfd.window_open = 0u;
      RvLink_ProducerDone(RV_PRODUCER_CAN_NATIVE);
    }
    else
    {
      g_canfd.window_open = 1u;
    }
  }

  /* ---- bus-off 恢复的第二道保险 ----
     正常恢复在 ErrorStatusCallback 里已经做了（清 INIT 位）。这里再查一遍
     协议状态，防御回调丢失的情况。频率不高（主循环空转很快），开销可忽略。 */
  for (i = 0u; i < CANFD_BUS_CNT; i++)
  {
    FDCAN_ProtocolStatusTypeDef ps;

    if (g_canfd.bus[i].started == 0u)
    {
      continue;
    }
    if (HAL_FDCAN_GetProtocolStatus(s_bus[i], &ps) == HAL_OK)
    {
      if (ps.BusOff != 0u)
      {
        /* MCAN 进 bus-off 时硬件自动置 CCCR.INIT 把自己摘下总线。
           清 INIT 后硬件开始标准恢复流程（等 129 个 11 位隐性序列）。 */
        CLEAR_BIT(s_bus[i]->Instance->CCCR, FDCAN_CCCR_INIT);
      }
    }
  }
}

/* ==========================================================================
 * HAL 回调（弱符号覆盖）
 * ========================================================================== */

/**
 * @brief RX FIFO0 有新帧 / 溢出。
 * @note  触发路径：帧通过验收滤波进入 RX FIFO0 → FDCANx_IT0_IRQHandler
 *        （NVIC 优先级 1 —— 2026-09-02 起最高的 0 让给 TIM2/XVS 脉宽整形；
 *        G4 硬件 FIFO 只有 3 深，但 500k/2M 下容忍度 ~260us，
 *        高一级的 TIM2 ISR 只占 ~30us，不构成威胁；
 *        500k/2M 下关节帧背靠背约 85us/帧，3 帧 ~260us 就塞满）
 *        → HAL_FDCAN_IRQHandler → 本回调。
 *
 *        在回调里用 while 把 FIFO 抽干，而不是取一帧就返回 —— 中断挂起期间
 *        可能又进了帧，抽干循环能少一次中断进出的开销。
 *
 *        这里直接解析并写帧（没走"中断搬运/主循环解析"的两段式），理由：
 *        关节帧只有 4 字节，解析 = 拼一个 uint32，比入队出队还便宜。
 *        将来触觉的 64 字节大帧再考虑队列。
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  const uint32_t idx = bus_index(hfdcan);
  FDCAN_RxHeaderTypeDef hdr;
  /* 64 字节：GetRxMessage 按帧实际 DLC 拷贝，对端固件出错发大帧时
     8 字节缓冲会被砸穿（潜在栈溢出），按 FD 最大帧长兜底。 */
  uint8_t  data[64];

  if (idx >= CANFD_BUS_CNT)
  {
    return;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0u)
  {
    /* 硬件 FIFO 满了还进帧 → 丢帧。这个计数一旦开始涨，
       说明中断被挡太久（检查优先级）或节点回帧撞在一起（检查节点时隙）。 */
    g_canfd.bus[idx].rx_lost++;
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0u)
  {
    return;
  }

  /* 抽干 FIFO */
  while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0u)
  {
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &hdr, data) != HAL_OK)
    {
      break;
    }

#if (GLOVE_SENSOR_MODE == 2)
    /* ---- 碳膜 ADC2CAN 回帧：16 字节 FD 帧 ----
       Byte0=frame_seq、Byte1=group(1..3)、Byte2..15=ch[0..6] uint16 小端。 */
    if ((hdr.IdType != FDCAN_STANDARD_ID) ||
        (hdr.RxFrameType != FDCAN_DATA_FRAME) ||
        (hdr.DataLength != FDCAN_DLC_BYTES_16))   /* 注意是 DLC 编码 0x0A */
    {
      g_canfd.bus[idx].rx_bad++;
      continue;
    }

    {
      const uint8_t seq   = data[0];
      const uint8_t group = data[1];
      uint32_t k;

      if ((group < 1u) || (group > GLOVE_A2C_GROUPS))
      {
        g_canfd.bus[idx].rx_bad++;
        continue;
      }
      if (hdr.Identifier != GLOVE_A2C_ID(group))
      {
        /* 总线接反：group 字节与到达总线不符。按 group 重映射，数据不丢，
           计数提示去查接线。 */
        g_canfd.bus[idx].a2c_group_mismatch++;
      }

      /* 板侧丢帧观测：seq 每轮 +1（0~255 循环），三路同轮次相同 */
      if ((g_canfd.bus[idx].rx_node[0] != 0u) &&
          (seq != (uint8_t)(g_canfd.bus[idx].a2c_last_seq + 1u)))
      {
        g_canfd.bus[idx].a2c_seq_gap++;
      }
      g_canfd.bus[idx].a2c_last_seq = seq;

      /* 一帧携带整组数据：有效通道的节点计数同步 +1（group3 只有 6 路有效，
         第 21 路丢弃），自检位图的通用循环（fsm）因此不需要任何模式分支 */
      {
        const uint32_t base_cnt = ((uint32_t)group - 1u) * GLOVE_A2C_CH_PER_GRP;
        for (k = 0u; (k < GLOVE_A2C_CH_PER_GRP) &&
                     ((base_cnt + k) < GLOVE_JOINT_CNT); k++)
        {
          g_canfd.bus[idx].rx_node[k]++;
        }
      }
      g_canfd.bus[idx].last_joint_raw[0] =
          (uint32_t)data[2] | ((uint32_t)data[3] << 8u);   /* ch[0] 体检用 */

      if (RvLink_FramePending() != 0u)
      {
        uint16_t *frame = RvLink_GetSendBuffer();
        const uint32_t base = ((uint32_t)group - 1u) * GLOVE_A2C_CH_PER_GRP;

        for (k = 0u; k < GLOVE_A2C_CH_PER_GRP; k++)
        {
          const uint32_t v    = (uint32_t)data[2u + 2u * k] |
                                ((uint32_t)data[3u + 2u * k] << 8u);
          const uint32_t slot = base + k;

          if (slot < GLOVE_JOINT_CNT)
          {
            GloveFrame_SetJoint(frame, slot, v);          /* 12bit 放 bit22:0 */
            RvLink_MarkNodeAlive(GLOVE_HB_JOINT_BIT(slot));
          }
          /* slot 20 = 第 21 路 ADC（group3 ch[6]）：直接丢弃 ——
             对 RV 与 20 关节方案零差异（2026-09-02 定） */
        }
        g_canfd.bus[idx].rx_joint++;
      }
      else
      {
        g_canfd.bus[idx].rx_late++;
      }
    }
  }
}
#else /* GLOVE_SENSOR_MODE != 2：关节节点协议（原逻辑，勿动） */
    /* 合规性检查。滤波器已保证 ID 在 0x101..0x104，这里再防 DLC / 帧型不对
       （节点固件的 bug 不该污染我们的数据）。 */
    if ((hdr.IdType != FDCAN_STANDARD_ID) ||
        (hdr.RxFrameType != FDCAN_DATA_FRAME) ||
        (hdr.DataLength != FDCAN_DLC_BYTES_4) ||
        (hdr.Identifier < GLOVE_JOINT_CAN_ID_BASE) ||
        (hdr.Identifier > GLOVE_BUS_JOINT_ID_MAX(idx)))
    {
      g_canfd.bus[idx].rx_bad++;
      continue;
    }

    {
      /* CAN 载荷是小端：data[0] = LSB */
      const uint32_t raw32 = ((uint32_t)data[0])        |
                             ((uint32_t)data[1] << 8u)  |
                             ((uint32_t)data[2] << 16u) |
                             ((uint32_t)data[3] << 24u);
      const uint32_t node = GLOVE_JOINT_NODE_FROM_ID(hdr.Identifier);  /* 1..4 */
      const uint32_t slot = GLOVE_JOINT_SLOT(idx, node);               /* 0..19 */

      g_canfd.bus[idx].last_joint_raw[node - 1u] = raw32;
      g_canfd.bus[idx].rx_node[node - 1u]++;

      /* 只在帧还没封（CRC 还没算）的时候写。晚到的帧宁可丢：
         封帧后再写会让内容和 CRC 不一致，RV 侧整帧作废，损失更大。
         这个检查在 ISR 里是无竞态的：ISR 完整执行完主循环才会继续，
         所以"看到 pending=1 然后写入"一定发生在 CRC 计算开始之前。 */
      if (RvLink_FramePending() != 0u)
      {
        GloveFrame_SetJoint(RvLink_GetSendBuffer(), slot, raw32);
        RvLink_MarkNodeAlive(GLOVE_HB_JOINT_BIT(slot));
        g_canfd.bus[idx].rx_joint++;
      }
      else
      {
        g_canfd.bus[idx].rx_late++;
      }
    }
  }
}
#endif /* GLOVE_SENSOR_MODE */

/**
 * @brief 总线错误状态变化（bus-off / error-passive / error-warning）。
 * @note  触发路径：FDCANx_IT1_IRQHandler（NVIC 优先级 2，错误组已分到线 1）
 *        → HAL_FDCAN_IRQHandler → 本回调。
 *        台面上没接节点时 SYNC 帧无人 ACK，错误计数会爬到 error-passive，
 *        这是正常现象（err_irq 会涨），接上第一个节点就消停。
 */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  const uint32_t idx = bus_index(hfdcan);

  if (idx >= CANFD_BUS_CNT)
  {
    return;
  }

  g_canfd.bus[idx].err_irq++;

  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0u)
  {
    g_canfd.bus[idx].busoff++;
    /* 立即发起恢复：清 INIT，硬件走标准 bus-off 恢复序列。
       不在这里做更多事 —— 统计已留痕，Poll 里还有第二道保险。 */
    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
  }
}
