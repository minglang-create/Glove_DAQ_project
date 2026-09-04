/**
  ******************************************************************************
  * @file    frame_sync.c
  * @brief   帧节拍源与 XVS 线控制实现（运行时切换主/从/强制电平）
  ******************************************************************************
  */

#include "main.h"
#include "frame_sync.h"
#include "glove_app.h"
#include <string.h>

/* tim.c 里由 CubeMX 生成的句柄 */
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim6;

/* ==========================================================================
 * 私有数据
 * ========================================================================== */

/* 机构模式（写进 g_sync_stats.mode 供观察） */
#define FS_MODE_RELEASED            0u
#define FS_MODE_MASTER              1u
#define FS_MODE_SLAVE               2u
#define FS_MODE_FORCE_LOW           3u

FrameSync_Stats_t g_sync_stats;

static volatile uint8_t  s_mode          = FS_MODE_RELEASED;
static volatile uint8_t  s_expect_ticks  = 0u;
static volatile uint32_t s_ms_since_sync = 0u;
static volatile uint8_t  s_have_first_ts = 0u;

/* 主机输出比较的相位：0 = 下一个比较事件产生下降沿（同步沿） */
static volatile uint8_t  s_phase_low = 0u;

/** 周期三拍抖动：16667,16667,16666（和=50000us）→ 平均精确 60.000Hz。
    只在主机 ISR 里调用，无并发。 */
static uint32_t next_period_us(void)
{
  static uint8_t k = 0u;
  k = (uint8_t)((k + 1u) % 3u);
  return (k == 0u) ? (FRAME_SYNC_PERIOD_US - 1u) : FRAME_SYNC_PERIOD_US;
}

/* ==========================================================================
 * 私有函数
 * ========================================================================== */

/** 停掉 CH3 的一切活动（比较/捕获中断、通道输出）。切换用法前必调。
    HAL 的 Stop 函数会清 CC3E —— CCMR 的通道方向位（CC3S）只有在
    CC3E=0 时才允许改写，这正是能在 OC/IC 之间来回切的前提。 */
static void ch3_stop(void)
{
  (void)HAL_TIM_OC_Stop_IT(&htim2, TIM_CHANNEL_3);
  (void)HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_3);
}

/**
 * @brief 配置 CH3 为输出比较并启动。
 * @param oc_mode  TIM_OCMODE_TOGGLE / TIM_OCMODE_FORCED_ACTIVE / FORCED_INACTIVE
 * @param with_it  1 = 开比较中断（只有 toggle 放号需要）
 *
 * 极性核算（易错，写死在这里）：
 *   统一用 TIM_OCPOLARITY_LOW → 引脚 = OCREF 取反（开漏：低=拉、高=放）。
 *   TOGGLE          ：OCREF 复位后=0 → 引脚高（释放）；每次比较翻转。
 *   FORCED_ACTIVE   ：OCREF=1 → 引脚低 = 强制拉低线。
 *   FORCED_INACTIVE ：OCREF=0 → 引脚高 = 释放线。
 */
static void ch3_start_oc(uint32_t oc_mode, uint32_t pulse, uint8_t with_it)
{
  TIM_OC_InitTypeDef oc = {0};

  oc.OCMode     = oc_mode;
  oc.OCPolarity = TIM_OCPOLARITY_LOW;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.Pulse      = pulse;

  if (HAL_TIM_OC_ConfigChannel(&htim2, &oc, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  if (with_it != 0u)
  {
    /* HAL_TIM_OC_Start_IT：置 CC3E（通道输出接管引脚）+ CC3IE（比较中断）。
       触发条件：TIM2 计数追上 CCR3；硬件已在那一微秒翻完引脚，
       然后才进 TIM2_IRQHandler（优先级 0，全系统最高——10us 脉宽窗口的保障）→ HAL_TIM_OC_DelayElapsedCallback。 */
    if (HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
  }
  else
  {
    if (HAL_TIM_OC_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
    {
      Error_Handler();
    }
  }
}

/* ==========================================================================
 * 接口
 * ========================================================================== */

void FrameSync_Init(void)
{
  memset(&g_sync_stats, 0, sizeof(g_sync_stats));
  g_sync_stats.period_min_us = 0xFFFFFFFFu;

  s_mode          = FS_MODE_RELEASED;
  s_expect_ticks  = 0u;
  s_ms_since_sync = 0u;
  s_have_first_ts = 0u;

  /* TIM2：1us 全局时基，只启动计数不开任何中断（32 位回绕由减法天然处理） */
  if (HAL_TIM_Base_Start(&htim2) != HAL_OK)
  {
    Error_Handler();
  }

  /* 上电即明确释放线（FORCED_INACTIVE，无中断），不给总线留悬空歧义 */
  FrameSync_Release();

  /* TIM6：1kHz。HAL_TIM_Base_Start_IT 置 UIE，每 1ms 进 TIM6_DAC_IRQHandler
     （优先级 3）→ HAL_TIM_PeriodElapsedCallback。职责：节拍丢失看门狗。 */
  if (HAL_TIM_Base_Start_IT(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
}

uint32_t FrameSync_Now(void)
{
  return __HAL_TIM_GET_COUNTER(&htim2);
}

void FrameSync_MasterRun(void)
{
  ch3_stop();
  s_phase_low = 0u;                     /* 第一个比较事件 = 下降沿（同步沿） */
  s_have_first_ts = 0u;
  g_sync_stats.sync_cnt = 0u;           /* CYCLE 同源：第一个下降沿 = 1。
                                           从机侧 SlaveRun 同样清零，双方数
                                           同一组物理沿 → 周期号天然一致 */
  ch3_start_oc(TIM_OCMODE_TOGGLE,
               FrameSync_Now() + FRAME_SYNC_PERIOD_US, 1u);
  s_mode = FS_MODE_MASTER;
  g_sync_stats.mode = FS_MODE_MASTER;
  FrameSync_ExpectTicks(1u);
}

void FrameSync_SlaveRun(void)
{
  TIM_IC_InitTypeDef ic = {0};

  ch3_stop();

  /* 下降沿 = 同步沿（开漏线的主动沿）。滤波 4：连续 4 个采样周期稳定才算。 */
  ic.ICPolarity  = TIM_INPUTCHANNELPOLARITY_FALLING;
  ic.ICSelection = TIM_ICSELECTION_DIRECTTI;
  ic.ICPrescaler = TIM_ICPSC_DIV1;
  ic.ICFilter    = 4u;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &ic, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* HAL_TIM_IC_Start_IT：置 CC3E + CC3IE。捕获发生时硬件把 CNT 锁进 CCR3，
     再进 TIM2_IRQHandler → HAL_TIM_IC_CaptureCallback。 */
  if (HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }

  s_have_first_ts = 0u;
  g_sync_stats.sync_cnt = 0u;           /* CYCLE 同源：第一个捕获沿 = 1（主机
                                           延迟 GLOVE_XVS_START_DELAY_MS 才放
                                           第一个脉冲，此处切换远早于它） */
  s_mode = FS_MODE_SLAVE;
  g_sync_stats.mode = FS_MODE_SLAVE;
  FrameSync_ExpectTicks(1u);
}

void FrameSync_Release(void)
{
  ch3_stop();
  ch3_start_oc(TIM_OCMODE_FORCED_INACTIVE, 0u, 0u); /* OCREF=0 → 引脚释放 */
  s_mode = FS_MODE_RELEASED;
  g_sync_stats.mode = FS_MODE_RELEASED;
  g_sync_stats.sync_cnt = 0u;   /* 只在重新自检时走到这：残留计数清掉，
                                   否则重启后的 LED"首拍灭灯"判据会被骗 */
  s_have_first_ts = 0u;
  FrameSync_ExpectTicks(0u);
}

uint8_t FrameSync_LineIsHigh(void)
{
  /* IDR 在任何模式（含 AF）下都反映引脚真实电平 */
  return ((GPIOB->IDR & GPIO_PIN_10) != 0u) ? 1u : 0u;
}

/**
 * @brief 强制把 XVS 线拉低（角色信令，仅相机锁相"投产"之前的阶段使用：
 *        主机上任宣告 2s / 从机自检完成应答 500ms）。停节拍、停中断。
 */
void FrameSync_ForceLow(void)
{
  ch3_stop();
  ch3_start_oc(TIM_OCMODE_FORCED_ACTIVE, 0u, 0u);   /* OCREF=1 → 引脚拉低 */
  s_mode = FS_MODE_FORCE_LOW;
  g_sync_stats.mode = FS_MODE_FORCE_LOW;
  FrameSync_ExpectTicks(0u);
}

/**
 * @brief 主机放号时，当前是否处于"释放相"（本机没在拉低）。
 *        10us 脉宽下释放相占 99.94%，从机应答的采样几乎不受自身脉冲打断。
 */
uint8_t FrameSync_OutputReleased(void)
{
  return ((s_mode == FS_MODE_MASTER) && (s_phase_low == 0u)) ? 1u : 0u;
}

void FrameSync_ExpectTicks(uint8_t expect)
{
  s_expect_ticks  = expect;
  s_ms_since_sync = 0u;
  if (expect == 0u)
  {
    g_sync_stats.alive = 0u;   /* 静默态 alive 无意义，归零避免误读 */
  }
}

void FrameSync_Handle(uint32_t ts)
{
  if (s_have_first_ts != 0u)
  {
    const uint32_t dt = ts - g_sync_stats.last_ts;   /* uint32 减法跨回绕正确 */
    g_sync_stats.period_us = dt;
    if (dt < g_sync_stats.period_min_us) { g_sync_stats.period_min_us = dt; }
    if (dt > g_sync_stats.period_max_us) { g_sync_stats.period_max_us = dt; }
  }
  else
  {
    s_have_first_ts = 1u;
  }

  g_sync_stats.last_ts = ts;
  g_sync_stats.sync_cnt++;
  g_sync_stats.alive   = 1u;
  s_ms_since_sync      = 0u;

  /* 驱动流水线。glove_app 里有状态门控：只有 RUNNING 态才真正跑采集。 */
  GloveApp_OnFrameSync(ts);
}

/* ==========================================================================
 * HAL 回调（弱符号覆盖）
 * ========================================================================== */

/**
 * @brief TIM 输出比较事件（主机放号）。
 * @note  TIM2_IRQHandler → HAL_TIM_IRQHandler（优先级 1）。进入时引脚已被
 *        硬件在 CCR3 那一微秒翻转完毕，边沿时刻永远精确；本函数只负责把
 *        CCR3 前推到下一个边沿的绝对时刻。
 *
 *        【10us 脉宽的迟到防护】
 *        下降沿之后只有 10us 的窗口把上升沿排进 CCR3。系统设计上此刻总线
 *        安静（CAN 由 XVS 触发，回帧早已结束），常规不会迟到；防的是同级
 *        中断（SPI1 错误/EXTI）恰好在跑、以及异步错误帧这类小概率事件。
 *        若迟到，写进去的比较值已是过去时，
 *        32 位自由跑计数器要 71 分钟才回绕 —— 线会卡死在低电平。
 *        防护：写完比较值立刻查是否已错过；错过则用 FORCED_INACTIVE 立即
 *        软件释放线（脉冲变宽但有界 = 实际中断延迟），再切回 toggle 直接排
 *        下一个下降沿。IMX415 侧偶发一个稍宽脉冲无害（8H 也是合法档位）。
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) &&
      (s_mode == FS_MODE_MASTER))
  {
    const uint32_t edge_ts = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_3);

    if (s_phase_low == 0u)
    {
      /* 刚产生下降沿 = 同步沿。第一时间排上升沿，然后才干重活。 */
      const uint32_t rise_ts = edge_ts + FRAME_SYNC_PULSE_LOW_US;

      s_phase_low = 1u;
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, rise_ts);

      /* 迟到防护（注释见函数头） */
      if ((int32_t)(FrameSync_Now() - rise_ts) >= 0)
      {
        /* 软件立即释放：FORCED_INACTIVE → OCREF=0 → 引脚高；
           再切回 TOGGLE（OCREF 保持 0，下次比较翻 1 = 下降沿）。
           两次 MODIFY_REG 都只动 OC3M 位，通道保持使能。 */
        MODIFY_REG(htim->Instance->CCMR2, TIM_CCMR2_OC3M, TIM_OCMODE_FORCED_INACTIVE);
        MODIFY_REG(htim->Instance->CCMR2, TIM_CCMR2_OC3M, TIM_OCMODE_TOGGLE);
        /* 上升比较可能恰好也发生了，丢掉它的挂起标志，防止相位机错乱 */
        __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC3);
        __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, edge_ts + next_period_us());
        s_phase_low = 0u;
        g_sync_stats.pulse_late_cnt++;
      }

      FrameSync_Handle(edge_ts);
    }
    else
    {
      /* 刚释放线（上升沿）。edge_ts = 上升时刻 = 下降 + 10us，
         下一个下降 = 本次下降 + 周期（三拍抖动出精确 60.000Hz）。 */
      s_phase_low = 0u;
      __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3,
                            edge_ts + (next_period_us() - FRAME_SYNC_PULSE_LOW_US));
    }
  }
}

/**
 * @brief TIM 输入捕获事件（从机收号）。
 * @note  CCR3 = 外部 XVS 下降沿被硬件锁存的时刻，不含中断延迟。
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  if ((htim->Instance == TIM2) && (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) &&
      (s_mode == FS_MODE_SLAVE))
  {
    FrameSync_Handle(HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3));
  }
}

/**
 * @brief TIM6 每 1ms 看门狗（优先级 3）。
 * @note  HAL 公共弱符号：将来还有别的定时器开更新中断必须在这里分流。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM6)
  {
    return;
  }

  if (s_expect_ticks == 0u)
  {
    return;   /* 自检/就绪/暂停：静默是正常的 */
  }

  if (s_have_first_ts == 0u)
  {
    return;   /* Run 已启动但第一个沿还没来：从机要等主机延迟放号
                 （最长 GLOVE_XVS_START_DELAY_MS+），这段安静是设计内的，
                 不算丢失。看门狗从第一个沿之后才开始监测。 */
  }

  if (s_ms_since_sync < 0xFFFFFFFFu)
  {
    s_ms_since_sync++;
  }

  if (s_ms_since_sync >= FRAME_SYNC_TIMEOUT_MS)
  {
    if (g_sync_stats.alive != 0u)
    {
      g_sync_stats.alive = 0u;
      g_sync_stats.sync_lost_cnt++;
      /* 主机模式走到这 = 定时器配置自伤；从机模式 = 主机停发/断线
         （FSM 用 alive 下降沿把 RUNNING 切到 PAUSED，那是正常暂停路径）。 */
    }
  }
}
