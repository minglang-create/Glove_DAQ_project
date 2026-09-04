/**
  ******************************************************************************
  * @file    glove_fsm.c
  * @brief   系统状态机实现（状态图见 glove_fsm.h 头注释）
  *
  *          全部逻辑跑在主循环上下文（Fsm_Poll），计时用 HAL_GetTick（1ms），
  *          XVS 线的信令时长都是百 ms 量级，1ms 分辨率绰绰有余。
  ******************************************************************************
  */

#include "main.h"
#include "glove_fsm.h"
#include "glove_protocol.h"
#include "glove_frame.h"
#include "rv_link.h"
#include "frame_sync.h"
#include "imu.h"
#include "canfd.h"
#include "mcp2518.h"
#include <string.h>

/* adc.c 里由 CubeMX 生成的句柄（PA0 = VBUS 分压） */
extern ADC_HandleTypeDef hadc1;

/* ==========================================================================
 * 私有数据
 * ========================================================================== */

Fsm_Stats_t g_fsm;

static Fsm_State_e s_state = FSM_ST_SELFTEST;
static Fsm_Role_e  s_role  = FSM_ROLE_UNKNOWN;

static uint32_t s_t0        = 0u;   /* 当前状态的进入时刻 / 通用计时基准（ms） */
static uint32_t s_last_ms   = 0u;   /* 上一次 Poll 的时刻，算增量用             */

/* SELFTEST 子步骤 */
static uint8_t  s_st_step    = 0u;
static uint32_t s_st_pulses  = 0u;
static uint32_t s_st_next_ms = 0u;

/* ROLE_WAIT / MASTER_HANDSHAKE 的 XVS 线电平累计 */
static uint32_t s_line_low_ms   = 0u;
static uint32_t s_listen_ms     = 0u;   /* MASTER_LISTEN 的随机监听时长 */
static uint32_t s_slave_low_ms  = 0u;   /* MASTER_HANDSHAKE 检测从机拉低的累计 */
static uint32_t s_ready_ms      = 0u;   /* MASTER_WAIT_START：两条件齐的时刻
                                           （0=未齐），放号裕量延时从这里起算 */
static uint8_t  s_rv_ready      = 0u;   /* 收到自己 RV 的 0x5501（RV 自检 OK：
                                           只测相机 I2C 在位，相机未启动） */
static uint8_t  s_st_done       = 0u;   /* STM32 自检位图已生成 */
static volatile uint8_t s_cam_ready = 0u; /* 收到 RV 回 0xC101 = 相机已启动就绪 */

/* 非运行态的 IMU 保活拍 */
static uint32_t s_imu_next_ms = 0u;

/* ==========================================================================
 * LED（PB2 排针外接，2026-09-02 定；三态语义见 glove_fsm.h 头注释）
 *   PB2 是 V4 排针上空闲的那根；⚠ PB1 在 XVS 网络上，绝不能配成输出！
 *   .ioc 里 PB2 未配置，这里直接 HAL 初始化（CubeMX 重新生成不会碰它）。
 * ========================================================================== */

#define FSM_LED_PORT        GPIOB
#define FSM_LED_PIN         GPIO_PIN_2
/* 外接 LED 假定"高电平点亮"（推挽输出接 LED+限流电阻到地）。接反了改这里。 */
#define FSM_LED_ON_STATE    GPIO_PIN_SET

static volatile uint8_t s_led_lit = 0u;   /* 1 = 正处于"自检完成"常亮态 */

static void led_write(uint8_t on)
{
  HAL_GPIO_WritePin(FSM_LED_PORT, FSM_LED_PIN,
                    (on != 0u) ? FSM_LED_ON_STATE
                               : ((FSM_LED_ON_STATE == GPIO_PIN_SET)
                                  ? GPIO_PIN_RESET : GPIO_PIN_SET));
  s_led_lit = on;
}

static void led_init(void)
{
  GPIO_InitTypeDef g = {0};
  __HAL_RCC_GPIOB_CLK_ENABLE();
  g.Pin   = FSM_LED_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(FSM_LED_PORT, &g);
  led_write(0u);
}

/* ==========================================================================
 * 私有函数
 * ========================================================================== */

static void enter_state(Fsm_State_e st)
{
  s_state      = st;
  s_t0         = HAL_GetTick();
  g_fsm.state  = (uint8_t)st;
}

/** 装载出站小包的便捷封装 */
static void load_pkt(uint16_t hdr, uint16_t p0, uint16_t p1, uint16_t p2)
{
  uint16_t pkt[GLOVE_SPKT_WORDS];
  GloveFrame_BuildSmallPkt(pkt, hdr, p0, p1, p2);
  RvLink_LoadCtrlPacket(pkt);
}

/** 重新装载自检包（在位图/特殊状态更新后调用） */
static void load_selftest_pkt(void)
{
  load_pkt(GLOVE_PKT_SELFTEST,
           (uint16_t)(g_fsm.st_bitmap >> 16),
           (uint16_t)(g_fsm.st_bitmap & 0xFFFFu),
           g_fsm.st_special);
}

/** 读一次 VBUS 分压 ADC（阻塞 ~10ms，只在自检里用一次） */
static uint16_t adc_read_vbus(void)
{
  uint16_t raw = 0u;

  /* G4 要求使能前跑一次校准，幂等，重复自检也安全 */
  (void)HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);

  if (HAL_ADC_Start(&hadc1) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(&hadc1, 10u) == HAL_OK)
    {
      raw = (uint16_t)HAL_ADC_GetValue(&hadc1);
    }
    (void)HAL_ADC_Stop(&hadc1);
  }
  return raw;
}

/** 汇总自检结果，生成**异常位图**（2026-09-02 与 RV 统一：bit=1 = 异常/缺席，
    与数据帧心跳同极性同布局；bit28..31 在自检包里恒 0）。 */
static void build_selftest_bitmap(void)
{
  /* bit0..27 先全标"异常"，检到谁正常就清谁 */
  uint32_t bad = 0x0FFFFFFFu;
  uint32_t bus, i;

  /* 关节：按拓扑逐总线数回帧计数（SYNC 测试拍期间 rx_node 有增量即正常） */
  for (bus = 0u; bus < CANFD_BUS_CNT; bus++)
  {
    for (i = 0u; i < GLOVE_BUS_JOINT_CNT(bus); i++)
    {
      if (g_canfd.bus[bus].rx_node[i] > 0u)
      {
        bad &= ~(1uL << GLOVE_HB_JOINT_BIT(GLOVE_JOINT_SLOT(bus, i + 1u)));
      }
    }
  }
  for (i = 0u; i < MCP_CHIP_CNT; i++)
  {
    uint32_t n;
    for (n = 0u; n < GLOVE_BUS_JOINT_CNT(MCP_BUS_BASE + i); n++)
    {
      if (g_mcp.chip[i].rx_node[n] > 0u)
      {
        bad &= ~(1uL << GLOVE_HB_JOINT_BIT(GLOVE_JOINT_SLOT(MCP_BUS_BASE + i, n + 1u)));
      }
    }
  }

  /* 触觉、磁力计：未实现 → 保持 1（缺席）。IMU：WHO_AM_I 认过即正常。 */
  if (g_imu.present != 0u)
  {
    bad &= ~(1uL << GLOVE_HB_IMU_BIT);
  }

  /* PD：VBUS 分压 ADC 高于阈值 = 高压在位 → 清异常位。
     阈值 = 0（未标定）时保持 1，宁可误报缺失也不假装正常。 */
  g_fsm.pd_adc_raw = adc_read_vbus();
#if (GLOVE_FSM_PD_ADC_MIN > 0u)
  if (g_fsm.pd_adc_raw > (uint16_t)GLOVE_FSM_PD_ADC_MIN)
  {
    bad &= ~(1uL << GLOVE_ST_PD_BIT);
  }
#endif

  g_fsm.st_bitmap = bad;
}

/** 停一切输出并进自检（重新自检包的统一入口） */
static void restart_selftest(void)
{
  FrameSync_Release();        /* 唯一允许相机失锁的场合 */
  RvLink_SetModeCtrl();

  s_role              = FSM_ROLE_UNKNOWN;
  g_fsm.role          = (uint8_t)FSM_ROLE_UNKNOWN;
  g_fsm.st_bitmap     = 0u;
  g_fsm.st_special    = 0u;
  g_fsm.slave_done_seen = 0u;

  s_st_step    = 0u;
  s_st_pulses  = 0u;
  s_st_next_ms = 0u;
  s_line_low_ms  = 0u;
  s_slave_low_ms = 0u;
  s_ready_ms   = 0u;
  s_rv_ready   = 0u;          /* 重新自检后 RV 需重发 0x5501（契约要求） */
  s_st_done    = 0u;
  s_cam_ready  = 0u;
  led_write(0u);              /* 自检中灭灯，双方自检都完成时再点亮 */

  /* 清 CAN 每节点计数：挂载判断看的是"本次测试拍有没有回帧"，
     不能被上一轮运行的残留计数骗到（首次上电全局本来就是 0，无影响）。
     此刻 XVS 已停 → 无 SYNC → 节点静默，裸清 uint32 数组无竞态。 */
  {
    uint32_t b, n;
    for (b = 0u; b < CANFD_BUS_CNT; b++)
    {
      for (n = 0u; n < GLOVE_JOINTS_MAX_PER_BUS; n++)
      {
        g_canfd.bus[b].rx_node[n] = 0u;
      }
    }
    for (b = 0u; b < MCP_CHIP_CNT; b++)
    {
      for (n = 0u; n < GLOVE_JOINTS_MAX_PER_BUS; n++)
      {
        g_mcp.chip[b].rx_node[n] = 0u;
      }
    }
  }

  load_pkt(GLOVE_PKT_COMMTEST, 0xFFFFu, 0xFFFFu, 0xFFFFu);
  enter_state(FSM_ST_SELFTEST);
}

/** 从机路径的统一入口 */
static void become_slave(void)
{
  s_role     = FSM_ROLE_SLAVE;
  g_fsm.role = (uint8_t)FSM_ROLE_SLAVE;
  led_write(0u);              /* 灭灯 = 流程已启动（对侧按了键） */
  /* 0x5201 通知 RV"你是从机"。若 RV 没来得及读就被后面的 0xC301 覆盖
     也无妨 —— 0xC301 的 P0 带角色，RV 以它为准。 */
  load_pkt(GLOVE_PKT_I_AM_SLAVE, 0xFFFFu, 0xFFFFu, 0xFFFFu);
  enter_state(FSM_ST_SLAVE_WAIT_RELEASE);
}

/** 进入采集态：切数据帧内容，不碰 XVS。三个入口：主机放号时、从机切
    捕获时（首帧在第一个沿产生）、PAUSED 收到 0xC101 恢复时。 */
static void start_acq(void)
{
  RvLink_SetModeData();
  enter_state(FSM_ST_RUNNING);
  /* LED 闪烁由 Fsm_LedRunTick（XVS 节拍中断里）驱动，这里不用管 */
}

/** 停止采集：切回小包内容，XVS 不动。 */
static void stop_acq(void)
{
  RvLink_SetModeCtrl();
  load_selftest_pkt();          /* 暂停期出站包 = 最近的自检包（状态可读） */
  led_write(0u);                /* 暂停 = 灭灯（与 READY 同观感） */
  enter_state(FSM_ST_PAUSED);
}

/**
 * @brief 处理一个校验通过的入站小包。与状态机耦合最深的一段，按包分发。
 */
static void handle_inbound(uint16_t hdr)
{
  g_fsm.inbound_ok++;
  g_fsm.last_inbound     = hdr;
  g_rv_stats.last_pkt_hdr = hdr;
  g_rv_stats.inbound_pkts++;

  /* 重新自检：任何状态都接受 */
  if (hdr == GLOVE_PKT_RETEST)
  {
    restart_selftest();
    return;
  }

  /* 版本查询：非运行态应答（RUNNING 时 MISO 被数据帧占用，忽略；
     RV 请在非运行态查询）。协议版本 = 帧头低字节 = 1，不是文档修订号。 */
  if (hdr == GLOVE_PKT_VER_QUERY)
  {
    if (s_state != FSM_ST_RUNNING)
    {
      load_pkt(GLOVE_PKT_VER_REPLY, (uint16_t)GLOVE_PROTO_VERSION,
               (uint16_t)GLOVE_FW_VER_MAJOR, (uint16_t)GLOVE_FW_VER_MINOR);
    }
    return;
  }

  /* "RV 自检完成"(0x5501)：RV 只测相机 I2C 在位即发（相机此时不启动）。
     记资格标志；STM32 自检也已完成且还在等按键 → 点亮 LED（= 双方自检
     都过，可以按键了）。 */
  if (hdr == GLOVE_PKT_RV_ST_DONE)
  {
    s_rv_ready = 1u;
    if ((s_st_done != 0u) && (s_state == FSM_ST_ROLE_WAIT))
    {
      led_write(1u);
    }
    return;
  }

  switch (s_state)
  {
    case FSM_ST_ROLE_WAIT:
      if (hdr == GLOVE_PKT_YOU_ARE_MASTER)
      {
        /* 工人按键。先回"收到"，再进入随机监听（双主机竞态保险） */
        load_pkt(GLOVE_PKT_ACK, 0xFFFFu, 0xFFFFu, 0xFFFFu);
        led_write(0u);                /* 灭灯 = 流程已启动 */
        s_listen_ms = 20u + (FrameSync_Now() & 0x7Fu);   /* 20~147ms */
        enter_state(FSM_ST_MASTER_LISTEN);
      }
      break;

    case FSM_ST_MASTER_WAIT_START:
    case FSM_ST_SLAVE_HOLD:
      /* 0xC101 在这里的语义 = RV 对 0xC301 的回执："相机已启动就绪" */
      if (hdr == GLOVE_PKT_ACQ_START)
      {
        g_fsm.acq_start_cnt++;
        s_cam_ready = 1u;
      }
      break;

    case FSM_ST_RUNNING:
      if (hdr == GLOVE_PKT_ACQ_STOP)
      {
        g_fsm.acq_stop_cnt++;
        stop_acq();
      }
      break;

    case FSM_ST_PAUSED:
      if (hdr == GLOVE_PKT_ACQ_START)
      {
        g_fsm.acq_start_cnt++;
        start_acq();                  /* 恢复采集（XVS 一直在放，不碰） */
      }
      break;

    default:
      break;
  }
}

/* ==========================================================================
 * 公共接口
 * ========================================================================== */

void Fsm_Init(void)
{
  memset(&g_fsm, 0, sizeof(g_fsm));
  s_role      = FSM_ROLE_UNKNOWN;
  s_last_ms   = HAL_GetTick();
  s_imu_next_ms = 0u;

  led_init();
  restart_selftest();
}

uint8_t Fsm_PipelineActive(void)
{
  return (s_state == FSM_ST_RUNNING) ? 1u : 0u;
}

void Fsm_LedRunTick(uint32_t cycle)
{
  /* 每 GLOVE_FSM_LED_BLINK_TICKS 拍翻转一次。cycle 从 1 起、双方同号 →
     两手套的灯翻转发生在同一个物理 XVS 沿上，相位精确一致。 */
  if ((cycle % GLOVE_FSM_LED_BLINK_TICKS) == 0u)
  {
    HAL_GPIO_TogglePin(FSM_LED_PORT, FSM_LED_PIN);
  }
}

void Fsm_Poll(void)
{
  const uint32_t now = HAL_GetTick();
  const uint32_t dt  = now - s_last_ms;   /* 主循环很快，通常 0 或 1 */
  s_last_ms = now;

  /* ---- 1. 入站小包 ---- */
  {
    uint16_t raw[GLOVE_SPKT_WORDS];
    if (RvLink_FetchInbound(raw) != 0u)
    {
      const uint16_t hdr = GloveFrame_ParseSmallPkt(raw);
      if (hdr != 0u)
      {
        handle_inbound(hdr);
      }
      else
      {
        g_fsm.inbound_bad++;   /* 含正常的 MOSI 填充，涨得快是正常的 */
      }
    }
  }

  /* ---- 2. 非运行态给 IMU 补拍（60Hz 保活）----
     没有 XVS 时也要持续排空 IMU FIFO、推进静止标定，
     否则开采后第一秒都拿不到有效四元数。 */
  if ((s_state != FSM_ST_RUNNING) && (now >= s_imu_next_ms))
  {
    s_imu_next_ms = now + 16u;
    IMU_OnFrameSync();
  }

  /* ---- 3. 状态机 ---- */
  switch (s_state)
  {
    /* ------------------------------------------------------------------ */
    case FSM_ST_SELFTEST:
      switch (s_st_step)
      {
        case 0u:   /* CAN 挂载检测：以 16ms 间隔发 SYNC 测试拍 */
          if (now >= s_st_next_ms)
          {
            s_st_next_ms = now + 16u;
            Canfd_OnFrameSync(FrameSync_Now());
#if (GLOVE_SENSOR_MODE != 2)
            Mcp_OnFrameSync(FrameSync_Now());   /* 碳膜模式 MCP 整链不参与 */
#endif
            s_st_pulses++;
            if (s_st_pulses >= GLOVE_FSM_CANTEST_PULSES)
            {
              s_st_step    = 1u;
              s_st_next_ms = now + 50u;   /* 等最后一拍的回帧收尾 */
            }
          }
          break;

        case 1u:   /* 汇总 → 装自检包 → 进角色分配 */
          if (now >= s_st_next_ms)
          {
            build_selftest_bitmap();
            s_st_done = 1u;
#if (GLOVE_FSM_AUTOSTART != 0)
            s_rv_ready = 1u;        /* 台架无 RV：视同 RV 自检完成 */
#endif
            load_selftest_pkt();
            if (s_rv_ready != 0u)
            {
              led_write(1u);        /* 亮灯 = 双方自检都完成，等工人按键；
                                       RV 的 0x5501 后到则在 handle_inbound 点亮 */
            }
            s_line_low_ms = 0u;
            enter_state(FSM_ST_ROLE_WAIT);
          }
          break;

        default:
          s_st_step = 0u;
          break;
      }
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_ROLE_WAIT:
      /* 从机判据：XVS 线被持续拉低 >100ms（对面主机在上任宣告）。
         此阶段相机尚未投产，线上允许角色信令。 */
      if (FrameSync_LineIsHigh() == 0u)
      {
        s_line_low_ms += (dt != 0u) ? dt : 1u;
        g_fsm.line_low_ms = s_line_low_ms;
        if (s_line_low_ms >= GLOVE_XVS_DECLARE_MS)
        {
          become_slave();
        }
      }
      else
      {
        s_line_low_ms = 0u;
        g_fsm.line_low_ms = 0u;
      }

#if (GLOVE_FSM_AUTOSTART != 0)
      /* 台架自跑：100ms 没人认领就自封主机（等价于收到"是主机"包） */
      if ((uint32_t)(now - s_t0) > 100u)
      {
        led_write(0u);
        s_listen_ms = 20u + (FrameSync_Now() & 0x7Fu);
        enter_state(FSM_ST_MASTER_LISTEN);
      }
#endif
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_MASTER_LISTEN:
      /* 随机时长持续监听 XVS：期内线被拉低 = 对面抢先上任 → 改判从机 */
      if (FrameSync_LineIsHigh() == 0u)
      {
        become_slave();
        break;
      }
      if ((uint32_t)(now - s_t0) >= s_listen_ms)
      {
        s_role     = FSM_ROLE_MASTER;
        g_fsm.role = (uint8_t)FSM_ROLE_MASTER;
        FrameSync_ForceLow();          /* 上任宣告 + 给从机相机启动时间 */
        enter_state(FSM_ST_MASTER_CLAIM);
      }
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_MASTER_CLAIM:
      if ((uint32_t)(now - s_t0) >= GLOVE_XVS_CLAIM_MS)
      {
        /* 宣告结束：释放线，进入握手窗口（本阶段不放号）。
           从机若在，会在看到线回高后几 ms 内把线拉低并保持。 */
        FrameSync_Release();
        s_slave_low_ms = 0u;
        enter_state(FSM_ST_MASTER_HANDSHAKE);
      }
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_MASTER_HANDSHAKE:
      if (FrameSync_LineIsHigh() == 0u)
      {
        s_slave_low_ms += (dt != 0u) ? dt : 1u;
      }
      if (s_slave_low_ms >= GLOVE_XVS_SLAVE_ACK_MS)
      {
        /* 双手模式：从机在拉线（它会拉到自己相机启动完才放）。
           立刻请自己的 RV 启动相机。 */
        g_fsm.slave_done_seen = 1u;
        g_fsm.st_special     |= 0x0001u;
        s_cam_ready = 0u;
        s_ready_ms  = 0u;
        load_pkt(GLOVE_PKT_ACQ_REQ, (uint16_t)FSM_ROLE_MASTER, 1u, 0xFFFFu);
        enter_state(FSM_ST_MASTER_WAIT_START);
      }
      else if ((uint32_t)(now - s_t0) >= GLOVE_XVS_HANDSHAKE_MS)
      {
        /* 窗口内没人拉低 = 单手模式 */
        g_fsm.slave_done_seen = 0u;
        s_cam_ready = 0u;
        s_ready_ms  = 0u;
        load_pkt(GLOVE_PKT_ACQ_REQ, (uint16_t)FSM_ROLE_MASTER, 0u, 0xFFFFu);
        enter_state(FSM_ST_MASTER_WAIT_START);
      }
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_MASTER_WAIT_START:
#if (GLOVE_FSM_AUTOSTART != 0)
      if ((uint32_t)(now - s_t0) > 100u)
      {
        s_cam_ready = 1u;            /* 台架无 RV：视同相机已就绪 */
      }
#endif
      /* 就绪判定：①自己 RV 回了 0xC101（相机就绪），②双手时从机已释放线
         （从机释放 = 它的相机也就绪了）。 */
      if (s_ready_ms == 0u)
      {
        if ((s_cam_ready != 0u) &&
            ((g_fsm.slave_done_seen == 0u) || (FrameSync_LineIsHigh() != 0u)))
        {
          s_ready_ms = (now != 0u) ? now : 1u;
        }
      }
      /* 双手：两条件齐后再延 GLOVE_XVS_START_DELAY_MS（从机切捕获只要
         微秒级，理论上它先就绪，这是纯裕量）；单手：立即放号。
         第一个下降沿 = 双方 CYCLE 1，同时就是正式采集的第一拍。 */
      if ((s_ready_ms != 0u) &&
          ((g_fsm.slave_done_seen == 0u) ||
           ((uint32_t)(now - s_ready_ms) >= GLOVE_XVS_START_DELAY_MS)))
      {
        FrameSync_MasterRun();
        start_acq();                 /* 放号即采集（MISO 切数据帧） */
        break;
      }
      /* 超时保护：RV 相机启动失败 / 从机死拉线 → 回自检（LED 灭→亮，
         工人可见异常，重按键重来） */
      if ((uint32_t)(now - s_t0) >= GLOVE_FSM_CAM_START_TIMEOUT_MS)
      {
        restart_selftest();
      }
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_SLAVE_WAIT_RELEASE:
      if (FrameSync_LineIsHigh() != 0u)
      {
        /* 主机宣告结束 → 立刻拉低占住线（= 应答"从机在"，并占线表示
           "相机准备中"），同时请自己的 RV 启动相机。 */
        FrameSync_ForceLow();
        s_cam_ready = 0u;
        load_pkt(GLOVE_PKT_ACQ_REQ, (uint16_t)FSM_ROLE_SLAVE, 1u, 0xFFFFu);
        enter_state(FSM_ST_SLAVE_HOLD);
      }
      else if ((uint32_t)(now - s_t0) >= GLOVE_XVS_RELEASE_TIMEOUT_MS)
      {
        restart_selftest();          /* 主机死拉线 → 重来 */
      }
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_SLAVE_HOLD:
      /* RV 回 0xC101（相机就绪）且至少拉了 GLOVE_XVS_SLAVE_HOLD_MIN_MS
         （保证主机的握手窗口看得见）→ 释放线并切输入捕获。
         主机在我们释放后还要等自己 RV + 300ms 裕量才放第一个脉冲 →
         我们必然先就位，第一个捕获沿 = 主机第一个自产沿 = CYCLE 1。 */
      if ((s_cam_ready != 0u) &&
          ((uint32_t)(now - s_t0) >= GLOVE_XVS_SLAVE_HOLD_MIN_MS))
      {
        FrameSync_SlaveRun();        /* 内部停 OC 切 IC = 释放线 */
        start_acq();                 /* 首帧在第一个捕获沿产生 */
      }
      else if ((uint32_t)(now - s_t0) >= GLOVE_FSM_CAM_START_TIMEOUT_MS)
      {
        restart_selftest();          /* RV 相机启动失败 → 重来 */
      }
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_RUNNING:
      if (s_role == FSM_ROLE_SLAVE)
      {
        /* 节拍丢失 = 主机重新自检/断线（安全兜底，不是正常暂停路径）。
           sync_cnt != 0 的守卫：主机延迟放号，首拍之前 alive 本来就是 0，
           那不是丢失（看门狗同样有首拍门控，这里双保险）。 */
        if ((g_sync_stats.sync_cnt != 0u) && (g_sync_stats.alive == 0u))
        {
          FrameSync_ExpectTicks(0u);
          stop_acq();
        }
      }
      break;

    /* ------------------------------------------------------------------ */
    case FSM_ST_PAUSED:
      /* 恢复由各自 RV 的开采包驱动。从机若因节拍丢失落到这里，IC 通道
         仍在跑，主机的号一恢复 alive 自动回 1，这时重新武装看门狗。 */
      if ((s_role == FSM_ROLE_SLAVE) && (g_sync_stats.alive != 0u) &&
          (g_sync_stats.mode == 2u))
      {
        FrameSync_ExpectTicks(1u);
      }
      break;

    default:
      restart_selftest();
      break;
  }
}
