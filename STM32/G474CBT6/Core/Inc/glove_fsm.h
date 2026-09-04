/**
  ******************************************************************************
  * @file    glove_fsm.h
  * @brief   系统状态机：自检 → 主从分配 → 就绪 → 运行 ⇄ 暂停
  *
  * ============================================================================
  * 全局状态图（对应 myDoc/frame/frame_note 的方案，2026-09 定稿）
  * ============================================================================
  *
  *   上电
  *    │
  *    ▼
  *  SELFTEST ──── 先装通信测试包(0xAA01) → CAN 挂载检测（发 12 拍 SYNC 数回帧）
  *    │           + IMU 在位 + PD ADC → 装自检包(0x5401)
  *    │           RV 侧此时只自检相机 I2C（在不在），**不启动相机**，
  *    │           完成后发 0x5501。STM32 自检完 + 收到 0x5501 → LED 亮
  *    ▼
  *  ROLE_WAIT ──┬── XVS 低电平持续>100ms ──────────► 从机路径（见右下）
  *              └── 收到"是主机"包(0x5101，工人按键) → 回"收到"(0x5301)
  *                    │
  *                    ▼
  *              MASTER_LISTEN（随机 20~147ms 监听 XVS，被拉低→改判从机；
  *                             双主机竞态的保险，见下"竞态说明"）
  *                    │ 线始终为高
  *                    ▼
  *              MASTER_CLAIM（拉低 XVS 300ms = 上任宣告）
  *                    │ 宣告结束 → 释放线
  *                    ▼
  *              MASTER_HANDSHAKE（500ms 窗口看从机有没有把线拉低：
  *                    │           低累计≥20ms = 从机在，记 P2.bit0）
  *                    │ 有从机 / 窗口超时（单手模式）
  *                    │ → 都发 0xC301 请自己的 RV 启动相机
  *                    ▼
  *              MASTER_WAIT_START（等 ①从机释放线（单手免）
  *                    │            ②自己 RV 回 0xC101"相机就绪"；
  *                    │  两者齐 + 延时 GLOVE_XVS_START_DELAY_MS(300ms)
  *                    │  —— 从机理论上先就绪，延时是纯裕量）
  *                    │ FrameSync_MasterRun()：第一个脉冲 = 双方 CYCLE 1
  *                    ▼
  *                 RUNNING（MISO=数据帧，XVS 驱动采集流水线）
  *                    │ ▲                          从机路径：
  *              0xC201│ │0xC101              SLAVE_WAIT_RELEASE（等主机宣告
  *                    ▼ │                      结束线回高；超时 5s→重新自检）
  *                 PAUSED（XVS 照常放号）        │ 线回高 → **立刻拉低占线**
  *                                              ▼ + 发 0xC301 请 RV 启动相机
  *                                          SLAVE_HOLD（拉低保持 = 应答；
  *                                              │ RV 回 0xC101"相机就绪"
  *                                              │ 且至少拉了 100ms）
  *                                              │ → SlaveRun()：释放线 +
  *                                              │   切输入捕获，等主机首脉冲
  *                                              ▼
  *                                           RUNNING（首帧 = 第一个捕获沿）
  *
  *   ⚠ 线路事实：两手套间只有一条线 —— PB10 经互连 Type-C 的 SBU 芯直连
  *   对侧 PB10（同一网络接着相机 XVS 输入）。IMX415 下降沿触发拍摄；
  *   握手期间相机尚未启动，信令绝不产生假帧。相机启动在握手之后 →
  *   两边相机都从第一个下降沿开始拍，**帧数天然一致**；双方 CYCLE 也从
  *   同一个沿起算 = 1（同源同号，上位机直接按号对齐）。
  *   放号后线上禁止信令；停采/恢复走事务包（XVS 不停），0x5F01 才停号。
  *
  *   LED（PB2 排针外接，2026-09-02 定）：
  *     亮 = STM32 自检完成 **且** 收到 RV 的 0x5501，等工人按键
  *     灭 = 流程推进中（按键/判从之后：握手、相机启动）
  *     闪 = RUNNING，每 GLOVE_FSM_LED_BLINK_TICKS(180) 拍翻转一次（3s 半周期）
  *          —— 两手套 CYCLE 同号 → 两只手套灯完全同相位闪 = 肉眼同步自证
  *     PAUSED = 灭。⚠ PB1 在 XVS 网络上，必须永远保持输入态！
  *
  *   任何状态收到"重新自检"(0x5F01) → 停 XVS 回 SELFTEST（只有这时才失锁）。
  *
  * ============================================================================
  * 竞态说明（双主机）
  * ============================================================================
  *   用户方案：收到"是主机"后先查线电平，高才拉低，低则改判从机。
  *   残余竞口：两只手在几乎同一瞬间查线（都看到高）→ 都拉低。
  *   保险：查线改为"随机时长的持续监听"（MASTER_LISTEN），两边抽到几乎相同
  *   随机数的概率可忽略。随机源用 TIM2 计数器低位（两板上电时刻天然不同）。
  *
  * ============================================================================
  * 事务模型（方案 A：全程恒定长度）
  * ============================================================================
  *   任何阶段每次事务都是 GLOVE_FRAME_WORDS(1345) 字，不存在长度切换。
  *   非 RUNNING 时 MISO = 小包占头 6 字 + 全 0 填充；RUNNING 时 = 数据帧。
  *   RV 按字 0 的帧头区分内容，长度永远不变，没有失步问题。
  ******************************************************************************
  */

#ifndef GLOVE_FSM_H
#define GLOVE_FSM_H

#include <stdint.h>

/* ==========================================================================
 * 配置
 * ========================================================================== */

/**
 * 台架自跑开关。
 *   1 = 不需要 RV 参与：自检完自动扮演"RV 发来了 是主机→自检完成→开采"，
 *       上电直通 RUNNING（等价于旧固件的行为，方便单板调试）
 *   0 = 正式流程，全部由 RV 的包驱动
 */
#define GLOVE_FSM_AUTOSTART         0   /* 2026-09-03 关闭：进入 RV 联调，
                                           全流程由 RV 的包驱动。单板台架
                                           调试再改回 1。 */

/**
 * PD 高压在位的 ADC 判据（PA0 = VBUS 分压，12bit）。
 * raw > 此值 → 自检包 bit27 置 1。**0 = 未标定，恒报 0**。
 * 2026-09-02 实测标定：取 500。
 */
#define GLOVE_FSM_PD_ADC_MIN        500u

/** CAN 挂载检测发多少拍 SYNC（16ms 一拍） */
#define GLOVE_FSM_CANTEST_PULSES    12u

/** RUNNING 态 LED 闪烁：每多少个 XVS 拍翻转一次（180 拍 = 3s 半周期）。
    两手套 CYCLE 同号 → 两灯同相位闪，工人肉眼即可确认同步。 */
#define GLOVE_FSM_LED_BLINK_TICKS   180u

/** 等 RV 启动相机（0xC301 发出后等 0xC101 回包）的超时；超时 = 相机
    启动失败，回自检重来（LED 灭→亮，工人可见异常）。 */
#define GLOVE_FSM_CAM_START_TIMEOUT_MS  30000u

/* ==========================================================================
 * 状态
 * ========================================================================== */

typedef enum
{
  FSM_ST_SELFTEST = 0,
  FSM_ST_ROLE_WAIT,           /**< 等按键包(0x5101)或对面宣告(线低>100ms) */
  FSM_ST_MASTER_LISTEN,       /**< 随机监听，双主竞态保险                 */
  FSM_ST_MASTER_CLAIM,        /**< 拉低宣告 300ms                         */
  FSM_ST_MASTER_HANDSHAKE,    /**< 释放后 500ms 窗口检测从机拉低          */
  FSM_ST_MASTER_WAIT_START,   /**< 已发 0xC301：等从机释放+RV 回 0xC101   */
  FSM_ST_SLAVE_WAIT_RELEASE,  /**< 判从：等主机宣告结束（线回高）         */
  FSM_ST_SLAVE_HOLD,          /**< 拉低占线：等 RV 回 0xC101 → 释放切捕获 */
  FSM_ST_RUNNING,
  FSM_ST_PAUSED,
} Fsm_State_e;

typedef enum
{
  FSM_ROLE_UNKNOWN = 0,
  FSM_ROLE_MASTER,
  FSM_ROLE_SLAVE,
} Fsm_Role_e;

typedef struct
{
  uint8_t  state;             /**< Fsm_State_e，Live Expressions 看这个         */
  uint8_t  role;              /**< Fsm_Role_e                                   */
  uint32_t st_bitmap;         /**< 自检异常位图（bit=1 异常/缺席，与心跳同极性，
                                   布局见 glove_protocol.h §9）                  */
  uint16_t st_special;        /**< 自检包 P2：bit0 = 检测到从机自检完成          */
  uint16_t pd_adc_raw;        /**< 自检时采到的 VBUS ADC 原始值                 */
  uint32_t inbound_ok;        /**< 收到的合法入站小包数                          */
  uint32_t inbound_bad;       /**< CRC/帧长不合法而丢弃的入站内容数（含填充）    */
  uint16_t last_inbound;      /**< 最近一个合法入站包头                          */
  uint32_t acq_start_cnt;     /**< 收到开采包次数                                */
  uint32_t acq_stop_cnt;      /**< 收到停采包次数                                */
  uint8_t  slave_done_seen;   /**< 主机：握手窗口检测到从机拉低（=双手模式）     */
  uint32_t line_low_ms;       /**< ROLE_WAIT 里线低电平已持续毫秒（调试观察）    */
} Fsm_Stats_t;

extern Fsm_Stats_t g_fsm;

/* ==========================================================================
 * 接口
 * ========================================================================== */

/**
 * @brief  进入自检状态并装载通信测试包。
 * @note   在所有模块 Init 之后、主循环开始之前调用（FrameSync_Init 之后）。
 */
void Fsm_Init(void);

/**
 * @brief  主循环反复调用：取入站包、跑状态机、维护各阶段计时。
 * @note   非阻塞。自检期的 CAN 测试拍、非运行期的 IMU 保活拍也在这里驱动。
 */
void Fsm_Poll(void);

/**
 * @brief  采集流水线是否放行（GloveApp_OnFrameSync 的状态门控）。
 * @return 1 = RUNNING 态，XVS 节拍应驱动 CAN 广播/缓冲交换/IMU 采集
 */
uint8_t Fsm_PipelineActive(void);

/**
 * @brief  RUNNING 态每个 XVS 拍调一次（GloveApp_OnFrameSync 门控之后），
 *         驱动 LED 按 GLOVE_FSM_LED_BLINK_TICKS 翻转。中断上下文安全。
 * @param  cycle  当前连续节拍号（g_sync_stats.sync_cnt）
 */
void Fsm_LedRunTick(uint32_t cycle);

#endif /* GLOVE_FSM_H */
