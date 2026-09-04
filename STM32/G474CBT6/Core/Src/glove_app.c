/**
  ******************************************************************************
  * @file    glove_app.c
  * @brief   应用层入口实现
  *
  * ============================================================================
  * 当前进度
  * ============================================================================
  *   [完成] SPI1 从机链路：双缓冲 + PA1 握手 + CRC + 生产者闸门
  *          （假数据自检保留：rv_link.h 的 RV_LINK_FAKE_DATA=1 可随时切回）
  *   [完成] 帧节拍：V4 起主手套自产 60Hz XVS（TIM2 CH3 输出比较，PB10 开漏，
  *          下降沿=同步沿）+ TIM6 看门狗；主/从/强拉低运行时切换（frame_sync）
  *   [完成] IMU：ICM-45686 六轴 FIFO 批量读 + Mahony 四元数（FSYNC 暂关）
  *   [完成] 3 路原生 FDCAN：XVS 同步广播 + 关节接收（canfd.c）
  *   [完成] 2 颗 MCP2518FD（SPI2/SPI3，bus 3..4）：同步广播 + 关节接收
  *          + 收发器 STBY 控制（初始化时拉低 = 上电即工作）
  *   [完成] 系统状态机（glove_fsm.c）：自检 → 主从分配 → 就绪 → 运行 ⇄ 暂停，
  *          SPI 事务包控制面（方案 A 恒定长度），台架自跑开关 GLOVE_FSM_AUTOSTART
  *   [待做] 触觉节点协议（CAN ID / 分帧方式定了之后加到 canfd.c 的 FIFO1）
  *   [待做] 磁力计桥接（imu.c 的 IMU_MagBridgeInit）
  *   [待做] CAN 发送包 0xFD01（RV 透传 CAN 帧，节点管理/升级用）
  *
  *   加新模块的位置：本文件的 GloveApp_Init() 和 GloveApp_Loop()，不要动 main.c。
  ******************************************************************************
  */

#include "main.h"
#include "glove_app.h"
#include "glove_protocol.h"
#include "glove_frame.h"
#include "rv_link.h"
#include "frame_sync.h"
#include "imu.h"
#include "canfd.h"
#include "mcp2518.h"
#include "glove_fsm.h"

void GloveApp_Init(void)
{
  uint32_t producers = 0u;

  /* --------------------------------------------------------------------------
   * 顺序要求：RvLink_Init() 必须在 FrameSync_Init() 之前。
   *
   * 因为 FrameSync_Init() 一旦打开 TIM2 的 CC3 中断（主机=输出比较自产节拍，
   * 从机=输入捕获外部 XVS），第一个节拍就会进中断调 RvLink_OnFrameSync()。
   * 那时候双缓冲、CRC 查表、统计结构体都必须已经是初始化好的状态，
   * 否则第一帧会拿着未初始化的内存去算 CRC。
   *
   * RvLink_Init() 内部还会做 CRC 自检（"123456789" -> 0xBB3D），
   * 不通过直接 Error_Handler()，避免带着错的 CRC 去和 RV 联调。
   * -------------------------------------------------------------------------- */
  RvLink_Init();

  /* --------------------------------------------------------------------------
   * IMU 必须在 FrameSync_Init() 之前初始化，理由同上：捕获中断一开，
   * 第一个 XVS 就会调 GloveApp_OnFrameSync() → IMU_OnFrameSync()。
   *
   * IMU_Init() 是阻塞式的（软复位 10ms + 陀螺起振 60ms），总共约 100ms，
   * 开机跑一次，此时还没有任何中断在跑，阻塞无所谓。
   *
   * 只有初始化成功才把 IMU 登记进生产者闸门 —— 芯片没焊上或 WHO_AM_I 读不对时
   * 不登记，否则每帧都要白等一个 RV_FINALIZE_TIMEOUT_MS。
   * -------------------------------------------------------------------------- */
  if (IMU_Init() != 0u)
  {
    producers |= RV_PRODUCER_IMU;
  }

  /* --------------------------------------------------------------------------
   * 3 路原生 FDCAN。Canfd_Init() 返回启动成功的总线位掩码；只要有一条活着
   * 就登记生产者（回帧窗口关闭时统一放行，单条总线坏不拖累其他）。
   * 全失败（比如收发器没上电）就不登记，闸门不会白等。
   * -------------------------------------------------------------------------- */
  if (Canfd_Init() != 0u)
  {
    producers |= RV_PRODUCER_CAN_NATIVE;
  }

#if (GLOVE_SENSOR_MODE != 2)
  /* --------------------------------------------------------------------------
   * 2 颗 MCP2518FD（bus 3..4）。初始化内部含 SPI 通信自检、2KB RAM 清零、
   * 收发器 STBY 拉低（上电即工作）、切 Normal FD 模式并回读校验。
   * 注意它排在 Canfd_Init() 之后没有依赖关系，纯粹按总线序号排。
   * -------------------------------------------------------------------------- */
  if (Mcp_Init() != 0u)
  {
    producers |= RV_PRODUCER_CAN_MCP;
  }
#endif /* 碳膜模式（GLOVE_SENSOR_MODE==2）只用 3 路原生 CAN，MCP 整条链
          （初始化/点火/轮询/生产者）不参与，芯片保持上电默认的配置态 */

  RvLink_SetExpectedProducers(producers);

  /* 启动时基与看门狗：
   *   TIM2 时基（1us，32 位自由运行，不开中断）+ XVS 线释放
   *   TIM6 更新中断（1ms -> 优先级 3，节拍看门狗）
   * 注意这里**不再自动放号**——XVS 的主/从/启停全部由状态机决定。 */
  FrameSync_Init();

  /* 系统状态机：装通信测试包、跑自检、进入角色分配。
     从这一行返回之后，整个系统由 RV 的事务包驱动
     （GLOVE_FSM_AUTOSTART=1 时台架自跑，直通 RUNNING）。 */
  Fsm_Init();
}

void GloveApp_OnFrameSync(uint32_t ts)
{
  /* 状态门控：只有 RUNNING 态的节拍才驱动采集流水线。
     READY/PAUSED 的 60Hz 只是给相机保锁相的，不广播 CAN、不动缓冲。 */
  if (Fsm_PipelineActive() == 0u)
  {
    return;
  }

  /* LED：RUNNING 态每 180 拍翻转（双方 CYCLE 同号 → 两手套同相位闪，
     肉眼同步自证）。sync_cnt 已在 FrameSync_Handle 里自增过 = 本拍的号。
     放最前，一次 GPIO 写，不占流水线时间。 */
  Fsm_LedRunTick(g_sync_stats.sync_cnt);

  /* 顺序有讲究：
     1. Canfd_OnFrameSync() 必须最先 —— SYNC 帧的发出时刻就是全系统的采样基准，
        绝不能被后面的事情推迟。它只是把帧压进 TX FIFO（约 1~2us/条），
        硬件在后台发送；节点最快也要 ~100us 后才会回帧，
        彼时下面的缓冲交换早已完成，不存在竞态。
     2. RvLink_OnFrameSync() 冻结上一周期的缓冲、交换双缓冲、复位生产者闸门。
        必须先于生产者的数据到达，因为它会清 s_producers_done 和心跳位图。
     3. IMU_OnFrameSync() 只做两件事：拉高 FSYNC（当前关闭）、置"该读 FIFO"标志。 */
  Canfd_OnFrameSync(ts);
#if (GLOVE_SENSOR_MODE != 2)
  Mcp_OnFrameSync(ts);      /* MCP 点火：预装载的 SYNC 只差一个 TXREQ 位，
                               每颗约 3us 的 SPI 事务，偏斜恒定可标定 */
#endif
  RvLink_OnFrameSync(ts, g_sync_stats.sync_cnt);   /* 周期号 = 连续节拍号 */
  IMU_OnFrameSync();
}

void GloveApp_Loop(void)
{
  /* 每个 XVS 之后把冻结的那一帧封好（帧头/长度/周期/心跳/CRC）、
     装进 SPI1 的 TX/RX DMA、最后拉高 PA1 通知 RV 可以读。
     没有待办时立即返回。 */
  /* IMU 要排在 RvLink_Poll() 之前：它是生产者，必须先把四元数写进 send 缓冲
     并调 RvLink_ProducerDone()，RvLink_Poll() 才会放行去算 CRC。
     反过来的话每帧都要多等一圈主循环。
     IMU_Poll() 是非阻塞状态机，一次调用只推进一步，没数据时立即返回。 */
  IMU_Poll();

  /* CAN：回帧窗口计时 + bus-off 恢复。接收本身全在中断里完成
     （4 字节小帧直接在 ISR 解析写帧，比排队更便宜）。 */
  Canfd_Poll();

#if (GLOVE_SENSOR_MODE != 2)
  /* MCP：读回帧（nINT 触发或窗口内巡检）、窗口关闭放行、重新预装载 SYNC */
  Mcp_Poll();
#endif

  RvLink_Poll();

  /* 状态机：入站包分发、角色分配计时、XVS 线信令、IMU 保活拍 */
  Fsm_Poll();
}
