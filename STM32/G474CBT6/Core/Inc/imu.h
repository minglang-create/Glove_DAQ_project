/**
  ******************************************************************************
  * @file    imu.h
  * @brief   ICM-45686 IMU：FIFO 批量读 + 四元数姿态解算（+ 预留 MMC5603 磁力计桥接）
  *
  * ============================================================================
  * 硬件连接（见 CLAUDE.md §3）
  * ============================================================================
  *   ICM-45686  挂 I2C1（PB8=SCL / PB9=SDA，400kHz，上拉 2.2k），地址 0x68
  *              （AP_AD0 接地 → 7 位地址 b1101000）
  *   FSYNC      STM32 PC13 输出脉冲 → ICM 的 FSYNC 脚
  *   MMC5603    挂在 ICM 的 AUX I2C（由 ICM 做主机自动桥接），第二阶段启用
  *   TMP1075    也在 I2C1 上（0x48），板温遥测，不属于本模块
  *
  * ============================================================================
  * 数据通路
  * ============================================================================
  *   ICM 以 IMU_ODR_HZ 的速率把样本压进片内 FIFO（2KB）。
  *   每个 XVS 周期（60Hz）主循环把 FIFO 里积攒的全部样本一次性批量读走，
  *   逐个喂给 Mahony 滤波器积分，得到该周期结束时刻的姿态四元数。
  *
  *   为什么批量读而不是每个样本读一次：
  *     400Hz 逐样本读 = 每秒 400 次 I2C 事务 + 800 次中断；
  *     60Hz 批量读 = 每秒 60 次事务，I2C 总线占用几乎一样（数据量相同），
  *     但中断次数降到 1/7，而且"这一批样本"正好就是"这一帧对应的全部运动"，
  *     积分完直接就是同步时刻的姿态，不需要额外做相位对齐。
  *
  * ============================================================================
  * FIFO 包格式（手册 §5.1；字节序注意！）
  * ============================================================================
  *   ⚠ 字节序：手册 §14 —— 芯片**默认 Little Endian**（传感器数据寄存器、
  *   FIFO 数据、FIFO 计数全部低字节在前）。§5.1 的"Ax H / Ax L"图是
  *   置位 SREG_CTRL.SREG_DATA_ENDIAN_SEL=1 之后的大端模式。
  *   本驱动不动那个位，全按默认小端解析（每个字段内低字节在前，
  *   字段偏移仍按下表）。
  *   6 轴模式 = 16 字节包：
  *     [0]     Header
  *     [1..6]  Ax H,L  Ay H,L  Az H,L
  *     [7..12] Gx H,L  Gy H,L  Gz H,L
  *     [13]    Temp（1 字节）
  *     [14,15] Timestamp H,L
  *
  *   9 轴模式 = 32 字节包（加 ES0 磁力计，包头变成 2 字节），见 imu.c 的解析函数。
  *
  *   包头位（手册 §5.2）：
  *     bit7 EXT_HEADER    1 = 包头是 2 字节（使能了外部传感器时）
  *     bit6 ACCEL_EN      1 = 含加速度数据
  *     bit5 GYRO_EN       1 = 含陀螺数据
  *     bit4 HIRES_EN      1 = 20 字节高分辨率格式
  *     bit3 TMST_FIELD_EN 1 = 含时间戳字段
  *     bit2 FSYNC_TAG_EN  1 = 这个样本被 FSYNC 打了标记，**时间戳字段改为装
  *                            FSYNC 边沿到该样本 ODR 时刻的延时**
  *     bit1 ACCEL_ODR     1 = 本包 accel ODR 与上一包不同
  *     bit0 GYRO_ODR      1 = 本包 gyro ODR 与上一包不同
  *
  *   FSYNC_TAG_EN 那一位是整个同步设计的关键：STM32 在 XVS 时刻用 PC13 打一个
  *   脉冲，ICM 会告诉我们"这个脉冲距离最近那个 IMO 采样点差了多少微秒"，
  *   RV 侧就能把姿态精确插值到相机曝光时刻。
  ******************************************************************************
  */

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

/* ==========================================================================
 * 1. 常改的配置（都在这儿，方便快速调整）
 * ========================================================================== */

/** I2C 从机地址。7 位 0x68（AP_AD0 接地）；HAL 要的是左移一位的 8 位形式。 */
#define IMU_I2C_ADDR_7BIT           0x68u
#define IMU_I2C_ADDR                ((uint16_t)(IMU_I2C_ADDR_7BIT << 1))

/* ---- 采样率（ACCEL_ODR / GYRO_ODR 的编码，手册 §16.26/16.27）----
   LN 模式下可选：0x03=6.4k 0x04=3.2k 0x05=1.6k 0x06=800 0x07=400
                  0x08=200 0x09=100 0x0A=50 0x0B=25 0x0C=12.5           */
#define IMU_ODR_800HZ               0x06u
#define IMU_ODR_400HZ               0x07u
#define IMU_ODR_200HZ               0x08u
#define IMU_ODR_100HZ               0x09u

/** 实际使用的 ODR。改这里的同时**必须**改下面的 IMU_ODR_HZ。 */
#define IMU_ODR_SEL                 IMU_ODR_400HZ
#define IMU_ODR_HZ                  400.0f

/* ---- 陀螺量程（GYRO_UI_FS_SEL）----
   0=±4000 1=±2000 2=±1000 3=±500 4=±250 5=±125 6=±62.5 7=±31.25 dps  */
#define IMU_GYRO_FS_4000DPS         0x00u
#define IMU_GYRO_FS_2000DPS         0x01u
#define IMU_GYRO_FS_1000DPS         0x02u
#define IMU_GYRO_FS_500DPS          0x03u
#define IMU_GYRO_FS_250DPS          0x04u

/** 手部快速腕转能到近千 dps，留一倍余量取 ±2000。 */
#define IMU_GYRO_FS_SEL             IMU_GYRO_FS_2000DPS

/* ---- 加速度量程（ACCEL_UI_FS_SEL）----
   0=±32g 1=±16g 2=±8g 3=±4g 4=±2g                                     */
#define IMU_ACCEL_FS_32G            0x00u
#define IMU_ACCEL_FS_16G            0x01u
#define IMU_ACCEL_FS_8G             0x02u
#define IMU_ACCEL_FS_4G             0x03u
#define IMU_ACCEL_FS_2G             0x04u

/** 手掌拍击冲击可以过 4g，取 ±8g。量程越小分辨率越高，静态定倾角越准。 */
#define IMU_ACCEL_FS_SEL            IMU_ACCEL_FS_8G

/**
 * 磁力计（MMC5603 经 ICM 的 AUX I2C 自动桥接）。
 *   0 = 只做 6 轴，FIFO 包 16 字节（当前阶段）
 *   1 = 9 轴，FIFO 包 32 字节
 * 置 1 之前请先读 imu.c 里 IMU_MagBridgeInit() 的说明 —— 那部分依赖 ICM 的
 * eDMP 和 IPREG_TOP1 间接寄存器，还没有实测通过。
 */
#define IMU_ENABLE_MAG              0

/**
 * FSYNC 打标记。
 *   1 = 每个 XVS 用 PC13 发一个脉冲，ICM 在对应样本的包头置 FSYNC_TAG_EN，
 *       并把 FSYNC→ODR 的延时放进该包的时间戳字段
 *   0 = 不用 FSYNC，姿态对齐到 ODR 网格（最大误差一个采样周期 2.5ms）
 *
 * 当前置 0：V3 板上 FSYNC 由 PC13 转发（代码本来就是这么做的），但实测
 * fsync_tag_cnt 一直为 0，标记链路未验证（待查：PC13→pin9 走线、pin9 的
 * INT2/FSYNC/CLKIN 复用是否需要额外配置）。用户决定当前阶段不追求
 * 亚采样周期的同步精度，先关掉；硬件链路验证通了再打开。
 */
#define IMU_ENABLE_FSYNC            0

/**
 * 把四元数换算成 roll/pitch/yaw 存进 g_imu.euler_deg，**仅供 Live Expressions
 * 观察**，不进帧（帧里只放四元数，避免俯仰过 ±90 度时的万向锁跳变）。
 *   1 = 计算（用到 atan2f/asinf，需要链接 libm；CubeIDE 默认带 -lm）
 *   0 = 不算，一并省掉对 math.h 的依赖
 */
#define IMU_COMPUTE_EULER_DEBUG     1

/* ---- Mahony 姿态滤波器增益 ----
   Kp 越大越信加速度（收敛快、但受线性加速度干扰大）；
   Ki 补偿陀螺零偏残差，手套场景开机已经标过零偏，给很小的值即可。 */
#define IMU_MAHONY_KP               1.0f
#define IMU_MAHONY_KI               0.02f

/* ---- 开机静止标定（CLAUDE.md §3：静止检测 → 估零偏 → 加速度定倾角 → yaw=0）---- */
/** 需要连续多少个静止样本才算标定完成（400Hz 下 400 个 = 1 秒） */
#define IMU_CALIB_SAMPLES           400u
/** 静止判据：陀螺模长上限（dps）。超过就重新累计。 */
#define IMU_CALIB_GYRO_MAX_DPS      3.0f
/** 静止判据：加速度模长必须落在 1g 附近这个带内（g） */
#define IMU_CALIB_ACC_TOL_G         0.10f

/* ---- 轴向重映射：芯片坐标系 → 手套机体坐标系 ----
   默认恒等。改的时候序号和符号要一起改，并且必须保持右手系
   （置换的行列式 × 三个符号之积 = +1），否则四元数会变成镜像。 */
#define IMU_AXIS_MAP_X              0
#define IMU_AXIS_MAP_Y              1
#define IMU_AXIS_MAP_Z              2
#define IMU_AXIS_SIGN_X             (+1)
#define IMU_AXIS_SIGN_Y             (+1)
#define IMU_AXIS_SIGN_Z             (+1)

/* ==========================================================================
 * 2. 派生常量（不用手改）
 * ========================================================================== */

/** 满量程数值。用浮点除法而不是移位，因为 ±62.5dps 这一档不是 2 的整数次幂。 */
#define IMU_GYRO_FS_DPS             (4000.0f / (float)(1u << IMU_GYRO_FS_SEL))
#define IMU_ACCEL_FS_G              (32.0f   / (float)(1u << IMU_ACCEL_FS_SEL))

/** int16 原始值 → 物理量的换算系数 */
#define IMU_GYRO_DPS_PER_LSB        (IMU_GYRO_FS_DPS / 32768.0f)
#define IMU_ACCEL_G_PER_LSB         (IMU_ACCEL_FS_G  / 32768.0f)

/** 标称采样间隔（秒）。实际积分优先用 FIFO 时间戳算出的真实间隔。 */
#define IMU_NOMINAL_DT_S            (1.0f / IMU_ODR_HZ)

/** FIFO 包长度 */
#if (IMU_ENABLE_MAG != 0)
  #define IMU_FIFO_PACKET_BYTES     32u
#else
  #define IMU_FIFO_PACKET_BYTES     16u
#endif

/** 一次批量读最多取多少个包。
    60Hz 周期 400Hz ODR 只会有 6~7 个，给到 32 个是为了容忍主循环偶发卡顿
    和将来提高 ODR。32 × 32B = 1024 字节，还在 2KB FIFO 之内。 */
#define IMU_FIFO_MAX_PACKETS        32u
#define IMU_FIFO_BUF_BYTES          (IMU_FIFO_MAX_PACKETS * IMU_FIFO_PACKET_BYTES)

/* ==========================================================================
 * 3. 数据结构
 * ========================================================================== */

/** 从 FIFO 解出来的一个原始样本 */
typedef struct
{
  int16_t  accel[3];        /**< 原始 LSB，已做轴向重映射，×IMU_ACCEL_G_PER_LSB 得 g   */
  int16_t  gyro[3];         /**< 原始 LSB，已做轴向重映射，×IMU_GYRO_DPS_PER_LSB 得 dps*/
  int8_t   temp_raw;        /**< 16 字节包里温度只有 1 字节，见 IMU_TempFromRaw()      */
  uint16_t tmst;            /**< 时间戳（1us 单位）；fsync_tag=1 时这里是 FSYNC 延时   */
  uint8_t  fsync_tag;       /**< 1 = 本样本被 FSYNC 打了标记                           */
  uint8_t  mag_valid;       /**< 1 = mag[] 有效（仅 9 轴模式）                        */
  int16_t  mag[3];          /**< MMC5603 原始值（仅 9 轴模式）                        */
} IMU_Sample_t;

/** 模块状态。用 CubeIDE 的 Live Expressions 直接看 g_imu。 */
typedef struct
{
  /* ---- 姿态 ---- */
  float    q[4];                /**< 单位四元数 w,x,y,z，机体→世界              */
  float    gyro_bias_dps[3];    /**< 开机静止标定出的陀螺零偏                   */
  float    euler_deg[3];        /**< roll,pitch,yaw，仅供调试观察，不进帧        */

  /* ---- 状态标志 ---- */
  uint8_t  present;             /**< 1 = WHO_AM_I 读对了，芯片在                */
  int16_t  accel_last[3];       /**< 最近样本的加速度原始 LSB（体检用）          */
  int16_t  gyro_last[3];        /**< 最近样本的角速度原始 LSB（体检用）          */
  float    acc_norm_g;          /**< 最近样本的加速度模长，静止时应≈1.0 g        */
  uint8_t  calibrated;          /**< 1 = 开机静止标定已完成，姿态可用            */
  uint32_t calib_progress;      /**< 已累计的静止样本数，卡在中途说明手在动      */

  /* ---- FSYNC ---- */
  uint16_t last_fsync_delay_us; /**< 最近一次 FSYNC 边沿到样本 ODR 时刻的延时    */
  uint32_t fsync_tag_cnt;       /**< 收到过 FSYNC 标记的样本数，一直是 0 说明
                                     FSYNC 没接通或极性反了                     */

  /* ---- 统计 ---- */
  uint32_t bursts;              /**< 完成的批量读次数（应≈周期数）              */
  uint32_t samples;             /**< 累计解析的样本数                           */
  uint8_t  last_burst_samples;  /**< 上一批读到几个样本（400Hz/60Hz 应为 6~7）   */
  uint16_t last_fifo_bytes;     /**< 上一次 FIFO_COUNT 读到的字节数             */
  uint32_t fifo_watermark;      /**< 观测到的 FIFO 字节数最大值（余量指标）      */
  uint32_t bad_header;          /**< 包头不认识的次数（对齐丢了 / 配置不对）     */
  uint32_t truncated;           /**< FIFO 里的包数超过缓冲、被截断的次数         */
  uint32_t i2c_error;           /**< I2C 出错次数                               */
  uint32_t i2c_stuck;           /**< 等 DMA 完成等到超时、被迫复位 I2C 的次数。
                                     非 0 说明中断没送达（NVIC 没开 I2C1_EV/ER）
                                     或总线被从机拉死                            */
  uint32_t timeout;             /**< 批量读没在本周期内完成的次数               */
} IMU_State_t;

extern IMU_State_t g_imu;

/* ==========================================================================
 * 4. 接口
 * ========================================================================== */

/**
 * @brief  初始化 ICM-45686。阻塞式（用 HAL_I2C_Mem_Write/Read + 超时），
 *         只在开机时跑一次，总耗时约 100ms（含软复位和陀螺启动等待）。
 * @return 1 = 成功（WHO_AM_I == 0xE9 且全部寄存器写入成功），0 = 失败
 * @note   返回 0 时不要把 RV_PRODUCER_IMU 登记进 RvLink_SetExpectedProducers()，
 *         否则每帧都要白等一个 8ms 超时。
 * @note   本函数不打开任何中断。后续的批量读用 I2C1_RX DMA，
 *         中断在 IMU_Poll() 里按需启动。
 */
uint8_t IMU_Init(void);

/**
 * @brief  XVS 到来时调用（TIM2 捕获中断里，优先级 1）。必须极短。
 * @note   只做两件事，纯 GPIO 写 + 置标志，不到 1us：
 *           1. IMU_ENABLE_FSYNC 时把 PC13 拉高，给 ICM 打时间标记
 *           2. 置"本周期该读 FIFO 了"的标志
 *         真正的 I2C 读在 IMU_Poll() 里做，不在中断里碰 I2C。
 */
void IMU_OnFrameSync(void);

/**
 * @brief  在主循环里反复调用，驱动"读 FIFO → 解析 → 滤波 → 写帧"的状态机。
 * @note   非阻塞。内部是三段式：
 *           IDLE  → 看到标志，启动 FIFO_COUNT 的 2 字节 DMA 读
 *           COUNT → DMA 完成，算出要读多少字节，启动 FIFO_DATA 的批量 DMA 读
 *           DATA  → DMA 完成，逐包解析 + Mahony 积分，把四元数写进 send 缓冲，
 *                   标记心跳，然后调 RvLink_ProducerDone(RV_PRODUCER_IMU)
 *         整条链路约 3~5ms（400kHz I2C 读 100 多字节），远小于 16.7ms 周期。
 * @note   即使一个样本都没读到也会调 ProducerDone，避免白等闸门超时。
 */
void IMU_Poll(void);

/**
 * @brief  取当前姿态四元数，转成协议要求的 int16 Q14 格式。
 * @param  q_out  4 个 int16，顺序 w, x, y, z
 * @note   标定未完成时返回单位四元数 (16384,0,0,0)。
 */
void IMU_GetQuatQ14(int16_t q_out[4]);

/**
 * @brief  把 16 字节包里的 1 字节温度原始值转成摄氏度。
 * @note   手册的 1 字节温度格式：温度(°C) = raw/2 + 25。
 *         这个温度是给陀螺温补用的（CLAUDE.md §3：温补用 ICM 片内温度，
 *         不是 TMP1075），本步骤只读出来观察，还没做温补。
 */
float IMU_TempFromRaw(int8_t raw);

#endif /* IMU_H */
