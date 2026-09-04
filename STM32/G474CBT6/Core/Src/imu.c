/**
  ******************************************************************************
  * @file    imu.c
  * @brief   ICM-45686 驱动 + Mahony 四元数姿态解算
  *
  *          寄存器地址、位域、量程、时序全部取自手册 DS-000489 Rev 1.1，
  *          关键处标了章节号，便于复核。
  ******************************************************************************
  */

#include "main.h"
#include "imu.h"
#include "rv_link.h"
#include "glove_frame.h"
#include "glove_protocol.h"
#include <string.h>

#if (IMU_COMPUTE_EULER_DEBUG != 0)
#include <math.h>
#endif

/* main.c 里由 CubeMX 生成的句柄 */
extern I2C_HandleTypeDef hi2c1;

/* ==========================================================================
 * ICM-45686 寄存器（User Bank 0，手册 §15.1 / §16）
 * ========================================================================== */

#define ICM_REG_PWR_MGMT0                   0x10u   /* §16.17 */
#define ICM_REG_FIFO_COUNT_0                0x12u   /* §16.18（默认小端模式下 0x12=低字节，见 §14） */
#define ICM_REG_FIFO_DATA                   0x14u   /* §16.20 */
#define ICM_REG_ACCEL_CONFIG0               0x1Bu   /* §16.26 */
#define ICM_REG_GYRO_CONFIG0                0x1Cu   /* §16.27 */
#define ICM_REG_FIFO_CONFIG0                0x1Du   /* §16.28 */
#define ICM_REG_FIFO_CONFIG1_0              0x1Eu   /* §16.29 水位低字节 */
#define ICM_REG_FIFO_CONFIG1_1              0x1Fu   /* §16.30 水位高字节 */
#define ICM_REG_FIFO_CONFIG2                0x20u   /* §16.31 */
#define ICM_REG_FIFO_CONFIG3                0x21u   /* §16.32 */
#define ICM_REG_FIFO_CONFIG4                0x22u   /* §16.33 */
#define ICM_REG_TMST_WOM_CONFIG             0x23u   /* §16.34 */
#define ICM_REG_FSYNC_CONFIG0               0x24u   /* §16.35 */
#define ICM_REG_DMP_EXT_SEN_ODR_CFG         0x27u   /* §16.38 */
#define ICM_REG_IOC_PAD_SCENARIO            0x2Fu   /* §16.46 只读 */
#define ICM_REG_IOC_PAD_SCENARIO_AUX_OVRD   0x30u   /* §16.47 */
#define ICM_REG_WHO_AM_I                    0x72u   /* §16.79 */
#define ICM_REG_IREG_ADDR_15_8              0x7Cu   /* §16.81 */
#define ICM_REG_IREG_ADDR_7_0               0x7Du   /* §16.82 */
#define ICM_REG_IREG_DATA                   0x7Eu   /* §16.83 */
#define ICM_REG_MISC2                       0x7Fu   /* §16.84 */

/** WHO_AM_I 的复位值。注意这和 I2C 从机地址是两回事。 */
#define ICM_WHOAMI_VALUE                    0xE9u

/* PWR_MGMT0：GYRO_MODE[3:2]，ACCEL_MODE[1:0]，11 = Low Noise 模式 */
#define ICM_GYRO_MODE_LN                    (0x03u << 2)
#define ICM_ACCEL_MODE_LN                   (0x03u << 0)

/* FIFO_CONFIG0：FIFO_MODE[7:6]，FIFO_DEPTH[5:0] */
#define ICM_FIFO_MODE_BYPASS                (0x00u << 6)
#define ICM_FIFO_MODE_STREAM                (0x01u << 6)
#define ICM_FIFO_MODE_STOP_ON_FULL          (0x02u << 6)
#define ICM_FIFO_DEPTH_2K                   0x07u

/* FIFO_CONFIG2 */
#define ICM_FIFO_FLUSH                      (1u << 7)

/* FIFO_CONFIG3 */
#define ICM_FIFO_ES1_EN                     (1u << 5)
#define ICM_FIFO_ES0_EN                     (1u << 4)
#define ICM_FIFO_HIRES_EN                   (1u << 3)
#define ICM_FIFO_GYRO_EN                    (1u << 2)
#define ICM_FIFO_ACCEL_EN                   (1u << 1)
#define ICM_FIFO_IF_EN                      (1u << 0)

/* FIFO_CONFIG4 */
#define ICM_FIFO_COMP_EN                    (1u << 2)
#define ICM_FIFO_TMST_FSYNC_EN              (1u << 1)
#define ICM_FIFO_ES0_9B                     (1u << 0)

/* TMST_WOM_CONFIG */
#define ICM_TMST_DELTA_EN                   (1u << 6)
#define ICM_TMST_RESOL_16US                 (1u << 5)

/* FSYNC_CONFIG0：AP_FSYNC_SEL[2:0]。
   1 = 把 FSYNC 标记打到 TEMP_DATA_UI 的 LSB。选温度是因为它最无害
   （我们只用温度做粗略监视），打到 gyro/accel 的 LSB 会污染姿态数据。
   注意：AP_FSYNC_SEL = 0 表示"FSYNC 标记功能关闭"，那样 FIFO 包头的
   FSYNC_TAG_EN 也不会置位，所以这里必须给非零值。 */
#define ICM_AP_FSYNC_SEL_TEMP_LSB           0x01u

/* REG_MISC2 */
#define ICM_SOFT_RST                        (1u << 1)
#define ICM_IREG_DONE                       (1u << 0)

/* FIFO 包头位（手册 §5.2） */
#define ICM_HDR_EXT_HEADER                  (1u << 7)
#define ICM_HDR_ACCEL_EN                    (1u << 6)
#define ICM_HDR_GYRO_EN                     (1u << 5)
#define ICM_HDR_HIRES_EN                    (1u << 4)
#define ICM_HDR_TMST_FIELD_EN               (1u << 3)
#define ICM_HDR_FSYNC_TAG_EN                (1u << 2)

/** 校验包头时只看这 5 位，低 3 位（FSYNC 标记 + ODR 变化标记）每包都可能变 */
#define ICM_HDR_CHECK_MASK                  0xF8u
#if (IMU_ENABLE_MAG != 0)
  /* 9 轴：2 字节包头 + accel + gyro + ES0 + ES1 + temp + tmst = 32 字节 */
  #define ICM_HDR_EXPECTED  (ICM_HDR_EXT_HEADER | ICM_HDR_ACCEL_EN | \
                             ICM_HDR_GYRO_EN    | ICM_HDR_TMST_FIELD_EN)   /* 0xE8 */
#else
  /* 6 轴：1 字节包头 + accel + gyro + temp + tmst = 16 字节 */
  #define ICM_HDR_EXPECTED  (ICM_HDR_ACCEL_EN | ICM_HDR_GYRO_EN | \
                             ICM_HDR_TMST_FIELD_EN)                        /* 0x68 */
#endif

/* 第二字节包头（仅使能外部传感器时存在，手册 §5.2） */
#define ICM_HDR2_ES0_9B                     (1u << 4)
#define ICM_HDR2_ES1_VLD                    (1u << 3)
#define ICM_HDR2_ES0_VLD                    (1u << 2)

/* 16 字节包内偏移 */
#define ICM_PKT6_OFF_ACCEL                  1u
#define ICM_PKT6_OFF_GYRO                   7u
#define ICM_PKT6_OFF_TEMP                   13u
#define ICM_PKT6_OFF_TMST                   14u

/* 32 字节包内偏移（ES0 = 9 字节时 ES1 从 23 开始） */
#define ICM_PKT9_OFF_ACCEL                  2u
#define ICM_PKT9_OFF_GYRO                   8u
#define ICM_PKT9_OFF_ES0                    14u
#define ICM_PKT9_OFF_TEMP                   29u
#define ICM_PKT9_OFF_TMST                   30u

/** 阻塞式寄存器访问的超时（ms）。只在初始化用，正常路径走 DMA。 */
#define ICM_I2C_TIMEOUT_MS                  20u

/** 等一次 DMA 读完成的上限（ms）。正常 3~5ms 就完了；超过这个值说明完成中断
    永远不会来了（NVIC 没开 I2C1_EV/ER、线断了、从机把总线拉死），
    必须复位 I2C 外设脱困，否则状态机会永久卡死到重启。 */
#define ICM_XFER_STUCK_MS                   12u

/* ==========================================================================
 * 模块状态
 * ========================================================================== */

IMU_State_t g_imu;

/** 批量读的状态机 */
typedef enum
{
  IMU_ST_IDLE = 0,      /* 空闲，等下一个 XVS            */
  IMU_ST_COUNT_WAIT,    /* 已发起 FIFO_COUNT 读，等 DMA  */
  IMU_ST_DATA_WAIT      /* 已发起 FIFO_DATA 读，等 DMA   */
} IMU_State_e;

static volatile IMU_State_e s_state       = IMU_ST_IDLE;
static volatile uint8_t     s_sync_pending = 0u;   /* XVS 来过，该读 FIFO 了 */
static volatile uint8_t     s_dma_done     = 0u;   /* I2C DMA 读完成         */
static volatile uint8_t     s_dma_error    = 0u;   /* I2C 出错               */

static uint8_t  s_count_buf[2];                          /* FIFO 包数 */
static uint8_t  s_fifo_buf[IMU_FIFO_BUF_BYTES];          /* 批量读缓冲 */
static uint16_t s_read_bytes  = 0u;                      /* 本次实际读多少字节 */
static uint8_t  s_read_packets = 0u;

/* Mahony 积分器状态 */
static float s_e_int[3] = { 0.0f, 0.0f, 0.0f };

/* 开机静止标定的累加器 */
static float    s_cal_gyro_sum[3] = { 0.0f, 0.0f, 0.0f };
static float    s_cal_acc_sum[3]  = { 0.0f, 0.0f, 0.0f };
static uint32_t s_cal_cnt         = 0u;

/* 上一个样本的时间戳，用来算真实 dt */
static uint16_t s_prev_tmst      = 0u;
static uint8_t  s_prev_tmst_ok   = 0u;

/* 进入等待态的时刻，用于检测"完成中断永远不来"的卡死 */
static uint32_t s_wait_t0_ms     = 0u;

/* ==========================================================================
 * 小工具
 * ========================================================================== */

/**
 * @brief 平方根倒数。
 * @note  用 Quake 那套魔数 + 两次牛顿迭代，而不是 1.0f/sqrtf(x)：
 *        既不依赖 libm，执行时间也是恒定的（没有分支和查表）。
 *        两次迭代后相对误差 < 1e-6，用于归一化完全够。
 *        用 union 而不是指针强转，避免违反严格别名规则。
 */
static float inv_sqrt(float x)
{
  union { float f; uint32_t i; } u;
  const float halfx = 0.5f * x;

  u.f = x;
  u.i = 0x5F3759DFu - (u.i >> 1);
  u.f = u.f * (1.5f - halfx * u.f * u.f);
  u.f = u.f * (1.5f - halfx * u.f * u.f);

  return u.f;
}

/** 从小端字节流取 int16（低字节在前）。
    ICM-45686 手册 §14：芯片默认 Little Endian —— 传感器数据寄存器、FIFO 数据、
    FIFO 计数**全部**低字节在前；§5.1 那张"Ax H / Ax L"包格式图标注的是
    "(big endian mode)"，即手动置位 SREG_CTRL.SREG_DATA_ENDIAN_SEL=1 之后
    才是那个字节序。我们不动那个位（它在 IPREG_TOP1 间接银行），全按默认小端解析。
    这是实测踩过的坑：按大端解析时包头全对（单字节无字节序），但静止时
    az=0x1000 被读成 0x0010≈0、陀螺噪声被放大 256 倍，静止标定永远不通过。 */
static inline int16_t le16(const uint8_t *p)
{
  return (int16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

/** 从小端字节流取 uint16 */
static inline uint16_t leu16(const uint8_t *p)
{
  return (uint16_t)(((uint16_t)p[1] << 8) | (uint16_t)p[0]);
}

float IMU_TempFromRaw(int8_t raw)
{
  /* 手册 Table「TEMPERATURE SENSOR」：25°C 输出 = 0 LSB（二进制补码），
     FIFO 数据的灵敏度 = 2 LSB/°C（寄存器路径是 128 LSB/°C，别混用）。 */
  return ((float)raw * 0.5f) + 25.0f;
}

/**
 * @brief 把芯片坐标系的三轴按 IMU_AXIS_MAP_* / IMU_AXIS_SIGN_* 重排到机体坐标系。
 */
static void axis_remap(const int16_t in[3], int16_t out[3])
{
  out[0] = (int16_t)(IMU_AXIS_SIGN_X * in[IMU_AXIS_MAP_X]);
  out[1] = (int16_t)(IMU_AXIS_SIGN_Y * in[IMU_AXIS_MAP_Y]);
  out[2] = (int16_t)(IMU_AXIS_SIGN_Z * in[IMU_AXIS_MAP_Z]);
}

/* ==========================================================================
 * 阻塞式寄存器访问（只用于初始化）
 * ========================================================================== */

static uint8_t reg_write(uint8_t reg, uint8_t val)
{
  return (HAL_I2C_Mem_Write(&hi2c1, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                            &val, 1u, ICM_I2C_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
}

static uint8_t reg_read(uint8_t reg, uint8_t *val)
{
  return (HAL_I2C_Mem_Read(&hi2c1, IMU_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                           val, 1u, ICM_I2C_TIMEOUT_MS) == HAL_OK) ? 1u : 0u;
}

#if (IMU_ENABLE_MAG != 0)
/**
 * @brief 通过 IREG 间接寄存器机制访问非 Bank0 的寄存器（手册 §16.81~16.83）。
 * @note  流程：写 16 位地址到 IREG_ADDR_15_8 / IREG_ADDR_7_0，再读写 IREG_DATA，
 *        然后轮询 REG_MISC2 的 IREG_DONE 位等操作完成。
 *        9 轴模式配置 AUX I2C 主机要用到这个，6 轴用不上。
 */
static uint8_t ireg_write(uint16_t addr, uint8_t val)
{
  uint8_t misc2 = 0u;
  uint32_t guard = 0u;

  if (reg_write(ICM_REG_IREG_ADDR_15_8, (uint8_t)(addr >> 8)) == 0u) { return 0u; }
  if (reg_write(ICM_REG_IREG_ADDR_7_0,  (uint8_t)(addr & 0xFFu)) == 0u) { return 0u; }
  if (reg_write(ICM_REG_IREG_DATA, val) == 0u) { return 0u; }

  /* 等 IREG_DONE = 1。手册没给具体时间，给 1000 次读作为上限保护。 */
  for (guard = 0u; guard < 1000u; guard++)
  {
    if (reg_read(ICM_REG_MISC2, &misc2) == 0u) { return 0u; }
    if ((misc2 & ICM_IREG_DONE) != 0u) { return 1u; }
  }
  return 0u;
}
#endif /* IMU_ENABLE_MAG */

/* ==========================================================================
 * 磁力计桥接（第二阶段，尚未实测）
 * ========================================================================== */

#if (IMU_ENABLE_MAG != 0)
/**
 * @brief 配置 ICM 的 AUX I2C 主机去读 MMC5603，并把结果作为 ES0 塞进 FIFO。
 *
 * @warning 这一段还没有在真硬件上验证过。已经确认的前提条件（手册 §4.11/§4.12）：
 *   1. AUX1 接口**默认是 SPI3W / I3C 模式**，必须先通过
 *      IOC_PAD_SCENARIO_AUX_OVRD 把它切成 I2C master 模式；
 *      当前配置可以读只读寄存器 IOC_PAD_SCENARIO 确认。
 *   2. 桥接由片内 eDMP 完成："I2C 主机读回数据后由内部处理器 eDMP 重排，
 *      再连同内部传感器数据一起送入 FIFO"。所以必须使能 eDMP
 *      （EDMP_APEX_EN1 的 EDMP_ENABLE 位）并设置 DMP_EXT_SEN_ODR_CFG 的
 *      EXT_SENSOR_EN 与 EXT_ODR。
 *   3. 每个 ODR 事件最多执行 4 条 I2C 事务，读写皆可、可突发。速率 ≤400kHz。
 *   4. I2CM_COMMAND_0..3 / I2CM_DEV_PROFILE0..3 / I2CM_CONTROL 这些寄存器
 *      在 Bank IPREG_TOP1（手册 §19），必须走上面的 ireg_write() 访问。
 *   5. MMC5603 侧要先配成连续测量模式并打开 **Auto_SR_en**（自动 SET/RESET，
 *      消除热漂移偏置），这样桥接只需要"读"，不用在 4 条事务里挤 SET/RESET 序列。
 *      而 MMC5603 的这几个配置字本身也只能通过 I2CM 的写事务下发。
 *   6. FIFO 包会从 16 字节变成 32 字节，**Stream 模式不支持 32 字节包**
 *      （会退回 Bypass），所以必须改用 Stop-on-full，这一点已在 IMU_Init() 处理。
 *
 * @return 1 = 成功，0 = 失败
 */
static uint8_t IMU_MagBridgeInit(void)
{
  /* TODO(第二阶段)：按上面 6 条实现 I2CM 配置。
     在没实现之前直接返回失败，避免"以为开了其实没开"。 */
  return 0u;
}
#endif /* IMU_ENABLE_MAG */

/* ==========================================================================
 * 初始化
 * ========================================================================== */

uint8_t IMU_Init(void)
{
  uint8_t who = 0u;
  uint8_t fifo_mode;
  uint8_t fifo_cfg3;
  uint8_t fifo_cfg4;

  memset(&g_imu, 0, sizeof(g_imu));
  g_imu.q[0] = 1.0f;      /* 单位四元数，标定完成前一直报这个 */

  s_state        = IMU_ST_IDLE;
  s_sync_pending = 0u;
  s_dma_done     = 0u;
  s_dma_error    = 0u;
  s_prev_tmst_ok = 0u;
  s_cal_cnt      = 0u;
  memset(s_e_int, 0, sizeof(s_e_int));
  memset(s_cal_gyro_sum, 0, sizeof(s_cal_gyro_sum));
  memset(s_cal_acc_sum, 0, sizeof(s_cal_acc_sum));

  /* FSYNC 引脚先拉低 */
  HAL_GPIO_WritePin(IMU_FSYNC_GPIO_Port, IMU_FSYNC_Pin, GPIO_PIN_RESET);

  /* ---- 1. 软复位 ----
     REG_MISC2 的 SOFT_RST 位写 1 触发，完成后硬件自清。
     手册给的"上电到可以读写寄存器"是 1ms，复位后同样留足余量。 */
  if (reg_write(ICM_REG_MISC2, ICM_SOFT_RST) == 0u) { return 0u; }
  HAL_Delay(10u);

  /* ---- 2. 认芯片 ----
     WHO_AM_I 应为 0xE9。读不对就别往下走了，后面全是白费。 */
  if (reg_read(ICM_REG_WHO_AM_I, &who) == 0u) { return 0u; }
  if (who != ICM_WHOAMI_VALUE)
  {
    g_imu.present = 0u;
    return 0u;
  }
  g_imu.present = 1u;

  /* ---- 3. FIFO 先置 Bypass ----
     FIFO_DEPTH 只能在 Bypass 模式下改（手册 §16.28）。 */
  if (reg_write(ICM_REG_FIFO_CONFIG0,
                ICM_FIFO_MODE_BYPASS | ICM_FIFO_DEPTH_2K) == 0u) { return 0u; }
  /* FIFO_IF_EN 也先关掉：手册要求"FIFO 处于 Bypass 时 FIFO_IF_EN 应为 0"，
     否则会白耗电。 */
  if (reg_write(ICM_REG_FIFO_CONFIG3, 0x00u) == 0u) { return 0u; }

  /* ---- 4. 时间戳 ----
     TMST_RESOL = 0 → 1us 分辨率；TMST_DELTA_EN = 0 → 时间戳是绝对值
     （16 位，1us 下每 65.536ms 回绕；相邻样本间隔 2.5ms，uint16 减法够用）。
     WOM 相关位保持 0。 */
  if (reg_write(ICM_REG_TMST_WOM_CONFIG, 0x00u) == 0u) { return 0u; }

  /* ---- 5. FSYNC ----
     AP_FSYNC_SEL 必须非 0 才算开启 FSYNC 标记功能，选 1（标记打到温度 LSB）
     是为了不污染 accel/gyro 数据。
     AP_FSYNC_FLAG_CLEAR_SEL = 0：标记随传感器寄存器更新自动清除，走 FIFO
     路径时不需要我们手动读寄存器去清。 */
#if (IMU_ENABLE_FSYNC != 0)
  if (reg_write(ICM_REG_FSYNC_CONFIG0, ICM_AP_FSYNC_SEL_TEMP_LSB) == 0u) { return 0u; }
#else
  if (reg_write(ICM_REG_FSYNC_CONFIG0, 0x00u) == 0u) { return 0u; }
#endif

  /* ---- 6. 量程与采样率 ----
     ACCEL_CONFIG0 = FS[6:4] | ODR[3:0]；GYRO_CONFIG0 = FS[7:4] | ODR[3:0]。
     注意两个寄存器的 FS 字段位置不一样（accel 是 3 位，gyro 是 4 位）。 */
  if (reg_write(ICM_REG_ACCEL_CONFIG0,
                (uint8_t)((IMU_ACCEL_FS_SEL << 4) | IMU_ODR_SEL)) == 0u) { return 0u; }
  if (reg_write(ICM_REG_GYRO_CONFIG0,
                (uint8_t)((IMU_GYRO_FS_SEL  << 4) | IMU_ODR_SEL)) == 0u) { return 0u; }

  /* ---- 7. FIFO 内容选择 ----
     FIFO_CONFIG4：使能时间戳/FSYNC 字段；不开压缩（压缩要主机侧解压，没必要）。
     FIFO_CONFIG3：使能 accel + gyro（+ ES0），FIFO_IF_EN 稍后单独置位。 */
  fifo_cfg4 = ICM_FIFO_TMST_FSYNC_EN;
  fifo_cfg3 = ICM_FIFO_ACCEL_EN | ICM_FIFO_GYRO_EN;

#if (IMU_ENABLE_MAG != 0)
  /* MMC5603 用 18 位输出时 ES0 占 9 字节 */
  fifo_cfg4 |= ICM_FIFO_ES0_9B;
  fifo_cfg3 |= ICM_FIFO_ES0_EN;
#endif

  if (reg_write(ICM_REG_FIFO_CONFIG4, fifo_cfg4) == 0u) { return 0u; }
  if (reg_write(ICM_REG_FIFO_CONFIG3, fifo_cfg3) == 0u) { return 0u; }

  /* 水位中断我们不用（不接 INT 脚，靠 60Hz 主动读），阈值写 0 即可。 */
  if (reg_write(ICM_REG_FIFO_CONFIG1_0, 0x00u) == 0u) { return 0u; }
  if (reg_write(ICM_REG_FIFO_CONFIG1_1, 0x00u) == 0u) { return 0u; }

#if (IMU_ENABLE_MAG != 0)
  /* ---- 7b. 磁力计桥接（第二阶段）---- */
  if (IMU_MagBridgeInit() == 0u)
  {
    return 0u;   /* 明确失败，不要假装 9 轴能用 */
  }
#endif

  /* ---- 8. 开传感器 ----
     GYRO_MODE = ACCEL_MODE = 11（Low Noise）。
     手册 Table 1：陀螺从使能到 drive ready 需要 35ms，这段时间的样本不可信。 */
  if (reg_write(ICM_REG_PWR_MGMT0,
                ICM_GYRO_MODE_LN | ICM_ACCEL_MODE_LN) == 0u) { return 0u; }
  HAL_Delay(60u);   /* 35ms 是典型值，留够余量 */

  /* ---- 9. 打开 FIFO ----
     模式选择有个坑（手册 §16.28）：Stream 模式**只支持 8/16/20 字节包**，
     32 字节包（我们开 ES0 磁力计之后就是 32）选 Stream 会让 FIFO 退回 Bypass，
     等于白配。所以 9 轴必须用 Stop-on-full。
     6 轴用 Stream 更好：万一某个周期主循环卡住没读，新数据会覆盖旧的，
     不会因为 FIFO 满而停止收集。 */
#if (IMU_ENABLE_MAG != 0)
  fifo_mode = ICM_FIFO_MODE_STOP_ON_FULL;
#else
  fifo_mode = ICM_FIFO_MODE_STREAM;
#endif
  if (reg_write(ICM_REG_FIFO_CONFIG0, (uint8_t)(fifo_mode | ICM_FIFO_DEPTH_2K)) == 0u)
  {
    return 0u;
  }

  /* 手册规定的使能顺序：先 1) 开 FIFO，再 2) 开"传感器寄存器→FIFO"接口。 */
  if (reg_write(ICM_REG_FIFO_CONFIG3,
                (uint8_t)(fifo_cfg3 | ICM_FIFO_IF_EN)) == 0u) { return 0u; }

  /* ---- 10. 冲掉启动阶段的脏数据 ----
     前面 60ms 里陀螺还在起振，那些样本会把姿态积歪，必须丢掉。 */
  if (reg_write(ICM_REG_FIFO_CONFIG2, ICM_FIFO_FLUSH) == 0u) { return 0u; }

  return 1u;
}

/* ==========================================================================
 * 帧同步钩子
 * ========================================================================== */

void IMU_OnFrameSync(void)
{
#if (IMU_ENABLE_FSYNC != 0)
  /* 拉高 FSYNC，让 ICM 记录"这个时刻距离最近的采样点差多少"。
     不在这里等脉冲宽度 —— 读完 FIFO 后在 IMU_Poll() 里拉低，
     宽度自然是 3~8ms，远超识别门限，而且中断里一条 GPIO 写就返回。 */
  HAL_GPIO_WritePin(IMU_FSYNC_GPIO_Port, IMU_FSYNC_Pin, GPIO_PIN_SET);
#endif

  /* 上一周期的批量读还没做完就又来了一个 XVS —— 说明主循环被拖住了。
     记账，然后照常置标志：状态机会在下次空闲时继续。 */
  if (s_state != IMU_ST_IDLE)
  {
    g_imu.timeout++;
  }

  s_sync_pending = 1u;
}

/* ==========================================================================
 * 姿态解算
 * ========================================================================== */

/**
 * @brief 用重力方向直接给出初始姿态（yaw = 0）。
 * @param an  归一化后的加速度（机体系，静止时指向天）
 * @note  取"把测得的重力方向旋到世界 Z 轴"的最短旋转四元数：
 *            q = normalize([1 + v·u, v×u])，其中 u = [0,0,1]
 *        展开后 v·u = vz，v×u = (vy, -vx, 0)。
 *        最短旋转不含绕竖直轴的扭转，所以天然就是 yaw = 0 —— 正好符合
 *        CLAUDE.md §3 的"yaw 由视觉锚定，不依赖磁罗盘"。
 *        全程只用乘加和一次平方根倒数，不需要 atan2/asin。
 */
static void quat_from_gravity(const float an[3])
{
  const float w  = 1.0f + an[2];
  float       n;

  if (w < 1.0e-4f)
  {
    /* 传感器几乎完全朝下，最短旋转退化（绕哪个水平轴都行）。
       取绕 X 轴 180 度作为确定性的选择。 */
    g_imu.q[0] = 0.0f;
    g_imu.q[1] = 1.0f;
    g_imu.q[2] = 0.0f;
    g_imu.q[3] = 0.0f;
    return;
  }

  g_imu.q[0] =  w;
  g_imu.q[1] =  an[1];
  g_imu.q[2] = -an[0];
  g_imu.q[3] =  0.0f;

  n = inv_sqrt((g_imu.q[0] * g_imu.q[0]) + (g_imu.q[1] * g_imu.q[1]) +
               (g_imu.q[2] * g_imu.q[2]) + (g_imu.q[3] * g_imu.q[3]));
  g_imu.q[0] *= n;
  g_imu.q[1] *= n;
  g_imu.q[2] *= n;
  g_imu.q[3] *= n;
}

/**
 * @brief Mahony 互补滤波，一个样本一步。
 * @param g   角速度（rad/s，已扣零偏）
 * @param a   加速度（任意单位，内部归一化）
 * @param dt  本样本与上一样本的时间间隔（秒）
 *
 * @note  原理：由当前四元数推算重力在机体系的方向 v，与实测加速度 a 做叉积
 *        得到姿态误差 e；把 Kp*e + Ki*∫e 加到角速度上再积分四元数。
 *        Ki 项负责吃掉陀螺零偏的残余漂移。
 *        自由落体或强烈加速时 |a| 不再代表重力，此时跳过修正、纯陀螺积分。
 */
static void mahony_update(const float g[3], const float a[3], float dt)
{
  float q0 = g_imu.q[0], q1 = g_imu.q[1], q2 = g_imu.q[2], q3 = g_imu.q[3];
  float gx = g[0], gy = g[1], gz = g[2];
  float norm_sq;
  float qd0, qd1, qd2, qd3, n;

  norm_sq = (a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]);

  if (norm_sq > 1.0e-6f)
  {
    const float rn = inv_sqrt(norm_sq);
    const float ax = a[0] * rn;
    const float ay = a[1] * rn;
    const float az = a[2] * rn;

    /* 由四元数推算的重力方向（机体系） */
    const float vx = 2.0f * ((q1 * q3) - (q0 * q2));
    const float vy = 2.0f * ((q0 * q1) + (q2 * q3));
    const float vz = (q0 * q0) - (q1 * q1) - (q2 * q2) + (q3 * q3);

    /* 误差 = 实测方向 × 推算方向 */
    const float ex = (ay * vz) - (az * vy);
    const float ey = (az * vx) - (ax * vz);
    const float ez = (ax * vy) - (ay * vx);

    s_e_int[0] += ex * dt;
    s_e_int[1] += ey * dt;
    s_e_int[2] += ez * dt;

    gx += (IMU_MAHONY_KP * ex) + (IMU_MAHONY_KI * s_e_int[0]);
    gy += (IMU_MAHONY_KP * ey) + (IMU_MAHONY_KI * s_e_int[1]);
    gz += (IMU_MAHONY_KP * ez) + (IMU_MAHONY_KI * s_e_int[2]);
  }

  /* 四元数微分并积分。四个分量必须用同一组旧值算，所以先全部算完再赋值。 */
  qd0 = 0.5f * ((-q1 * gx) - (q2 * gy) - (q3 * gz));
  qd1 = 0.5f * (( q0 * gx) + (q2 * gz) - (q3 * gy));
  qd2 = 0.5f * (( q0 * gy) - (q1 * gz) + (q3 * gx));
  qd3 = 0.5f * (( q0 * gz) + (q1 * gy) - (q2 * gx));

  q0 += qd0 * dt;
  q1 += qd1 * dt;
  q2 += qd2 * dt;
  q3 += qd3 * dt;

  n = inv_sqrt((q0 * q0) + (q1 * q1) + (q2 * q2) + (q3 * q3));
  g_imu.q[0] = q0 * n;
  g_imu.q[1] = q1 * n;
  g_imu.q[2] = q2 * n;
  g_imu.q[3] = q3 * n;
}

#if (IMU_COMPUTE_EULER_DEBUG != 0)
/** 仅供 Live Expressions 观察，不进帧。会跳变是欧拉角的固有问题，属正常。 */
static void update_euler_debug(void)
{
  const float q0 = g_imu.q[0], q1 = g_imu.q[1], q2 = g_imu.q[2], q3 = g_imu.q[3];
  float sinp;

  g_imu.euler_deg[0] = atan2f(2.0f * ((q0 * q1) + (q2 * q3)),
                              1.0f - (2.0f * ((q1 * q1) + (q2 * q2)))) * 57.2957795f;

  sinp = 2.0f * ((q0 * q2) - (q3 * q1));
  if (sinp >  1.0f) { sinp =  1.0f; }
  if (sinp < -1.0f) { sinp = -1.0f; }
  g_imu.euler_deg[1] = asinf(sinp) * 57.2957795f;

  g_imu.euler_deg[2] = atan2f(2.0f * ((q0 * q3) + (q1 * q2)),
                              1.0f - (2.0f * ((q2 * q2) + (q3 * q3)))) * 57.2957795f;
}
#endif

/* ==========================================================================
 * 包解析
 * ========================================================================== */

/**
 * @brief 解析一个 FIFO 包。
 * @return 1 = 包头合法且已填好 out，0 = 包头不认识
 */
static uint8_t parse_packet(const uint8_t *p, IMU_Sample_t *out)
{
  int16_t raw[3];

  memset(out, 0, sizeof(*out));

  if ((p[0] & ICM_HDR_CHECK_MASK) != ICM_HDR_EXPECTED)
  {
    return 0u;
  }

  out->fsync_tag = ((p[0] & ICM_HDR_FSYNC_TAG_EN) != 0u) ? 1u : 0u;

#if (IMU_ENABLE_MAG != 0)
  /* 32 字节包：2 字节包头，第二字节带外部传感器有效位 */
  raw[0] = le16(&p[ICM_PKT9_OFF_ACCEL + 0]);
  raw[1] = le16(&p[ICM_PKT9_OFF_ACCEL + 2]);
  raw[2] = le16(&p[ICM_PKT9_OFF_ACCEL + 4]);
  axis_remap(raw, out->accel);

  raw[0] = le16(&p[ICM_PKT9_OFF_GYRO + 0]);
  raw[1] = le16(&p[ICM_PKT9_OFF_GYRO + 2]);
  raw[2] = le16(&p[ICM_PKT9_OFF_GYRO + 4]);
  axis_remap(raw, out->gyro);

  out->temp_raw = (int8_t)p[ICM_PKT9_OFF_TEMP];
  out->tmst     = leu16(&p[ICM_PKT9_OFF_TMST]);

  if ((p[1] & ICM_HDR2_ES0_VLD) != 0u)
  {
    /* MMC5603 输出是无符号、以 32768 为中点，转成有符号存放 */
    out->mag[0] = (int16_t)((int32_t)leu16(&p[ICM_PKT9_OFF_ES0 + 0]) - 32768);
    out->mag[1] = (int16_t)((int32_t)leu16(&p[ICM_PKT9_OFF_ES0 + 2]) - 32768);
    out->mag[2] = (int16_t)((int32_t)leu16(&p[ICM_PKT9_OFF_ES0 + 4]) - 32768);
    out->mag_valid = 1u;
  }
#else
  /* 16 字节包：1 字节包头 */
  raw[0] = le16(&p[ICM_PKT6_OFF_ACCEL + 0]);
  raw[1] = le16(&p[ICM_PKT6_OFF_ACCEL + 2]);
  raw[2] = le16(&p[ICM_PKT6_OFF_ACCEL + 4]);
  axis_remap(raw, out->accel);

  raw[0] = le16(&p[ICM_PKT6_OFF_GYRO + 0]);
  raw[1] = le16(&p[ICM_PKT6_OFF_GYRO + 2]);
  raw[2] = le16(&p[ICM_PKT6_OFF_GYRO + 4]);
  axis_remap(raw, out->gyro);

  out->temp_raw = (int8_t)p[ICM_PKT6_OFF_TEMP];
  out->tmst     = leu16(&p[ICM_PKT6_OFF_TMST]);
#endif

  return 1u;
}

/**
 * @brief 处理一个样本：标定阶段累加，标定完成后跑 Mahony。
 */
static void process_sample(const IMU_Sample_t *s)
{
  float a[3];
  float g_dps[3];
  float g_rad[3];
  float dt;
  uint32_t i;

  for (i = 0u; i < 3u; i++)
  {
    a[i]     = (float)s->accel[i] * IMU_ACCEL_G_PER_LSB;
    g_dps[i] = (float)s->gyro[i]  * IMU_GYRO_DPS_PER_LSB;
    g_imu.accel_last[i] = s->accel[i];
    g_imu.gyro_last[i]  = s->gyro[i];
  }

  /* 加速度模长（g）。静止时应 ≈1.0；明显偏离说明量程/字节序/接线有问题。
     这个数在字节序踩坑那次能一眼暴露问题，保留作常驻体检指标。 */
  {
    const float nsq = (a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]);
    g_imu.acc_norm_g = (nsq > 1.0e-12f) ? (nsq * inv_sqrt(nsq)) : 0.0f;
  }

  /* ---- 算 dt ----
     FSYNC 打过标记的那个包，时间戳字段装的是 FSYNC 延时而不是时间戳，
     不能拿来算间隔，这一步用标称值。 */
  if ((s->fsync_tag == 0u) && (s_prev_tmst_ok != 0u))
  {
    const uint16_t d = (uint16_t)(s->tmst - s_prev_tmst);   /* uint16 减法跨回绕正确 */
    dt = (float)d * 1.0e-6f;
    /* 异常值保护：正常应该就是 1/ODR，偏离太远说明中间丢过包或时间戳被占用 */
    if ((dt < (IMU_NOMINAL_DT_S * 0.5f)) || (dt > (IMU_NOMINAL_DT_S * 4.0f)))
    {
      dt = IMU_NOMINAL_DT_S;
    }
  }
  else
  {
    dt = IMU_NOMINAL_DT_S;
  }

  if (s->fsync_tag == 0u)
  {
    s_prev_tmst    = s->tmst;
    s_prev_tmst_ok = 1u;
  }
  else
  {
    /* 记下 FSYNC 边沿到该样本 ODR 时刻的延时，供 RV 侧做曝光时刻插值 */
    g_imu.last_fsync_delay_us = s->tmst;
    g_imu.fsync_tag_cnt++;
  }

  /* ---- 开机静止标定 ----
     CLAUDE.md §3：静止检测 → 估陀螺零偏 → 加速度定倾角 → yaw = 0。 */
  if (g_imu.calibrated == 0u)
  {
    const float gn_sq = (g_dps[0] * g_dps[0]) + (g_dps[1] * g_dps[1]) +
                        (g_dps[2] * g_dps[2]);
    const float an_sq = (a[0] * a[0]) + (a[1] * a[1]) + (a[2] * a[2]);
    const float lo    = (1.0f - IMU_CALIB_ACC_TOL_G) * (1.0f - IMU_CALIB_ACC_TOL_G);
    const float hi    = (1.0f + IMU_CALIB_ACC_TOL_G) * (1.0f + IMU_CALIB_ACC_TOL_G);

    if ((gn_sq > (IMU_CALIB_GYRO_MAX_DPS * IMU_CALIB_GYRO_MAX_DPS)) ||
        (an_sq < lo) || (an_sq > hi))
    {
      /* 动了，从头再来 */
      s_cal_cnt = 0u;
      memset(s_cal_gyro_sum, 0, sizeof(s_cal_gyro_sum));
      memset(s_cal_acc_sum, 0, sizeof(s_cal_acc_sum));
    }
    else
    {
      for (i = 0u; i < 3u; i++)
      {
        s_cal_gyro_sum[i] += g_dps[i];
        s_cal_acc_sum[i]  += a[i];
      }
      s_cal_cnt++;
    }

    g_imu.calib_progress = s_cal_cnt;

    if (s_cal_cnt >= IMU_CALIB_SAMPLES)
    {
      const float inv_n = 1.0f / (float)s_cal_cnt;
      float an[3];
      float rn;

      for (i = 0u; i < 3u; i++)
      {
        g_imu.gyro_bias_dps[i] = s_cal_gyro_sum[i] * inv_n;
        an[i]                  = s_cal_acc_sum[i]  * inv_n;
      }

      rn = inv_sqrt((an[0] * an[0]) + (an[1] * an[1]) + (an[2] * an[2]));
      an[0] *= rn; an[1] *= rn; an[2] *= rn;

      quat_from_gravity(an);
      memset(s_e_int, 0, sizeof(s_e_int));
      g_imu.calibrated = 1u;
    }

    return;   /* 标定期间不做姿态积分 */
  }

  /* ---- 正常滤波 ---- */
  for (i = 0u; i < 3u; i++)
  {
    g_rad[i] = (g_dps[i] - g_imu.gyro_bias_dps[i]) * 0.0174532925f;  /* dps → rad/s */
  }

  mahony_update(g_rad, a, dt);
}

/* ==========================================================================
 * 主循环状态机
 * ========================================================================== */

/**
 * @brief 完成中断迟迟不来时的脱困：复位 I2C1 外设并重新初始化。
 * @note  与 rv_link 里复位 SPI1 同一套路：FORCE_RESET 只清外设寄存器，不动 RCC
 *        时钟使能；把 State 置回 RESET 后 HAL_I2C_Init() 会重新走 MspInit
 *        （重配 GPIO、重挂 DMA），这些动作都是幂等的。
 *        局限：如果是从机在字节中间把 SDA 拉死，复位我们这边救不了总线 ——
 *        标准解法是把 SCL 切成 GPIO 手动打 9 个时钟。等 i2c_stuck 真在
 *        硬件上出现再加，现在不写用不到的代码。
 */
static void i2c_recover(void)
{
  if (hi2c1.hdmarx != NULL)
  {
    (void)HAL_DMA_Abort(hi2c1.hdmarx);   /* 先停 DMA，防止复位后它还挂着 */
  }

  __HAL_RCC_I2C1_FORCE_RESET();
  __HAL_RCC_I2C1_RELEASE_RESET();

  hi2c1.State = HAL_I2C_STATE_RESET;
  (void)HAL_I2C_Init(&hi2c1);

  s_dma_done  = 0u;
  s_dma_error = 0u;
  g_imu.i2c_stuck++;
}

/** 干完活（或放弃）之后统一收尾：写帧、标心跳、开闸、拉低 FSYNC。 */
static void burst_finish(uint8_t got_data)
{
#if (IMU_ENABLE_FSYNC != 0)
  /* FSYNC 脉冲到此结束，宽度约 3~8ms */
  HAL_GPIO_WritePin(IMU_FSYNC_GPIO_Port, IMU_FSYNC_Pin, GPIO_PIN_RESET);
#endif

  if ((got_data != 0u) && (RvLink_FramePending() != 0u))
  {
    uint16_t *frame = RvLink_GetSendBuffer();

    /* 原始六轴（本批最后一个样本，最贴近同步沿）无条件写；
       四元数只在标定完成后才有意义。 */
    GloveFrame_SetImuRaw(frame, g_imu.accel_last, g_imu.gyro_last);

    if (g_imu.calibrated != 0u)
    {
      int16_t q14[4];
      IMU_GetQuatQ14(q14);
      GloveFrame_SetQuat(frame, q14);
    }
    RvLink_MarkNodeAlive(GLOVE_HB_IMU_BIT);
  }

  /* 不管拿到数据没有都要开闸，否则 RvLink_Poll() 要白等一个 8ms 超时。
     没数据的情况由心跳位（保持 1 = 无数据）告诉 RV。 */
  s_state = IMU_ST_IDLE;
  RvLink_ProducerDone(RV_PRODUCER_IMU);
}

void IMU_Poll(void)
{
  if (g_imu.present == 0u)
  {
    return;   /* 芯片不在，IMU 也没被登记进闸门，什么都不用做 */
  }

  /* ---- I2C 出错：放弃本周期，开闸让帧照常发出去 ---- */
  if (s_dma_error != 0u)
  {
    s_dma_error = 0u;
    s_dma_done  = 0u;
    g_imu.i2c_error++;
    burst_finish(0u);
    return;
  }

  switch (s_state)
  {
    case IMU_ST_IDLE:
    {
      if (s_sync_pending == 0u)
      {
        break;
      }
      s_sync_pending = 0u;
      s_dma_done     = 0u;

      /* 先读 FIFO 里有几个包。
         注意 FIFO_COUNT 的单位是**包数**（手册 §16.18：
         "count indicates the number of packets available in FIFO"），
         不是字节数 —— 按字节算会读错长度。
         FIFO_COUNT_0(0x12) 是高字节、FIFO_COUNT_1(0x13) 是低字节，
         所以从 0x12 连读 2 字节得到的是大端。

         HAL_I2C_Mem_Read_DMA()：寄存器地址阶段走中断（I2C_XFER_TX_IT），
         数据阶段走 DMA1_Channel2（I2C1_RX，NVIC 优先级 2）。
         完成时 HAL 回调 HAL_I2C_MemRxCpltCallback()；
         出错走 HAL_I2C_ErrorCallback()。所以只需要 RX DMA，不需要 TX DMA。 */
      if (HAL_I2C_Mem_Read_DMA(&hi2c1, IMU_I2C_ADDR, ICM_REG_FIFO_COUNT_0,
                               I2C_MEMADD_SIZE_8BIT, s_count_buf, 2u) != HAL_OK)
      {
        g_imu.i2c_error++;
        burst_finish(0u);
        break;
      }
      s_wait_t0_ms = HAL_GetTick();
      s_state = IMU_ST_COUNT_WAIT;
      break;
    }

    case IMU_ST_COUNT_WAIT:
    {
      uint16_t packets;

      if (s_dma_done == 0u)
      {
        /* 完成中断超时不来 → 复位外设脱困，放弃本周期 */
        if ((uint32_t)(HAL_GetTick() - s_wait_t0_ms) > ICM_XFER_STUCK_MS)
        {
          i2c_recover();
          burst_finish(0u);
        }
        break;
      }
      s_dma_done = 0u;

      /* FIFO_COUNT 按小端解析：0x12 是低字节。
         依据手册 §14：芯片默认 Little Endian，FIFO 计数也在其列；
         §16.18 的 [15:8]/[7:0] 描述对应的是置位 SREG_DATA_ENDIAN_SEL 之后
         的大端模式。按大端解析时真实包数 2/7 会读成 512/1792，
         远超 2KB FIFO 的物理上限（128 包）。 */
      packets = (uint16_t)((uint16_t)s_count_buf[0] |
                           ((uint16_t)s_count_buf[1] << 8));

      g_imu.last_fifo_bytes = (uint16_t)(packets * IMU_FIFO_PACKET_BYTES);
      if (g_imu.last_fifo_bytes > g_imu.fifo_watermark)
      {
        g_imu.fifo_watermark = g_imu.last_fifo_bytes;
      }

      if (packets == 0u)
      {
        /* FIFO 空。开机头几个周期或 ODR 配错时会这样。 */
        s_read_packets = 0u;
        burst_finish(0u);
        break;
      }

      if (packets > (2048u / IMU_FIFO_PACKET_BYTES))
      {
        /* 超过 2KB FIFO 的物理上限 → 读回来的计数本身就是坏的
           （字节序、总线错位等）。别拿它当长度用，冲掉 FIFO 重来。 */
        g_imu.bad_header++;
        (void)reg_write(ICM_REG_FIFO_CONFIG2, ICM_FIFO_FLUSH);
        s_prev_tmst_ok = 0u;
        burst_finish(0u);
        break;
      }

      if (packets > IMU_FIFO_MAX_PACKETS)
      {
        /* 缓冲装不下，这次先读满，剩下的下个周期再读。
           truncated 一直涨说明主循环跟不上或 ODR 设得太高。 */
        packets = IMU_FIFO_MAX_PACKETS;
        g_imu.truncated++;
      }

      s_read_packets = (uint8_t)packets;
      s_read_bytes   = (uint16_t)(packets * IMU_FIFO_PACKET_BYTES);

      /* 从 FIFO_DATA 连续读 s_read_bytes 个字节。
         FIFO_DATA 是一个数据端口，连读会自动往后吐，地址不递增。 */
      if (HAL_I2C_Mem_Read_DMA(&hi2c1, IMU_I2C_ADDR, ICM_REG_FIFO_DATA,
                               I2C_MEMADD_SIZE_8BIT, s_fifo_buf,
                               s_read_bytes) != HAL_OK)
      {
        g_imu.i2c_error++;
        burst_finish(0u);
        break;
      }
      s_wait_t0_ms = HAL_GetTick();
      s_state = IMU_ST_DATA_WAIT;
      break;
    }

    case IMU_ST_DATA_WAIT:
    {
      uint8_t  i;
      uint8_t  ok_cnt = 0u;

      if (s_dma_done == 0u)
      {
        /* 同 COUNT_WAIT：完成中断超时不来就复位脱困 */
        if ((uint32_t)(HAL_GetTick() - s_wait_t0_ms) > ICM_XFER_STUCK_MS)
        {
          i2c_recover();
          burst_finish(0u);
        }
        break;
      }
      s_dma_done = 0u;

      for (i = 0u; i < s_read_packets; i++)
      {
        IMU_Sample_t smp;

        if (parse_packet(&s_fifo_buf[i * IMU_FIFO_PACKET_BYTES], &smp) == 0u)
        {
          /* 包头不认识 → FIFO 读写指针可能错位了。
             冲掉 FIFO 重新开始，比继续拿错数据去积分要好。 */
          g_imu.bad_header++;
          (void)reg_write(ICM_REG_FIFO_CONFIG2, ICM_FIFO_FLUSH);
          s_prev_tmst_ok = 0u;
          break;
        }

        process_sample(&smp);
        ok_cnt++;
      }

      g_imu.samples += ok_cnt;
      g_imu.last_burst_samples = ok_cnt;
      g_imu.bursts++;

#if (IMU_COMPUTE_EULER_DEBUG != 0)
      update_euler_debug();
#endif

      burst_finish((ok_cnt > 0u) ? 1u : 0u);
      break;
    }

    default:
      s_state = IMU_ST_IDLE;
      break;
  }
}

void IMU_GetQuatQ14(int16_t q_out[4])
{
  uint32_t i;

  if (g_imu.calibrated == 0u)
  {
    q_out[GLOVE_QUAT_W] = (int16_t)GLOVE_QUAT_ONE;
    q_out[GLOVE_QUAT_X] = 0;
    q_out[GLOVE_QUAT_Y] = 0;
    q_out[GLOVE_QUAT_Z] = 0;
    return;
  }

  for (i = 0u; i < 4u; i++)
  {
    float v = g_imu.q[i] * (float)GLOVE_QUAT_ONE;

    /* 单位四元数每个分量都在 [-1,1]，理论上不会溢出 int16，
       但滤波器万一发散就会，钳一下更安全。 */
    if (v >  32767.0f) { v =  32767.0f; }
    if (v < -32768.0f) { v = -32768.0f; }

    q_out[i] = (int16_t)v;
  }
}

/* ==========================================================================
 * HAL 回调（弱符号覆盖）
 * ========================================================================== */

/**
 * @brief I2C 存储器读（DMA）完成。
 * @note  完整调用链（注意有两个中断都必须在 NVIC 里使能）：
 *          1. 寄存器地址阶段：I2C1 的 TXIS 事件 → I2C1_EV_IRQHandler（优先级 2）
 *             → HAL 把寄存器地址写进 TXDR（这一步不走 DMA！）
 *          2. 数据阶段：DMA1_Channel2（I2C1_RX）搬运，传完 →
 *             DMA1_Channel2_IRQHandler → HAL 内部 I2C_DMAMasterReceiveCplt
 *          3. 收尾：I2C1 的 TC 事件 → I2C1_EV_IRQHandler 发 STOP → 本回调
 *        所以 **NVIC 里必须开 I2C1_EV 和 I2C1_ER**，只开 DMA 通道中断的话
 *        流程会卡死在第 1 步（现象：present=1 但 last_burst_samples 恒 0，
 *        producer_timeout 每帧 +1）。
 * @note  这是 HAL 的公共弱符号。将来 TMP1075 也挂在 I2C1 上时，
 *        必须加一个"当前谁占用总线"的标志来区分，不能只看 Instance。
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1)
  {
    return;
  }
  s_dma_done = 1u;
}

/**
 * @brief I2C 出错。
 * @note  典型原因：从机没应答（地址错/没焊上）、总线仲裁丢失、上拉太弱导致
 *        上升沿超时。处理只做记账 + 置标志，恢复在 IMU_Poll() 里做。
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance != I2C1)
  {
    return;
  }
  s_dma_error = 1u;
}
