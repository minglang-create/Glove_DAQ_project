/**
  ******************************************************************************
  * @file    glove_frame.c
  * @brief   SPI 上行帧组装 + CRC-16/ARC + 第一步的假数据
  ******************************************************************************
  */

#include "glove_frame.h"
#include <string.h>

/* ==========================================================================
 * CRC-16/ARC（反射式，查表实现）
 * ========================================================================== */

/**
 * 256 项查表。反射式 CRC-16 的字节更新式：
 *     crc = (crc >> 8) ^ table[(crc ^ byte) & 0xFF]
 * 逐位算需要 8 次迭代/字节，整帧约 500us；查表约 5 周期/字节，整帧约 83us。
 */
static uint16_t s_crc_tab[256];
static uint8_t  s_crc_tab_ready = 0u;

void GloveFrame_CrcInit(void)
{
  for (uint32_t i = 0u; i < 256u; i++)
  {
    uint16_t c = (uint16_t)i;
    for (uint8_t b = 0u; b < 8u; b++)
    {
      /* 反射式：从最低位往外移，反馈多项式用 0xA001（= 0x8005 位反转） */
      c = (uint16_t)((c & 1u) ? ((c >> 1) ^ GLOVE_CRC16_POLY_REFLECTED) : (c >> 1));
    }
    s_crc_tab[i] = c;
  }
  s_crc_tab_ready = 1u;
}

/** 单字节更新 */
static inline uint16_t crc_step(uint16_t crc, uint8_t data)
{
  return (uint16_t)((crc >> 8) ^ s_crc_tab[(uint8_t)(crc ^ data)]);
}

uint8_t GloveFrame_CrcSelfTest(void)
{
  static const uint8_t vec[9] = { '1','2','3','4','5','6','7','8','9' };
  uint16_t crc = GLOVE_CRC16_INIT;

  if (s_crc_tab_ready == 0u)
  {
    return 0u;
  }

  for (uint32_t i = 0u; i < sizeof(vec); i++)
  {
    crc = crc_step(crc, vec[i]);
  }

  return (uint8_t)(crc == GLOVE_CRC16_CHECK_VALUE);
}

uint16_t GloveFrame_Crc16(const uint16_t *words, uint32_t word_cnt)
{
  uint16_t crc = GLOVE_CRC16_INIT;

  for (uint32_t i = 0u; i < word_cnt; i++)
  {
    const uint16_t w = words[i];
    /* 线上字节序：高字节先，低字节后。
       绝对不能写成 memcpy 后按字节遍历 —— 小端 CPU 会反过来。 */
    crc = crc_step(crc, (uint8_t)(w >> 8));
    crc = crc_step(crc, (uint8_t)(w & 0xFFu));
  }

  return crc;
}

/* ==========================================================================
 * 帧组装
 * ========================================================================== */

void GloveFrame_ClearForCollect(uint16_t *f)
{
  /* 整帧清 0（帧头/长度/周期/心跳/CRC 稍后由 Finalize 覆盖） */
  memset(f, 0, GLOVE_FRAME_BYTES);

  /* 关节默认"无效"：整个 32 位全 1。
     这样 RV 即便忽略心跳位图，也能一眼看出这个关节没数据。 */
  for (uint32_t i = 0u; i < GLOVE_JOINT_CNT; i++)
  {
    const uint32_t off = GLOVE_JOINT_WORD_OFF(i);
    f[off]     = (uint16_t)(GLOVE_JOINT_INVALID >> 16);
    f[off + 1] = (uint16_t)(GLOVE_JOINT_INVALID & 0xFFFFu);
  }

  /* 四元数默认单位姿态 */
  f[GLOVE_OFF_QUAT + GLOVE_QUAT_W] = (uint16_t)GLOVE_QUAT_ONE;
  f[GLOVE_OFF_QUAT + GLOVE_QUAT_X] = 0u;
  f[GLOVE_OFF_QUAT + GLOVE_QUAT_Y] = 0u;
  f[GLOVE_OFF_QUAT + GLOVE_QUAT_Z] = 0u;
}

void GloveFrame_Finalize(uint16_t *f, uint32_t cycle, uint32_t heartbeat)
{
  f[GLOVE_OFF_MAGIC]        = GLOVE_MAGIC;
  f[GLOVE_OFF_LENGTH]       = (uint16_t)GLOVE_LENGTH_VALUE;

  f[GLOVE_OFF_CYCLE]        = (uint16_t)(cycle >> 16);          /* 高字在前 */
  f[GLOVE_OFF_CYCLE + 1]    = (uint16_t)(cycle & 0xFFFFu);

  f[GLOVE_OFF_HEARTBEAT]    = (uint16_t)(heartbeat >> 16);      /* 高字在前 */
  f[GLOVE_OFF_HEARTBEAT + 1]= (uint16_t)(heartbeat & 0xFFFFu);

  /* CRC 覆盖 [0, GLOVE_OFF_CRC)，即除自己以外的全部 1335 个字 */
  f[GLOVE_OFF_CRC]          = GloveFrame_Crc16(f, GLOVE_OFF_CRC);
}

/* ==========================================================================
 * 字段写入
 * ========================================================================== */

void GloveFrame_SetJoint(uint16_t *f, uint32_t idx, uint32_t raw32)
{
  if (idx >= GLOVE_JOINT_CNT)
  {
    return;
  }

  const uint32_t off = GLOVE_JOINT_WORD_OFF(idx);
  f[off]     = (uint16_t)(raw32 >> 16);        /* 高半字在低下标 */
  f[off + 1] = (uint16_t)(raw32 & 0xFFFFu);
}

void GloveFrame_SetTactileStatus(uint16_t *f, uint32_t sensor, uint16_t status)
{
  if (sensor >= GLOVE_TACTILE_CNT)
  {
    return;
  }
  f[GLOVE_OFF_TACTILE_STAT + sensor] = status;
}

void GloveFrame_SetTactileArray(uint16_t *f, uint32_t sensor, const uint16_t *cells)
{
  if ((sensor >= GLOVE_TACTILE_CNT) || (cells == NULL))
  {
    return;
  }
  memcpy(&f[GLOVE_TACTILE_WORD_OFF(sensor)], cells,
         GLOVE_TACTILE_CELLS * sizeof(uint16_t));
}

void GloveFrame_SetQuat(uint16_t *f, const int16_t q[4])
{
  f[GLOVE_OFF_QUAT + GLOVE_QUAT_W] = (uint16_t)q[GLOVE_QUAT_W];
  f[GLOVE_OFF_QUAT + GLOVE_QUAT_X] = (uint16_t)q[GLOVE_QUAT_X];
  f[GLOVE_OFF_QUAT + GLOVE_QUAT_Y] = (uint16_t)q[GLOVE_QUAT_Y];
  f[GLOVE_OFF_QUAT + GLOVE_QUAT_Z] = (uint16_t)q[GLOVE_QUAT_Z];
}

void GloveFrame_SetImuRaw(uint16_t *f, const int16_t accel[3], const int16_t gyro[3])
{
  f[GLOVE_OFF_IMU_RAW + 0u] = (uint16_t)accel[0];
  f[GLOVE_OFF_IMU_RAW + 1u] = (uint16_t)accel[1];
  f[GLOVE_OFF_IMU_RAW + 2u] = (uint16_t)accel[2];
  f[GLOVE_OFF_IMU_RAW + 3u] = (uint16_t)gyro[0];
  f[GLOVE_OFF_IMU_RAW + 4u] = (uint16_t)gyro[1];
  f[GLOVE_OFF_IMU_RAW + 5u] = (uint16_t)gyro[2];
}

void GloveFrame_SetMag(uint16_t *f, const int16_t mag[3])
{
  f[GLOVE_OFF_MAG + 0u] = (uint16_t)mag[0];
  f[GLOVE_OFF_MAG + 1u] = (uint16_t)mag[1];
  f[GLOVE_OFF_MAG + 2u] = (uint16_t)mag[2];
}

/* ==========================================================================
 * SPI 事务小包
 * ========================================================================== */

void GloveFrame_BuildSmallPkt(uint16_t pkt[GLOVE_SPKT_WORDS], uint16_t hdr,
                              uint16_t p0, uint16_t p1, uint16_t p2)
{
  pkt[GLOVE_SPKT_OFF_HDR] = hdr;
  pkt[GLOVE_SPKT_OFF_LEN] = (uint16_t)GLOVE_SPKT_LEN_VALUE;
  pkt[GLOVE_SPKT_OFF_P0]  = p0;
  pkt[GLOVE_SPKT_OFF_P1]  = p1;
  pkt[GLOVE_SPKT_OFF_P2]  = p2;
  pkt[GLOVE_SPKT_OFF_CRC] = GloveFrame_Crc16(pkt, GLOVE_SPKT_OFF_CRC);
}

uint16_t GloveFrame_ParseSmallPkt(const uint16_t pkt[GLOVE_SPKT_WORDS])
{
  if (pkt[GLOVE_SPKT_OFF_LEN] != (uint16_t)GLOVE_SPKT_LEN_VALUE)
  {
    return 0u;
  }
  if (GloveFrame_Crc16(pkt, GLOVE_SPKT_OFF_CRC) != pkt[GLOVE_SPKT_OFF_CRC])
  {
    return 0u;
  }
  return pkt[GLOVE_SPKT_OFF_HDR];
}

/* ==========================================================================
 * 第一步调试用假数据
 * ========================================================================== */

void GloveFrame_FillFake(uint16_t *f, uint32_t cycle, uint32_t *heartbeat)
{
  uint32_t hb = GLOVE_HB_ALL_MISSING;

  /* ---- 关节：值随关节序号和周期变化，RV 能看出"哪一路是哪一路" ---- */
  for (uint32_t i = 0u; i < GLOVE_JOINT_CNT; i++)
  {
    const uint32_t raw = (((cycle & 0x0FFFu) + (i << 12)) & GLOVE_JOINT_ANGLE_MASK);
    GloveFrame_SetJoint(f, i, raw);
  }

  /* ---- 触觉状态 ---- */
  for (uint32_t s = 0u; s < GLOVE_TACTILE_CNT; s++)
  {
    GloveFrame_SetTactileStatus(f, s, (uint16_t)(0xA500u | s));
  }

  /* ---- 触觉阵列：每个字 = 它自己的绝对字偏移 ----
     这是最好用的错位检测：RV 解出来第 k 个字如果不等于 k，就说明偏了。 */
  for (uint32_t w = GLOVE_OFF_TACTILE;
       w < GLOVE_OFF_TACTILE + GLOVE_TACTILE_CNT * GLOVE_TACTILE_CELLS; w++)
  {
    f[w] = (uint16_t)w;
  }

  /* ---- 六轴原始 + 磁力计：静态图案 IMU_RAW[i]=1000+i、MAG_RAW[i]=2000+i
          （2026-09-02 与 RV 验收文档 §6 统一） ---- */
  {
    const int16_t a[3] = { 1000, 1001, 1002 };
    const int16_t g[3] = { 1003, 1004, 1005 };
    const int16_t m[3] = { 2000, 2001, 2002 };
    GloveFrame_SetImuRaw(f, a, g);
    GloveFrame_SetMag(f, m);
  }

  /* ---- 四元数：w 固定，x 缓慢变化，方便 RV 看"活着没" ---- */
  {
    int16_t q[4];
    q[GLOVE_QUAT_W] = (int16_t)GLOVE_QUAT_ONE;
    q[GLOVE_QUAT_X] = (int16_t)((cycle & 0x03FFu) * 8u);   /* 0 .. 8184 循环 */
    q[GLOVE_QUAT_Y] = 0;   /* RV 验收文档 §6：y、z 恒 0 */
    q[GLOVE_QUAT_Z] = 0;
    GloveFrame_SetQuat(f, q);
  }

  /* ---- 心跳：关节 0..3（第一路）+ IMU 恒在线，再加一个轮转位 ---- */
  hb &= ~(1uL << GLOVE_HB_JOINT_BIT(0));
  hb &= ~(1uL << GLOVE_HB_JOINT_BIT(1));
  hb &= ~(1uL << GLOVE_HB_JOINT_BIT(2));
  hb &= ~(1uL << GLOVE_HB_JOINT_BIT(3));
  hb &= ~(1uL << GLOVE_HB_IMU_BIT);
  hb &= ~(1uL << GLOVE_HB_JOINT_BIT(4u + (cycle % 16u)));   /* 每周期换一位 */

  *heartbeat = hb;
}
