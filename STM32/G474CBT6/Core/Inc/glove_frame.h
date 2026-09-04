/**
  ******************************************************************************
  * @file    glove_frame.h
  * @brief   SPI 上行帧的组装、CRC 计算、以及第一步调试用的假数据生成
  *
  *          本模块只操作内存里的 uint16 数组，不碰任何外设，方便单独测试。
  *          帧布局定义在 glove_protocol.h。
  ******************************************************************************
  */

#ifndef GLOVE_FRAME_H
#define GLOVE_FRAME_H

#include <stdint.h>
#include "glove_protocol.h"

/* ==========================================================================
 * CRC-16/ARC
 * ========================================================================== */

/**
 * @brief  初始化 CRC 查表（256 项，占 512 字节 RAM）。
 * @note   必须在任何 GloveFrame_Crc16() 之前调用一次。
 *         用运行时建表而不是硬编码常量表，是为了避免手抄 256 个常量出错。
 */
void GloveFrame_CrcInit(void);

/**
 * @brief  自检：用标准向量 "123456789" 验证 CRC 实现是否为 CRC-16/ARC。
 * @return 1 = 通过（结果 == 0xBB3D），0 = 失败
 * @note   必须在 GloveFrame_CrcInit() 之后调用。失败说明实现或查表有问题，
 *         这时候去联调只会浪费时间，直接 Error_Handler() 卡住更好。
 */
uint8_t GloveFrame_CrcSelfTest(void);

/**
 * @brief  按线上字节序计算 CRC-16/ARC。
 * @param  words     uint16 数组起始地址
 * @param  word_cnt  参与计算的字数
 * @return CRC 值
 * @note   每个 uint16 先喂高字节、再喂低字节（= SPI 16bit MSB-first 的线上顺序）。
 *         RV 侧对收到的字节流直接顺序计算就能得到同样的值。
 */
uint16_t GloveFrame_Crc16(const uint16_t *words, uint32_t word_cnt);

/* ==========================================================================
 * 帧组装
 * ========================================================================== */

/**
 * @brief  把一块缓冲清成"全部节点无数据"的初始状态，准备接收本周期的采集。
 * @param  f  指向 GLOVE_FRAME_WORDS 个 uint16 的缓冲
 * @note   关节全部填 GLOVE_JOINT_INVALID(0xFFFFFFFF)，触觉和状态清 0，
 *         四元数填单位四元数。帧头/长度/周期/心跳/CRC 由 Finalize 负责，这里不管。
 *         耗时约 4us @160MHz，在中断里调用是可以接受的。
 */
void GloveFrame_ClearForCollect(uint16_t *f);

/**
 * @brief  封帧：填帧头、长度、周期号、心跳位图，最后算 CRC 写入尾字。
 * @param  f          已经装好传感器数据的缓冲
 * @param  cycle      本帧数据所属的 XVS 周期号
 * @param  heartbeat  心跳位图（bit=1 表示无数据）
 * @note   耗时以 CRC 为主，约 83us @160MHz。所以放主循环调用，不要放中断。
 */
void GloveFrame_Finalize(uint16_t *f, uint32_t cycle, uint32_t heartbeat);

/* ==========================================================================
 * 字段写入（给后续的 CAN / IMU 模块用）
 * ========================================================================== */

/**
 * @brief  写入一个关节角度。
 * @param  f     帧缓冲
 * @param  idx   关节序号 0..19（全局索引，不是 CAN 上的 NODE_ID）
 * @param  raw32 32 位原始值，bit[22:0] 是角度，bit[31:23] 预留
 * @note   自动拆成两个 uint16，高半字放低下标。
 */
void GloveFrame_SetJoint(uint16_t *f, uint32_t idx, uint32_t raw32);

/**
 * @brief  写入一个触觉节点的状态字。
 */
void GloveFrame_SetTactileStatus(uint16_t *f, uint32_t sensor, uint16_t status);

/**
 * @brief  写入一个触觉节点的整片 16x16 阵列。
 * @param  cells  256 个 uint16，行优先
 */
void GloveFrame_SetTactileArray(uint16_t *f, uint32_t sensor, const uint16_t *cells);

/**
 * @brief  写入四元数。
 * @param  q  4 个 int16，Q14，顺序 w, x, y, z
 */
void GloveFrame_SetQuat(uint16_t *f, const int16_t q[4]);

/**
 * @brief  写入六轴原始样本（本周期最后一个，最贴近同步沿）。
 * @param  accel/gyro  各 3 个 int16 原始 LSB
 */
void GloveFrame_SetImuRaw(uint16_t *f, const int16_t accel[3], const int16_t gyro[3]);

/**
 * @brief  写入磁力计原始三轴。桥接未启用时不调用（帧里保持 0 + 心跳位=1）。
 */
void GloveFrame_SetMag(uint16_t *f, const int16_t mag[3]);

/* ==========================================================================
 * SPI 事务小包（6 字，布局见 glove_protocol.h §9）
 * ========================================================================== */

/**
 * @brief  组一个小包：帧头 + 帧长(4) + 3 字载荷 + CRC。
 * @param  pkt  输出缓冲（GLOVE_SPKT_WORDS 字）
 * @param  hdr  GLOVE_PKT_xxx
 * @param  p0/p1/p2  载荷（无意义填 0xFFFF）
 */
void GloveFrame_BuildSmallPkt(uint16_t pkt[GLOVE_SPKT_WORDS], uint16_t hdr,
                              uint16_t p0, uint16_t p1, uint16_t p2);

/**
 * @brief  校验并解析一个入站小包。
 * @param  pkt  6 字原始内容
 * @return 帧头（合法时）；0 = 不是合法小包（CRC 错 / 帧长不对）
 * @note   全 0 / 全 0xFFFF 的 MOSI 填充在这里被自然拒绝。
 */
uint16_t GloveFrame_ParseSmallPkt(const uint16_t pkt[GLOVE_SPKT_WORDS]);

/* ==========================================================================
 * 第一步调试用：假数据
 * ========================================================================== */

/**
 * @brief  往帧里填可校验的假数据，用于在没有任何传感器的情况下打通 SPI 链路。
 * @param  f          帧缓冲（应先调用过 GloveFrame_ClearForCollect）
 * @param  cycle      当前周期号，用来让数据随周期变化
 * @param  heartbeat  [出] 生成的心跳位图
 *
 * @note   数据规律（RV 侧照这个校验）：
 *         - 关节 i : raw = ((cycle & 0xFFF) + (i << 12)) & 0x7FFFFF
 *                    → 每个关节值不同，且随周期递增
 *         - 触觉状态 i : 0xA500 | i
 *         - 触觉阵列   : 每个字 = 它自己在帧中的绝对字偏移
 *                    → 只要 RV 解出来的值等于下标，就证明没有错位/移位
 *         - 四元数     : w 恒 16384，x 随周期变化，y = z = 0
 *         - 六轴原始   : IMU_RAW[i] = 1000+i；磁力计 MAG_RAW[i] = 2000+i（静态）
 *         - 心跳       : 关节 0..3 与 IMU 位 = 0，另有一个随周期轮转的关节位；
 *                        其余（含触觉/磁力计）保持 1
 *                        （以上与 RV 验收文档 §6 逐字一致，2026-09-02 统一）
 */
void GloveFrame_FillFake(uint16_t *f, uint32_t cycle, uint32_t *heartbeat);

#endif /* GLOVE_FRAME_H */
