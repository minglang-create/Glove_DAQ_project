/* =============================================================================
 * align.h —— 相机帧 ↔ CYCLE 自动对齐引擎(契约第四次修订: "RV 自动标定偏移")
 *
 * 【原理】手套线程每帧提供 (CYCLE, PA1沿内核时间戳); 相机帧对带 PTS(同源单调钟)。
 *   两个 60Hz 网格只差一个常数 offset(含: PTS基准差 + 曝光→SOF延迟 + PA1 的 3~8ms):
 *   PA1 抖动 ±2.5ms ≪ 半周期 8.3ms → 取中位数标定 offset 后, 最近邻整数指派唯一。
 * 【为什么必须有它】IMX415 从机无 XVS 时自由跑(V2 实测), 相机在 0xC301 启动后、
 *   第一个 XVS 前已吐出数量不定的自由跑帧 → "帧数天然一致"不成立, 帧号↔CYCLE 的
 *   偏移只能每次会话实测标定(锁相后恒定)。
 * ============================================================================= */
#ifndef ALIGN_H
#define ALIGN_H
#include <stdint.h>

void align_reset(void);
/* 手套线程每收到一个有效数据帧调用(edge_ns=0 表示电平兜底路径, 不入锚点) */
void align_on_glove(uint32_t cycle, uint64_t edge_ns);
/* 相机 sink 每帧对调用: 返回 0=已标定(cycle/residual 有效), -1=标定未完成 */
int  align_lookup(uint64_t pts_us, uint32_t *cycle, int64_t *residual_us);
/* 状态: 0=采样中 1=已锁定; 输出统计(打印用, 任意出参可为 NULL)
 *   offset_ms      当前偏移(ms)
 *   resid_std_us   窗内残差 σ(µs) = 当前对齐抖动; 半周期/σ 即判决裕量
 *   n_samples      累计已指派帧数
 *   drift_us_per_s 跟踪到的漂移率(µs/s): 同源≈0, 异源=两晶振 ppm 差
 *   slips          滑移计数: >0 表示曾发生错拍指派(漂移越过半周期), 应排查 */
int  align_status(double *offset_ms, double *resid_std_us, uint32_t *n_samples,
                  double *drift_us_per_s, uint32_t *slips);
#endif
