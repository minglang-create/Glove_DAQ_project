/* =============================================================================
 * recorder.h —— 分段落盘(SD 卡, exFAT)
 *   <base>/<虚拟时间YYYYMMDDHHMM>_seg_<段号>/   (虚拟时间: 见 recorder.c vtime_init 注释)
 *     cam0.h265 cam1.h265   两路裸码流(顺序追加, IPPP 无B帧)
 *     pairs.csv             pair_seq,seq0,seq1,pts0,pts1,dpts_us,cycle,residual_us,
 *                           off0,len0,off1,len1  (off=本段码流文件内字节偏移)
 *     glove.bin             原始 2690B 数据帧顺序追加
 *     glove.csv             cycle,edge_ns,off  (off=本段 glove.bin 内偏移)
 *     ext_joints.csv        cycle,seq,valid,latency_us,ch0..ch20  (外接 21 路关节 ADC,
 *                           -U 启用时才有内容; valid=0 表示本拍缺失, latency 见 ext_uart.h)
 *   段的产生: ①每次进入 RUNNING 开新段  ②运行中每 rotate_min 分钟自动切段
 *   跨段联结靠 cycle(全局单调), 段只是"文件容器", 对上位机无语义。
 *
 * 【线程模型】
 *   热路径 rec_on_pair(相机 sink 线程)/rec_on_glove(FSM 线程): 一把互斥锁 + fwrite
 *   → 数据进内核页缓存(微秒级), 从不直接等 SD 卡。
 *   慢操作(fsync / 切段开关文件 / 查剩余空间)全部在【flush 线程】做, 绝不阻塞热路径
 *   —— 否则 SD 卡一次几百 ms 的 GC 停顿就会撞上相机队列 QCAP=8 帧(133ms)的丢帧线。
 *
 * 【为什么必须周期 fsync】exFAT 的文件大小只在 fsync/close 时写进目录项。
 *   不 fsync 就拔卡 → 数据块在卡上、目录项却是旧大小 → PC 上看到 0 字节/截断文件。
 *   每 fsync_sec 秒同步一次, 最坏丢 fsync_sec 秒; 段结束时也 fsync。
 * 【为什么按时间切段】损伤隔离: 拔卡/掉电只危及当前段尾巴, 之前的段已 fsync+close 完好;
 *   副产品: adb 可边录边拉已完成的段。
 * ============================================================================= */
#ifndef RECORDER_H
#define RECORDER_H
#include <stdint.h>
#include <stddef.h>
#include "cam_pipeline.h"
#include "ext_uart.h"

typedef struct {
	int    fsync_sec;      /* 周期 fsync 间隔(秒), 默认 5, 0=关 */
	int    rotate_min;     /* 按时间切段(分钟), 默认 10, 0=关 */
	double min_free_gb;    /* 剩余空间阈值(GB), 默认 2.0; 0=完全不检查存储(调试用) */
} rec_cfg_t;

/* 自检: base 所在文件系统必须是 SD 卡(/dev/mmcblk1*)且剩余 ≥ min_free_gb。
 * 返回 0=通过; -1=失败, why 里是人能读的原因。min_free_gb==0 时直接通过(不检查)。 */
int  rec_check_storage(const char *base, double min_free_gb, char *why, size_t why_len);

int  rec_open(const char *base_dir, const rec_cfg_t *cfg);   /* 建基目录+起 flush 线程 */
int  rec_segment_start(void);                                /* 进入 RUNNING 时调 */
void rec_on_pair(const cam_pair_t *p, int calib_ok, uint32_t cycle, int64_t residual_us);
void rec_on_glove(const uint8_t *raw2690, uint32_t cycle, uint64_t edge_ns);
void rec_on_ext(uint32_t cycle, const ext_frame_t *f);      /* FSM 线程, 紧跟 rec_on_glove */
void rec_segment_stop(void);                                 /* 暂停/结束时调(含 fsync) */
void rec_close(void);                                        /* 停 flush 线程 */

int      rec_active(void);
unsigned rec_segment_count(void);   /* 已开过的段数(含自动切的) */
int      rec_space_low(void);       /* 1 = flush 线程发现剩余 < 阈值(FSM 应停止采集) */
double   rec_free_gb(void);         /* 最近一次测到的剩余空间(状态行显示) */
#endif
