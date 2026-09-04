/* =============================================================================
 * recorder.h —— 分段落盘(每次进入 RUNNING 开一个新段目录)
 *   <base>/seg_<段号>_<boottime秒>/
 *     cam0.h265 cam1.h265   两路裸码流(顺序追加, IPPP 无B帧)
 *     pairs.csv             pair_seq,seq0,seq1,pts0,pts1,dpts_us,cycle,residual_us,
 *                           off0,len0,off1,len1  (off=码流文件内字节偏移)
 *     glove.bin             原始 2690B 数据帧顺序追加
 *     glove.csv             cycle,edge_ns,off  (off=glove.bin 内偏移)
 * 线程模型: 相机行(pairs/camX)在 sink 线程写, 手套行在 FSM 线程写 —— 文件各归各,
 * 无共享文件, 无锁。csv 行缓冲, bin 大块写。
 * ============================================================================= */
#ifndef RECORDER_H
#define RECORDER_H
#include <stdint.h>
#include "cam_pipeline.h"

int  rec_open(const char *base_dir);              /* 建基目录, 失败 -1 */
int  rec_segment_start(unsigned seg);             /* 进入 RUNNING 时调 */
void rec_on_pair(const cam_pair_t *p, int calib_ok, uint32_t cycle, int64_t residual_us);
void rec_on_glove(const uint8_t *raw2690, uint32_t cycle, uint64_t edge_ns);
void rec_segment_stop(void);
void rec_close(void);
int  rec_active(void);
#endif
