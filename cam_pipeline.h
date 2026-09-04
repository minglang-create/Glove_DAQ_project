/* =============================================================================
 * cam_pipeline.h —— 双 IMX415 采集管线(ENC/H.265, V4 双从机)对外接口
 * 实现提取自已验证的 dual_cam 血统(见 cam_pipeline.c 头注释)。
 * 帧对的 pts 与 GPIO 沿时间戳同源(CLOCK_MONOTONIC µs), 可直接进对齐引擎。
 * ============================================================================= */
#ifndef CAM_PIPELINE_H
#define CAM_PIPELINE_H
#include <stdint.h>

typedef struct {
	int width, height;        /* 0=默认 1920x1080 */
	int bitrate_kbps;         /* 0=默认 10240 */
	int fps;                  /* 0=默认 60 */
	int codec_h265;           /* 1=H.265(默认) 0=H.264 */
	const char *iq_dir;       /* NULL=/oem/usr/share/iqfiles */
} cam_cfg_t;

typedef struct {
	uint32_t pair_seq;
	uint64_t pts0, pts1;      /* 两路帧 PTS(µs, 单调钟) */
	int64_t  dpts_us;         /* pts0-pts1, 硬同步锁相时恒定(µs级) */
	uint32_t seq0, seq1;      /* 各路 VI/VENC 帧序号 */
	const uint8_t *d0; uint32_t l0;   /* cam0 H.265 一帧码流 */
	const uint8_t *d1; uint32_t l1;   /* cam1 */
} cam_pair_t;

/* 每配成一对帧回调一次。★运行在 sync 线程上下文: 里面别做慢事(落盘要快写/缓冲)★ */
typedef void (*cam_pair_cb)(const cam_pair_t *p, void *user);

int  cam_probe_i2c(void);                          /* 自检: 返回在位相机数(应=2), 不启动 */
int  cam_start(const cam_cfg_t *cfg);              /* NULL=全默认 */
void cam_set_pair_cb(cam_pair_cb cb, void *user);  /* 可在 start 前后设 */
void cam_stop(void);
int  cam_hw_sync_ok(void);                         /* 1=探到双从机(硬同步拓扑) */
#endif
