/* =============================================================================
 * cam_pipeline.c —— 双 IMX415 采集管线(V4: 双从机, XVS 由 STM32 供)
 *   从 glove_daqV4.c(←dual_cam.c)手术提取: 只保留 ENC(H.265)路径,
 *   删除 RTSP/预览/RAW/手套线程/main —— 相机逻辑与已验证版本逐行同源。
 *
 * 对外 API(cam_pipeline.h):
 *   cam_start(&cfg)         启动双摄(ISP→VI→VENC→采集线程→配对线程)
 *   cam_set_pair_cb(cb)     每配成一对帧回调一次(sync 线程上下文!)
 *   cam_stop()              停流并释放(与启动严格逆序)
 * 帧对里的 pts 与 GPIO 沿时间戳同源(CLOCK_MONOTONIC), 对齐引擎直接可用。
 * ============================================================================= */
#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>   /* open()/O_WRONLY: 写 /sys/.../lock_vts 用 */
#include <sys/ioctl.h>          /* 硬同步探测: 对 v4l-subdev 发 ioctl */
#include <linux/videodev2.h>    /* BASE_VIDIOC_PRIVATE (硬同步 ioctl 号基址) */

#include <rk_aiq_user_api2_imgproc.h>
#include <rk_aiq_user_api2_sysctl.h>

#include "rk_comm_video.h"
#include "rk_debug.h"
#include "rk_defines.h"
#include "rk_mpi_mb.h"
#include "rk_mpi_sys.h"
#include "rk_mpi_venc.h"
#include "rk_mpi_vi.h"


#include "glove_link.h"   /* 手套链路: 2672B 大端帧 + PA1 握手 + CRC16/ARC */
#include "glove_view.h"   /* 手套数据的终端视图(-w imu/joint/tactile/all) */

#define NUM_CAM 2
#define MAX_AIQ_CTX 4
#define QCAP 8 /* 每摄帧队列深度 */

/* ============== 配置 ============== */

static int g_width = 1920, g_height = 1080;
/* 输出形态: 恒 ENC(H.265) */
static RK_CODEC_ID_E g_codec = RK_VIDEO_ID_HEVC;
/* g_encode 已由 cam_cfg_t.codec_h265 取代 */
static RK_U32 g_bitrate = 10 * 1024;
static RK_S32 g_pair_cnt = -1;            /* 配对数上限, -1 无限 */
static RK_U64 g_sync_tol_us = 16000;      /* 软同步容差: 半帧(30fps≈16ms) */
static const char *g_iq_dir = "/oem/usr/share/iqfiles";
/* 落盘改由 recorder 模块负责 */
static int g_fps = 60;                     /* 目标帧率(经 rkaiq setFrameRate): >0=请求该帧率, 0=自动。
                                              注意: 本平台 setFrameRate(MANUAL) 压不住 AEC 拉长 VMAX,
                                              想"恒帧率挡光不掉"要靠 -L(驱动级VTS锁), 见下。默认 60。（此功能不一定有用） */
static int g_hw_sync = 0;                  /* XVS/XHS 硬同步就绪(hw_sync_setup 探测结果):
                                              1=驱动+DT已配1主1从 → 启动顺序反转(从机先);
                                              0=软同步(共享PTS基准+最近邻配对), 行为同旧版 */
static int g_lock_vts = 1;                 /* VTS(VMAX)锁, 写内核 imx415 模块参数 lock_vts:
                                              1(默认)=锁→恒60fps、挡住镜头也不掉帧(暗光只调曝光/增益);
                                              0=不锁→AEC 暗光下拉长VMAX降帧, 换更亮画面(自动曝光的完整行为)。
                                              这是"开不开自动降帧"的总开关, 比 -f 可靠(在驱动层卡死VBLANK上限)。*/
static volatile bool g_quit = false;

static rk_aiq_sys_ctx_t *g_aiq[MAX_AIQ_CTX];
static rk_aiq_working_mode_t g_wdr[MAX_AIQ_CTX];

/* ============== 一帧(编码后码流 或 原始NV12) ============== */
typedef struct {
	int cam_id;
	uint32_t seq;
	uint64_t pts;   /* 共享基准 PTS (us) */
	int is_raw;     /* 0=编码码流 1=NV12 */
	int width, height, vir_width;
	uint8_t *data;  /* 自有拷贝 */
	uint32_t len;
} cam_frame_t;

static cam_frame_t *frame_new(int cam, uint32_t seq, uint64_t pts, int is_raw, int w,
                              int h, int vw, const void *src, uint32_t len) {
	cam_frame_t *f = (cam_frame_t *)malloc(sizeof(*f));
	if (!f) return NULL;
	f->cam_id = cam; f->seq = seq; f->pts = pts; f->is_raw = is_raw;
	f->width = w; f->height = h; f->vir_width = vw; f->len = len;
	f->data = (uint8_t *)malloc(len);
	if (!f->data) { free(f); return NULL; }
	memcpy(f->data, src, len);
	return f;
}
static void frame_free(cam_frame_t *f) { if (f) { free(f->data); free(f); } }

/* ============== 线程安全帧队列(满则丢最旧) ============== */
typedef struct {
	cam_frame_t *buf[QCAP];
	int head, tail, count;
	pthread_mutex_t mtx;
	pthread_cond_t cond;
} frame_q_t;

static void q_init(frame_q_t *q) {
	memset(q, 0, sizeof(*q));
	pthread_mutex_init(&q->mtx, NULL);
	pthread_cond_init(&q->cond, NULL);
}
static void q_push(frame_q_t *q, cam_frame_t *f) {
	pthread_mutex_lock(&q->mtx);
	if (q->count == QCAP) { /* 丢最旧 */
		cam_frame_t *old = q->buf[q->head];
		q->head = (q->head + 1) % QCAP; q->count--;
		frame_free(old);
	}
	q->buf[q->tail] = f; q->tail = (q->tail + 1) % QCAP; q->count++;
	pthread_cond_signal(&q->cond);
	pthread_mutex_unlock(&q->mtx);
}
/* 阻塞取一帧, 超时或退出返回 NULL */
static cam_frame_t *q_pop(frame_q_t *q, int timeout_ms) {
	pthread_mutex_lock(&q->mtx);
	while (q->count == 0 && !g_quit) {
		struct timespec ts;
		clock_gettime(CLOCK_REALTIME, &ts);
		ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
		ts.tv_sec += timeout_ms / 1000 + ts.tv_nsec / 1000000000L;
		ts.tv_nsec %= 1000000000L;
		if (pthread_cond_timedwait(&q->cond, &q->mtx, &ts) == ETIMEDOUT) break;
	}
	cam_frame_t *f = NULL;
	if (q->count > 0) {
		f = q->buf[q->head]; q->head = (q->head + 1) % QCAP; q->count--;
	}
	pthread_mutex_unlock(&q->mtx);
	return f;
}
/* 非阻塞取一帧: 队列空就立刻返回 NULL(不等待).
 * 最近邻配对要"偷看下一帧"判断哪个更接近, 所以需要这个不阻塞的版本. */
static cam_frame_t *q_trypop(frame_q_t *q) {
	pthread_mutex_lock(&q->mtx);
	cam_frame_t *f = NULL;
	if (q->count > 0) {
		f = q->buf[q->head]; q->head = (q->head + 1) % QCAP; q->count--;
	}
	pthread_mutex_unlock(&q->mtx);
	return f;
}
static frame_q_t g_q[NUM_CAM];

/* ============== 打包格式(预留, eth 发送方序列化用) ============== */
#define DCAM_MAGIC 0x4d414344u /* 'D''C''A''M' */
typedef struct {
	uint32_t magic;
	uint16_t version;
	uint16_t cam_count;
	uint32_t pair_seq;
	uint64_t pts_us;   /* 参考时间戳(Cam0) */
} __attribute__((packed)) dcam_pkt_hdr_t;
typedef struct {
	uint8_t cam_id;
	uint8_t is_raw;    /* 0=H264/H265 1=NV12 */
	uint16_t width, height;
	uint64_t pts_us;
	uint32_t payload_len;
} __attribute__((packed)) dcam_sub_hdr_t;

/* 一个配对帧对(传给发送回调) */
typedef struct {
	uint32_t pair_seq;
	uint64_t pts_ref;      /* Cam0 pts */
	int64_t  dpts_us;      /* pts0 - pts1 */
	cam_frame_t *f[NUM_CAM];
} frame_pair_t;

/* 把帧对序列化进一段连续 buffer(eth 发送方调用). 返回写入字节数, 0=失败.
 * 调用方负责保证 out 足够大: hdr + NUM_CAM*(subhdr+payload). */
size_t dcam_serialize(const frame_pair_t *fp, uint8_t *out, size_t out_cap) {
	size_t need = sizeof(dcam_pkt_hdr_t);
	for (int i = 0; i < NUM_CAM; i++)
		need += sizeof(dcam_sub_hdr_t) + (fp->f[i] ? fp->f[i]->len : 0);
	if (out_cap < need) return 0;

	uint8_t *p = out;
	dcam_pkt_hdr_t h = {DCAM_MAGIC, 1, NUM_CAM, fp->pair_seq, fp->pts_ref};
	memcpy(p, &h, sizeof(h)); p += sizeof(h);
	for (int i = 0; i < NUM_CAM; i++) {
		cam_frame_t *f = fp->f[i];
		dcam_sub_hdr_t sh = {(uint8_t)(f ? f->cam_id : i), (uint8_t)(f ? f->is_raw : 0),
		                     (uint16_t)(f ? f->width : 0), (uint16_t)(f ? f->height : 0),
		                     f ? f->pts : 0, f ? f->len : 0};
		memcpy(p, &sh, sizeof(sh)); p += sizeof(sh);
		if (f && f->len) { memcpy(p, f->data, f->len); p += f->len; }
	}
	return (size_t)(p - out);
}

/* ============== 发送回调接口(预留: eth/UDP/TCP/ROS 在此挂入) ============== */
typedef int (*pub_sink_fn)(const frame_pair_t *fp, void *user);
static pub_sink_fn g_sink = NULL;
static void *g_sink_user = NULL;
void dual_set_sink(pub_sink_fn fn, void *user) { g_sink = fn; g_sink_user = user; }

/* ============== 硬同步探测(XVS/XHS 主从, 2026-08-05 落地) ==============
 * 驱动侧已按 DT(dual-cam-daq-hwsync.dtsi)把 cam0 配成 internal_master、
 * cam1 配成 slave(imx415.c 的 sync_mode)。这里只【探测】不【设置】:
 * 遍历 /dev/v4l-subdev* 读 RKMODULE_GET_SYNC_MODE, 凑齐 1主1从 = 硬同步就绪。
 * 老固件/未配 hwsync dtsi 时读不到 → 返回 -1 自动退回软同步, 完全兼容。
 * 注: 驱动故意忽略 SET_SYNC_MODE(防 rkaiq 覆盖 DT 配置), 故这里无任何设置调用。
 * 设计与验证计划: project/app/cam_daq/XVS_HWSYNC_DESIGN.md */

/* ioctl 号与 enum rkmodule_sync_mode 来自 rkaiq 自带的 rk-camera-module.h
 * (经 rk_aiq_user_api2_imgproc.h 传递包含, 与内核 uapi 同源, 无需本地定义) */

static int hw_sync_setup(void) {
	int masters = 0, slaves = 0;
	char path[32];
	for (int i = 0; i < 16; i++) {
		uint32_t mode = 0;
		int fd;
		snprintf(path, sizeof(path), "/dev/v4l-subdev%d", i);
		fd = open(path, O_RDWR);
		if (fd < 0)
			continue;
		/* 非 sensor 的 subdev 不认这个 ioctl, 返回错误 → 自然跳过 */
		if (ioctl(fd, RKMODULE_GET_SYNC_MODE, &mode) == 0) {
			if (mode == INTERNAL_MASTER_MODE) { masters++; printf("[sync] %s: XVS master\n", path); }
			else if (mode == SLAVE_MODE) { slaves++; printf("[sync] %s: XVS slave\n", path); }
		}
		close(fd);
	}
	/* V3 拓扑: 1 主 1 从(相机互同步);  V4 拓扑: 0 主 2 从(XVS 由 STM32 供)。
	 * 两种都算"硬同步就绪"; V4 下没有相机间启动顺序约束(都是从机等脉冲)。 */
	if (masters == 1 && slaves == 1) {
		printf("[sync] 拓扑: 1主1从(相机互同步, V3 式)\n");
		return 0;
	}
	if (masters == 0 && slaves == 2) {
		printf("[sync] 拓扑: 双从机(XVS 由 STM32/PB10 供, V4 式)\n");
		printf("[sync] ★相机锁不锁得住取决于 STM32 XVS 是否在发★ dpts 恒定=锁上\n");
		return 0;
	}
	return -1;   /* 未配同步 dtsi 或旧驱动 → 维持软同步 */
}

/* ============== VTS 锁开关 (写内核 imx415 模块参数) ============== */
/* 把 on(0/1) 写到 /sys/module/imx415/parameters/lock_vts。
 * 这是内核驱动暴露的"可写模块参数"——用户态往这个文件写字符就能改内核里的变量,
 * 不用重编内核。驱动在每次"开流"时读取它, 决定 AEC 能不能为了曝光把帧率拉低。
 *   on=1: 锁帧率(VBLANK 上限压到额定 VMAX)→ 恒 60fps、挡光不掉;
 *   on=0: 不锁 → 暗光下自动降帧换更亮画面。
 * 用 open/write 系统调用直接写文件(sysfs 就是普通文件接口)。失败仅提示、不中断。*/
static void apply_lock_vts(int on) {
	const char *path = "/sys/module/imx415/parameters/lock_vts";
	int fd = open(path, O_WRONLY);              /* 以"只写"打开这个内核参数文件 */
	if (fd < 0) {
		printf("[lock_vts] 打不开 %s (%s); 内核可能没编 lock_vts 参数, 跳过\n",
		       path, strerror(errno));
		return;
	}
	char c = on ? '1' : '0';                    /* 写一个字符 '0' 或 '1' */
	if (write(fd, &c, 1) == 1)
		printf("[lock_vts] 已设为 %d (%s)\n", on, on ? "锁帧率,恒60fps挡光不掉" : "不锁,AEC暗光自动降帧");
	else
		printf("[lock_vts] 写 %s 失败(%s)\n", path, strerror(errno));
	close(fd);
}

/* ============== ISP/3A (rkaiq user api2) ============== */
static int isp_init(int cam, rk_aiq_working_mode_t wdr, RK_BOOL multi, const char *iq) {
	if (cam >= MAX_AIQ_CTX) return -1;
	g_wdr[cam] = wdr;
	char s[16]; snprintf(s, sizeof(s), "%d", (int)wdr); setenv("HDR_MODE", s, 1);
	/* ★2026-07-08 防护(与 dual_cam_push 同源修复):enumStaticMetasByPhyId 失败时
	 * 【不填充】info——不查返回值就会把栈上随机残渣当传感器名传给 sysctl_init,
	 * 在 librkaiq 里段错误(开机早期 rkaiq 看不到相机时 100% 复现)。
	 * 以"枚举成功+名字可打印"为真就绪条件,失败每秒重试最多 60 次。 */
	rk_aiq_static_info_t info;
	{
		int tries = 0;
		for (;;) {
			memset(&info, 0, sizeof(info));
			XCamReturn eret = rk_aiq_uapi2_sysctl_enumStaticMetasByPhyId(cam, &info);
			const char *n = info.sensor_info.sensor_name;
			int ok = (eret == XCAM_RETURN_NO_ERROR) && n[0];
			for (const char *p = n; ok && *p; p++)
				if (*p < 0x20 || *p > 0x7E) ok = 0;
			if (ok) break;
			if (++tries >= 60) { printf("[ISP] cam%d 枚举传感器失败(ret=%d), 放弃\n", cam, (int)eret); return -1; }
			if (tries == 1) printf("[ISP] cam%d 传感器枚举未就绪, 每秒重试中...\n", cam);
			sleep(1);
		}
		if (tries) printf("[ISP] cam%d 枚举就绪(等了%d秒)\n", cam, tries);
	}
	printf("[ISP] CamId %d sensor=%s\n", cam, info.sensor_info.sensor_name);
	rk_aiq_uapi2_sysctl_preInit_devBufCnt(info.sensor_info.sensor_name, "rkraw_rx", 2);
	rk_aiq_sys_ctx_t *ctx = rk_aiq_uapi2_sysctl_init(info.sensor_info.sensor_name, iq, NULL, NULL);
	if (!ctx) { printf("[ISP] init fail cam%d\n", cam); return -1; }
	if (multi) rk_aiq_uapi2_sysctl_setMulCamConc(ctx, true);
	g_aiq[cam] = ctx;
	return 0;
}
static int isp_run(int cam) {
	if (!g_aiq[cam]) return -1;
	if (rk_aiq_uapi2_sysctl_prepare(g_aiq[cam], 0, 0, g_wdr[cam])) { g_aiq[cam] = NULL; return -1; }
	if (rk_aiq_uapi2_sysctl_start(g_aiq[cam])) return -1;
	printf("[ISP] cam%d aiq started\n", cam);

	/* ---- 设置帧率：决定 sensor/ISP 跑多快，以及 AEC 能否为了曝光拉长帧时间 ----
	 * 背景：sensor 驱动里 binning 模式的默认能力是 ~90fps，但 rkaiq 的自动曝光(AEC)
	 *       在暗光下会"拉长每帧时间(增大 sensor 的 VTS 寄存器)来延长曝光" → 帧率掉下来。
	 *       (这就是之前挡光会从60掉到60以下的根因；过去靠改内核驱动锁死 VTS 解决。)
	 * 现在改成应用层控制：rk_aiq_uapi2_setFrameRate(ctx, info) 是 rkaiq 的运行时帧率接口，
	 *   info.mode = OP_MANUAL → 把帧率"锁"在 info.fps（AEC 不许再拉长帧，暗光只靠加增益变亮）；
	 *   info.mode = OP_AUTO   → 交给 AEC 自动（亮处快、暗处慢，上限是 sensor 模式能力）。
	 * 这样"锁不锁帧率/锁多少"完全由 -f 参数控制，不必再改内核驱动重烧固件。
	 *   (接口原型: media/out/include/rkaiq/uAPI2/rk_aiq_user_api2_imgproc.h:151
	 *    枚举/结构: .../uAPI2/rk_aiq_user_api_common.h:11(opMode_t) / :89(frameRateInfo_t)) */
	frameRateInfo_t fr;
	if (g_fps > 0) { fr.mode = OP_MANUAL; fr.fps = (unsigned int)g_fps; }  /* 固定帧率 */
	else           { fr.mode = OP_AUTO;   fr.fps = 0; }                    /* 自动(随光照变) */
	if (rk_aiq_uapi2_setFrameRate(g_aiq[cam], fr) == 0)
		printf("[ISP] cam%d setFrameRate mode=%s fps=%d\n",
		       cam, (g_fps > 0) ? "MANUAL(锁定)" : "AUTO(自动)", g_fps);
	else
		printf("[ISP] cam%d setFrameRate 失败(忽略,改用 sensor 模式默认帧率)\n", cam);
	return 0;
}
static void isp_stop(int cam) {
	if (!g_aiq[cam]) return;
	rk_aiq_uapi2_sysctl_stop(g_aiq[cam], false);
	rk_aiq_uapi2_sysctl_deinit(g_aiq[cam]);
	g_aiq[cam] = NULL;
}

/* ============== VI ============== */
static int vi_dev_init(int dev) {
	VI_DEV_ATTR_S da; VI_DEV_BIND_PIPE_S bp;
	memset(&da, 0, sizeof(da)); memset(&bp, 0, sizeof(bp));
	int ret = RK_MPI_VI_GetDevAttr(dev, &da);
	if (ret == RK_ERR_VI_NOT_CONFIG) {
		if ((ret = RK_MPI_VI_SetDevAttr(dev, &da)) != RK_SUCCESS) {
			printf("VI_SetDevAttr dev%d 0x%x\n", dev, ret); return -1;
		}
	}
	if (RK_MPI_VI_GetDevIsEnable(dev) != RK_SUCCESS) {
		if ((ret = RK_MPI_VI_EnableDev(dev)) != RK_SUCCESS) {
			printf("VI_EnableDev dev%d 0x%x\n", dev, ret); return -1;
		}
		bp.u32Num = 1; bp.PipeId[0] = dev;
		if ((ret = RK_MPI_VI_SetDevBindPipe(dev, &bp)) != RK_SUCCESS) {
			printf("VI_SetDevBindPipe dev%d 0x%x\n", dev, ret); return -1;
		}
	}
	return 0;
}
static int vi_chn_init(int dev, int chn, int w, int h, int depth) {
	VI_CHN_ATTR_S a; memset(&a, 0, sizeof(a));
	a.stIspOpt.u32BufCount = (depth > 0) ? 4 : 2;
	a.stIspOpt.enMemoryType = VI_V4L2_MEMORY_TYPE_DMABUF;
	a.stSize.u32Width = w; a.stSize.u32Height = h;
	a.enPixelFormat = RK_FMT_YUV420SP;
	a.enCompressMode = COMPRESS_MODE_NONE;
	a.u32Depth = depth; /* enc(绑VENC)=0; raw(GetChnFrame)>=1 */
	int ret = RK_MPI_VI_SetChnAttr(dev, chn, &a);
	if (ret) { printf("VI_SetChnAttr dev%d 0x%x\n", dev, ret); return ret; }
	ret = RK_MPI_VI_EnableChn(dev, chn);
	if (ret) { printf("VI_EnableChn dev%d 0x%x\n", dev, ret); return ret; }
	return 0;
}

/* ============== VENC ============== */
static int venc_init(int chn, int w, int h, RK_CODEC_ID_E type) {
	VENC_CHN_ATTR_S a; VENC_RECV_PIC_PARAM_S r;
	memset(&a, 0, sizeof(a));
	if (type == RK_VIDEO_ID_AVC) {
		a.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
		a.stRcAttr.stH264Cbr.u32BitRate = g_bitrate; a.stRcAttr.stH264Cbr.u32Gop = 60;
		a.stVencAttr.u32Profile = H264E_PROFILE_HIGH;
	} else {
		a.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
		a.stRcAttr.stH265Cbr.u32BitRate = g_bitrate; a.stRcAttr.stH265Cbr.u32Gop = 60;
	}
	a.stVencAttr.enType = type;
	a.stVencAttr.enPixelFormat = RK_FMT_YUV420SP;
	a.stVencAttr.u32PicWidth = w; a.stVencAttr.u32PicHeight = h;
	a.stVencAttr.u32VirWidth = w; a.stVencAttr.u32VirHeight = h;
	a.stVencAttr.u32StreamBufCnt = 2;
	a.stVencAttr.u32BufSize = w * h * 3 / 2;
	a.stVencAttr.enMirror = MIRROR_NONE;
	int ret = RK_MPI_VENC_CreateChn(chn, &a);
	if (ret != RK_SUCCESS) { printf("VENC_CreateChn %d 0x%x\n", chn, ret); return ret; }
	memset(&r, 0, sizeof(r)); r.s32RecvPicNum = -1;
	RK_MPI_VENC_StartRecvFrame(chn, &r);
	return 0;
}
static void vi_bind_venc(int viDev, int viChn, int vencChn) {
	MPP_CHN_S s = {RK_ID_VI, viDev, viChn}, d = {RK_ID_VENC, 0, vencChn};
	int ret = RK_MPI_SYS_Bind(&s, &d);
	if (ret != RK_SUCCESS) printf("bind VI(%d,%d)->VENC%d 0x%x\n", viDev, viChn, vencChn, ret);
}
static void vi_unbind_venc(int viDev, int viChn, int vencChn) {
	MPP_CHN_S s = {RK_ID_VI, viDev, viChn}, d = {RK_ID_VENC, 0, vencChn};
	RK_MPI_SYS_UnBind(&s, &d);
}

/* ============== 实时 fps 统计小工具 ==============
 * 每满 ~1 秒打印一次"过去这一秒发生了多少次事件"(即 fps), 然后清零重计.
 * 用 CLOCK_MONOTONIC(单调递增时钟, 不受系统对时影响), 反映的是"稳态实时帧率",
 * 比"总帧数/总耗时"更准——后者会被启动初始化那几秒拉低、产生误导.
 *   last: 上次打印的时刻(调用方初始化一次)
 *   cnt : 自上次打印以来的事件计数(调用方每发生一次就 ++)
 *   label: 打印用的名字 */
static void fps_tick(struct timespec *last, uint32_t *cnt, const char *label) {
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	double el = (now.tv_sec - last->tv_sec) + (now.tv_nsec - last->tv_nsec) / 1e9;
	if (el >= 1.0) {                 /* 满 1 秒才算一次, 避免抖动 */
		double fps = (double)*cnt / el;
		/* 只交数据不打印: FSM 每秒的状态行会把三路 fps 合成一行输出(减少刷屏);
		 * 视图模式下由视图页脚显示。 */
		gv_cam_fps(label, fps);
		*cnt = 0;
		*last = now;
	}
}

/* ============== 采集线程(每摄一个) ============== */
typedef struct { int cam; int vi_dev; int venc_chn; } cap_arg_t;

static void *cap_enc_thread(void *arg) {
	cap_arg_t *c = (cap_arg_t *)arg;
	VENC_STREAM_S st; st.pstPack = malloc(sizeof(VENC_PACK_S));
	uint32_t seq = 0;
	/* 实时采集 fps 统计(这一路相机每秒真正出了多少帧) */
	struct timespec fps_t; clock_gettime(CLOCK_MONOTONIC, &fps_t);
	uint32_t fps_c = 0;
	char fps_lbl[16]; snprintf(fps_lbl, sizeof(fps_lbl), "cam%d采集", c->cam);
	while (!g_quit) {
		if (RK_MPI_VENC_GetStream(c->venc_chn, &st, 1000) == RK_SUCCESS) {
			void *p = RK_MPI_MB_Handle2VirAddr(st.pstPack->pMbBlk);
			cam_frame_t *f = frame_new(c->cam, seq++, st.pstPack->u64PTS, 0,
			                           g_width, g_height, g_width, p, st.pstPack->u32Len);
			if (f) q_push(&g_q[c->cam], f);
			RK_MPI_VENC_ReleaseStream(c->venc_chn, &st);
			fps_c++; fps_tick(&fps_t, &fps_c, fps_lbl); /* 每出一帧计一次 */
		}
	}
	free(st.pstPack);
	printf("[cap] cam%d enc thread exit (%u frames)\n", c->cam, seq);
	return NULL;
}
/* ============== 同步配对线程(软同步, 最近邻匹配) ==============
 *
 * 【为什么不用"各取队首、超差就丢"的贪心法】
 *   两摄帧率常不完全一致(实测 cam0≈30 / cam1≈24fps). 贪心法只比较两队队首,
 *   速率不一致时会误丢, 配对率被压低(实测只到 ~18fps, 低于慢摄的 24fps).
 *
 * 【最近邻匹配】
 *   目标: 让较慢一路的每一帧, 都和较快一路里"时间最接近"的那帧配上,
 *   配对率就能逼近"较慢一路的帧率"(即这套硬件双摄的理论上限 ~24fps).
 *   做法: 比较两队当前帧 a(cam0)/b(cam1):
 *     - 若较早的一路再往后取一帧能更接近对方 => 当前这帧是"多余帧", 丢掉, 前进;
 *     - 否则 a/b 已是彼此最接近的一对: 时间差<=容差就配对, 否则丢较早的那帧推进时间.
 *   用 q_trypop "偷看"下一帧; 偷看出来但暂时不用的帧, 用 *_hold 暂存到下一轮, 不丢失.
 */
static void *sync_thread(void *arg) {
	(void)arg;
	cam_frame_t *a = NULL, *b = NULL;       /* 当前待配对的两帧 */
	cam_frame_t *a_hold = NULL, *b_hold = NULL; /* 偷看出来、留给下一轮的帧 */
	uint32_t pair_seq = 0;
	long drop0 = 0, drop1 = 0;              /* 各路被丢弃的"多余帧"数 */
	/* 实时"同步成对" fps 统计——这就是你要看的"成功同步匹配帧率" */
	struct timespec fps_t; clock_gettime(CLOCK_MONOTONIC, &fps_t);
	uint32_t fps_c = 0;

	while (!g_quit) {
		/* 装载当前帧: 优先用上一轮暂存的, 否则阻塞取(最多等200ms) */
		if (!a) { a = a_hold; a_hold = NULL; if (!a) a = q_pop(&g_q[0], 200); }
		if (!b) { b = b_hold; b_hold = NULL; if (!b) b = q_pop(&g_q[1], 200); }
		if (!a || !b) continue;

		int64_t d = (int64_t)a->pts - (int64_t)b->pts; /* a 比 b 早多少(负=a更早) */

		if (d < 0) {
			/* a(cam0) 比 b 早. 看下一帧 cam0 是否更接近 b */
			cam_frame_t *a2 = q_trypop(&g_q[0]);
			if (a2) {
				if (llabs((int64_t)a2->pts - (int64_t)b->pts) <= -d) {
					/* 下一帧 a2 离 b 更近(或相等) => 当前 a 是多余帧, 丢, 用 a2 重来 */
					frame_free(a); a = a2; drop0++;
					continue;
				}
				a_hold = a2; /* a 才是离 b 最近的; a2 留下一轮用 */
			}
		} else if (d > 0) {
			/* b(cam1) 比 a 早. 看下一帧 cam1 是否更接近 a */
			cam_frame_t *b2 = q_trypop(&g_q[1]);
			if (b2) {
				if (llabs((int64_t)b2->pts - (int64_t)a->pts) <= d) {
					frame_free(b); b = b2; drop1++;
					continue;
				}
				b_hold = b2;
			}
		}

		/* 到这里: a、b 已是彼此时间最接近的一对 */
		int64_t ad = d < 0 ? -d : d;
		if ((uint64_t)ad <= g_sync_tol_us) {
			/* 时间差在容差内 => 配成一对, 交给 sink */
			frame_pair_t fp = {pair_seq, a->pts, d, {a, b}};
			if (g_sink) g_sink(&fp, g_sink_user);
			frame_free(a); frame_free(b); a = b = NULL;
			pair_seq++;
			fps_c++; fps_tick(&fps_t, &fps_c, "同步成对"); /* 每配成一对计一次 */
			if (g_pair_cnt >= 0 && (RK_S32)pair_seq >= g_pair_cnt) { g_quit = true; break; }
		} else if (d < 0) {
			/* 最近的也超差: 丢较早的 a 推进时间(下轮用 a_hold 再和 b 比) */
			frame_free(a); a = NULL; drop0++;
		} else {
			frame_free(b); b = NULL; drop1++;
		}
	}
	frame_free(a); frame_free(b); frame_free(a_hold); frame_free(b_hold);
	printf("[sync] exit: %u pairs, drop cam0=%ld cam1=%ld (tol=%lluus)\n",
	       pair_seq, drop0, drop1, (unsigned long long)g_sync_tol_us);
	return NULL;
}


/* ================================================================
 * 对外 API(以下为新增, 上方全部来自已验证的 glove_daqV4/dual_cam)
 * ================================================================ */
#include "cam_pipeline.h"

static pthread_t   g_cap_tid[NUM_CAM], g_sync_tid;
static cap_arg_t   g_ca[NUM_CAM];
static int         g_running = 0;

/* 帧对回调: 内部 frame_pair_t → 对外 cam_pair_t 的蹦床 */
static cam_pair_cb g_pair_cb = NULL;
static void       *g_pair_user = NULL;

static int pair_trampoline(const frame_pair_t *fp, void *user)
{
	(void)user;
	if (!g_pair_cb) return 0;
	cam_pair_t p = {
		.pair_seq = fp->pair_seq,
		.pts0 = fp->f[0]->pts, .pts1 = fp->f[1]->pts,
		.dpts_us = fp->dpts_us,
		.seq0 = fp->f[0]->seq, .seq1 = fp->f[1]->seq,
		.d0 = fp->f[0]->data, .l0 = fp->f[0]->len,
		.d1 = fp->f[1]->data, .l1 = fp->f[1]->len,
	};
	g_pair_cb(&p, g_pair_user);
	return 0;
}

void cam_set_pair_cb(cam_pair_cb cb, void *user) { g_pair_cb = cb; g_pair_user = user; }
int  cam_hw_sync_ok(void) { return g_hw_sync; }

/* 相机"在位"探测(不启动): 内核 boot 时已 probe 过传感器,
 * 探测成功的铁证 = i2c 设备已绑定驱动(sysfs 有 driver 符号链接)。
 * 比用户态裸 i2c 读靠谱: 此刻传感器断电(XCLR 低), 裸读必 NAK。 */
int cam_probe_i2c(void)
{
	const char *dev[2] = { "/sys/bus/i2c/devices/3-001a/driver",
	                       "/sys/bus/i2c/devices/4-001a/driver" };
	int ok = 0;
	for (int i = 0; i < 2; i++) {
		if (access(dev[i], F_OK) == 0) { ok++; }
		else printf("[cam] ★相机%d 未探到★(%s 不存在: 排线/供电/内核probe失败)\n", i, dev[i]);
	}
	return ok;   /* 返回在位个数(应=2) */
}

int cam_start(const cam_cfg_t *cfg)
{
	if (g_running) return 0;
	if (cfg) {
		if (cfg->width > 0)  g_width  = cfg->width;
		if (cfg->height > 0) g_height = cfg->height;
		if (cfg->bitrate_kbps > 0) g_bitrate = (RK_U32)cfg->bitrate_kbps;
		if (cfg->fps > 0) g_fps = cfg->fps;
		if (cfg->iq_dir) g_iq_dir = cfg->iq_dir;
		g_codec = cfg->codec_h265 ? RK_VIDEO_ID_HEVC : RK_VIDEO_ID_AVC;
	}
	g_quit = false;
	dual_set_sink(pair_trampoline, NULL);

	/* 同步方式: V4 = 0主2从(XVS 由 STM32 供)也算硬同步就绪 */
	g_hw_sync = (hw_sync_setup() == 0);
	printf("[cam] %s\n", g_hw_sync ? "硬同步就绪(等 STM32 XVS 锁相)" : "★未探到从机配置, 软同步兜底★");
	g_lock_vts = 1;                       /* XVS 从机必须锁 VTS, 否则 AE 拉长帧就失步 */
	apply_lock_vts(1);

	for (int k = 0; k < NUM_CAM; k++) {   /* 双从机无顺序约束, 沿用反序无害 */
		int i = g_hw_sync ? (NUM_CAM - 1 - k) : k;
		if (isp_init(i, RK_AIQ_WORKING_MODE_NORMAL, RK_TRUE, g_iq_dir) != 0) return -1;
		if (isp_run(i) != 0) return -1;
	}
	if (RK_MPI_SYS_Init() != RK_SUCCESS) { printf("[cam] SYS_Init fail\n"); return -1; }
	RK_MPI_SYS_InitPTSBase(0);

	for (int i = 0; i < NUM_CAM; i++) q_init(&g_q[i]);
	for (int k = 0; k < NUM_CAM; k++) {
		int i = g_hw_sync ? (NUM_CAM - 1 - k) : k;
		if (vi_dev_init(i) != 0) return -1;
		if (vi_chn_init(i, 0, g_width, g_height, 0) != 0) return -1;
	}
	for (int i = 0; i < NUM_CAM; i++) {
		if (venc_init(i, g_width, g_height, g_codec) != 0) return -1;
		vi_bind_venc(i, 0, i);
	}
	for (int i = 0; i < NUM_CAM; i++) {
		g_ca[i].cam = i; g_ca[i].vi_dev = i; g_ca[i].venc_chn = i;
		pthread_create(&g_cap_tid[i], NULL, cap_enc_thread, &g_ca[i]);
	}
	pthread_create(&g_sync_tid, NULL, sync_thread, NULL);
	g_running = 1;
	printf("[cam] 双摄管线已启动 %dx%d %s %ukbps(从机模式, 等 XVS; 无 XVS 时自由跑)\n",
	       g_width, g_height, g_codec == RK_VIDEO_ID_HEVC ? "H265" : "H264", g_bitrate);
	return 0;
}

void cam_stop(void)
{
	if (!g_running) return;
	g_quit = true;
	pthread_join(g_sync_tid, NULL);
	for (int i = 0; i < NUM_CAM; i++) pthread_join(g_cap_tid[i], NULL);
	for (int i = 0; i < NUM_CAM; i++) {
		vi_unbind_venc(i, 0, i);
		RK_MPI_VENC_StopRecvFrame(i);
		RK_MPI_VENC_DestroyChn(i);
	}
	for (int i = 0; i < NUM_CAM; i++) { RK_MPI_VI_DisableChn(i, 0); RK_MPI_VI_DisableDev(i); }
	RK_MPI_SYS_Exit();
	for (int i = 0; i < NUM_CAM; i++) isp_stop(i);
	g_running = 0;
	printf("[cam] 双摄管线已停止\n");
}
