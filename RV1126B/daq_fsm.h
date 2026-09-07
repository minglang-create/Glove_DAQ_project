/* =============================================================================
 * daq_fsm.h —— 手套 DAQ 生命周期状态机(按《协议契约_V1 第四次修订》实现)
 *
 *   SELFCHECK → WAIT_READY(自检包/按键/0xC301) → CAM_START → WAIT_FRAME
 *   → RUNNING →(长按/0x5F01)→ exec 重启回 SELFCHECK
 *
 * 事件来源: ① SPI 小包(100ms 轮询) ② 数据帧出现(采集开始的最终确认)
 *          ③ 按键(空闲短按=开采; 任何时候长按≥2s=落盘+发0x5F01复位STM32+程序重启回待机;
 *             之后再短按=重新开采(新段)。按键永不退出程序; 真要结束=长按落盘后断电) ④ 信号(USR2=模拟短按,
 *          USR1=发 0x5F01 重新自检并重启进程, INT/TERM=优雅退出)
 * ============================================================================= */
#ifndef DAQ_FSM_H
#define DAQ_FSM_H
#include "cam_pipeline.h"
#include "recorder.h"

typedef struct {
	int auto_mode;         /* -A 台架直通: 不等按键/0xC301, MISO 见数据帧即启相机开跑 */
	int no_cam;            /* -X 纯链路调试: 不碰相机 */
	int verify_fake;       /* -V 假数据逐帧验收 */
	int verbose;           /* -v 相机初始化时显示全部 SDK 日志(默认收进 /tmp/cam_init.log) */
	const char *rec_dir;   /* 落盘基目录(NULL=不落盘) */
	rec_cfg_t   rec;       /* fsync 间隔/切段时长/空间阈值 */
	const char *ext_dev;   /* -U 外接关节 ADC 串口(NULL=不启用) */
	int         ext_baud;
	int         ext_trig;  /* 1=trig 方案一, 0=free 方案二 */
	cam_cfg_t   cam;
} fsm_cfg_t;

/* 阻塞运行到结束。返回 0=正常退出, 2=已发 0x5F01 请求重新自检(调用方应 exec 重启自身) */
int  fsm_run(const fsm_cfg_t *cfg);

void fsm_request_button(int long_press);   /* 供按键线程/信号处理调用 */
void fsm_request_recheck(void);            /* SIGUSR1 */
void fsm_request_quit(void);               /* SIGINT/SIGTERM */
void fsm_bind_cam(void);                   /* 把对齐+落盘挂到相机帧对回调 */
#endif
