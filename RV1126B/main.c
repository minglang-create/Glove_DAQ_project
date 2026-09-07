/* =============================================================================
 * main.c —— Glove_DAQ_RV1126B: 手套数据采集主程序(总装, 保持薄)
 *
 * 系统: 每只手套 = RV1126B(双 IMX415 从机 + SPI 主机) + STM32G474(传感器主控
 *       + SPI 从机 + XVS 时基)。生命周期/协议 = 《协议契约_V1 第四次修订》。
 * 模块: glove_link(SPI事务/协议) daq_fsm(生命周期) cam_pipeline(双摄H.265)
 *       align(帧↔CYCLE 自动标定) recorder(分段落盘) button(按键) glove_view(视图)
 *
 * 用法:  glove_daq_rv [选项]
 *   -D <dev>     spidev            默认 /dev/spidev0.0
 *   -S <hz>      SPI 速率          默认 5000000
 *   -G <c:l>     PA1(DATA_READY)   默认 0:4  (球K13=GPIO0_A4)
 *   -K <c:l>     按键 GPIO(低有效)  默认 0:0(SW3=球A2=GPIO0_A0); -1:-1=禁用
 *   -o <dir>     落盘基目录        默认 /mnt/sd/daq ("none"=不落盘)
 *   -F <sec>     周期 fsync 间隔    默认 5 (0=关; exFAT 不 fsync 拔卡会丢文件大小)
 *   -T <min>     按时间自动切段     默认 10 (0=关; 损伤隔离+可边录边拉)
 *   -M <gb>      SD 剩余空间阈值    默认 2.0 (0=不检查存储, 允许落 eMMC 调试)
 *   -U <dev>[:baud][:trig|free]  外接转接板 21 路关节 ADC 串口
 *                ★默认已启用 = /dev/ttyS0:460800:trig★  关闭: -U none
 *                串口打不开(如旧 boot 没让出 UART0)只告警不阻塞
 *   -w <view>    视图 hdr/imu/joint/tactile/all(默认 hdr)   -r <hz> 重绘率
 *   -V           STM32 假数据逐帧验收
 *   -A           台架直通(STM32 AUTOSTART=1 时: 见数据帧即启相机开跑)
 *   -X           不碰相机(纯链路调试)
 *   -W/-H/-b     相机宽/高/码率kbps(默认 1920/1080/10240)
 * 信号: USR2=模拟短按  USR1=发0x5F01重新自检并重启  INT/TERM=优雅退出
 * ============================================================================= */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <getopt.h>

#include "glove_link.h"
#include "glove_view.h"
#include "daq_fsm.h"
#include "button.h"

static char **g_argv;                     /* exec 重启自身用 */

static void on_sig(int s)
{
	switch (s) {
	case SIGUSR2: fsm_request_button(0); break;
	case SIGUSR1: fsm_request_recheck(); break;
	default:      fsm_request_quit();
	}
}
static void on_btn(int lp, void *u) { (void)u; fsm_request_button(lp); }

int main(int argc, char *argv[])
{
	const char *spidev = "/dev/spidev0.0", *rec_dir = "/mnt/sd/daq";
	const char *view = "hdr";
	uint32_t hz = 5000000;
	/* 按键 = SW3 → SoM 球 A2 = 芯片 GPIO0_A0(_Z 无默认拉, 代码里开内部上拉;
	 * 按下对地 = 低有效)。2026-09-03 用户确认映射。 */
	int pa1_c = 0, pa1_l = 4, btn_c = 0, btn_l = 0, vhz = 15;
	fsm_cfg_t cfg;
	memset(&cfg, 0, sizeof(cfg));
	cfg.rec.fsync_sec = 5; cfg.rec.rotate_min = 10; cfg.rec.min_free_gb = 2.0;   /* SD 落盘默认 */
	cfg.ext_dev = "/dev/ttyS0"; cfg.ext_baud = 460800; cfg.ext_trig = 1;         /* 外接 ADC 串口默认开(2026-09-07) */
	g_argv = argv;

	int ch, a, b;
	while ((ch = getopt(argc, argv, "D:S:G:K:o:w:r:W:H:b:F:T:M:U:VAXh")) != -1) {
		switch (ch) {
		case 'D': spidev = optarg; break;
		case 'S': hz = (uint32_t)strtoul(optarg, NULL, 0); break;
		case 'G': if (sscanf(optarg, "%d:%d", &a, &b) == 2) { pa1_c = a; pa1_l = b; } break;
		case 'K': if (sscanf(optarg, "%d:%d", &a, &b) == 2) { btn_c = a; btn_l = b; } break;
		case 'o': rec_dir = strcmp(optarg, "none") ? optarg : NULL; break;
		case 'w': view = optarg; break;
		case 'r': vhz = atoi(optarg); break;
		case 'W': cfg.cam.width = atoi(optarg); break;
		case 'H': cfg.cam.height = atoi(optarg); break;
		case 'b': cfg.cam.bitrate_kbps = atoi(optarg); break;
		case 'U': {
			if (strcmp(optarg, "none") == 0) { cfg.ext_dev = NULL; break; }   /* -U none = 关闭 */
			static char ubuf[128]; snprintf(ubuf, sizeof(ubuf), "%s", optarg);
			char *dev = strtok(ubuf, ":"), *bd = strtok(NULL, ":"), *md = strtok(NULL, ":");
			cfg.ext_dev = dev; cfg.ext_baud = bd ? atoi(bd) : 460800;
			cfg.ext_trig = (md && strcmp(md, "free") == 0) ? 0 : 1;
			break;
		}
		case 'F': cfg.rec.fsync_sec = atoi(optarg); break;
		case 'T': cfg.rec.rotate_min = atoi(optarg); break;
		case 'M': cfg.rec.min_free_gb = atof(optarg); break;
		case 'V': cfg.verify_fake = 1; break;
		case 'A': cfg.auto_mode = 1; break;
		case 'X': cfg.no_cam = 1; break;
		default:
			printf("见 main.c 头注释\n");
			return ch == 'h' ? 0 : 1;
		}
	}
	cfg.rec_dir = rec_dir;
	cfg.cam.codec_h265 = 1;
	setvbuf(stdout, NULL, _IOLBF, 0);

	printf("== Glove_DAQ_RV1126B (协议契约V1第四次修订) pid=%d ==\n", getpid());
	if (!cfg.no_cam && access("/dev/mpi/vsys", F_OK) != 0) {
		printf("★/dev/mpi 缺失: rockit 模块没加载 → 相机会启动失败★\n");
		printf("  先执行:  sh /oem/usr/ko/insmod_ko.sh   然后重跑本程序\n");
		printf("  (纯链路调试可加 -X 跳过相机)\n");
	}
	if (glove_selftest() != 0) return 1;
	if (gv_set_mode(view) != 0) { printf("-w 只能 hdr/imu/joint/tactile/all\n"); return 1; }
	gv_set_hz(vhz);
	if (glove_open(spidev, hz, pa1_c, pa1_l) != 0) return 1;
	if (!cfg.no_cam) fsm_bind_cam();
	btn_start(btn_c, btn_l, on_btn, NULL);

	signal(SIGINT, on_sig); signal(SIGTERM, on_sig);
	signal(SIGUSR1, on_sig); signal(SIGUSR2, on_sig);

	int rc = fsm_run(&cfg);

	btn_stop();
	glove_close();
	if (rc == 2) {                          /* 0x5F01 → 干净地重启自己(全新自检) */
		printf("== 重新自检: exec 重启 ==\n");
		execv("/proc/self/exe", g_argv);
		perror("execv");
	}
	printf("== 退出 ==\n");
	return rc;
}
