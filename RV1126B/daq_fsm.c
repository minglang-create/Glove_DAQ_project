/* =============================================================================
 * daq_fsm.c —— 生命周期状态机实现(协议契约 V1 第四次修订)
 * 线程模型: 本状态机独占手套链路(glove_link 非线程安全, 全部收发在此线程);
 *           相机帧对经 sink 线程回调(pair_cb)→ 只做对齐查表+落盘(无锁文件分工)。
 * ============================================================================= */
#include "daq_fsm.h"
#include "glove_link.h"
#include "glove_view.h"
#include "align.h"
#include "recorder.h"
#include "ext_uart.h"

#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <stdatomic.h>

/* ---------- 事件标志(按键线程/信号处理写, FSM 线程读) ---------- */
static volatile int g_ev_short, g_ev_long, g_ev_recheck, g_ev_quit;
static volatile int g_loop_break;      /* 借给 glove_read_frame 当"退出"旗, 用于打断阻塞 */

void fsm_request_button(int lp) { if (lp) g_ev_long = 1; else g_ev_short = 1; g_loop_break = 1; }
void fsm_request_recheck(void)  { g_ev_recheck = 1; g_loop_break = 1; }
void fsm_request_quit(void)     { g_ev_quit = 1; g_loop_break = 1; }

/* ---------- 小包分发(非运行态轮询用) ----------
 * 返回: 0=无事 1=收到 0xC301(role/hands 填好) 2=MISO 出现数据帧 */
static int g_stm_alive = 0;            /* 见过 0x5401 = STM32 自检完成 */
static uint32_t g_stm_bitmap = 0;
static int poll_dispatch(int *role, int *hands)
{
	uint16_t head, d[3];
	static uint16_t last_head = 0;         /* 协议"同包可重复读": 同一包会被连读到多次 */
	int r = glove_txn_poll(&head, d);
	if (r == 2) { last_head = 0; return 2; }
	if (r != 1) { if (r == 0) last_head = 0; return 0; }
	if (head == last_head && head != GLV_PKT_START_REQ)
		return 0;                       /* 同包重复: 只处理一次(0xC301 除外, 状态机自会去重) */
	last_head = head;
	switch (head) {
	case GLV_PKT_COMMTEST:
		printf("[fsm] STM32 通信测试包 ✓\n");
		break;
	case GLV_PKT_SELFTEST:
		g_stm_bitmap = ((uint32_t)d[0] << 16) | d[1];
		if (!g_stm_alive) {
			printf("[fsm] STM32 自检包: 异常位图=0x%08X 特殊=0x%04X%s\n",
			       g_stm_bitmap, d[2], (d[2] & 1) ? " [双手模式]" : "");
			glove_queue_pkt(GLV_PKT_VER_REQ, NULL);   /* 顺手查个版本 */
		}
		g_stm_alive = 1;
		break;
	case GLV_PKT_IAM_SLAVE:
		printf("[fsm] 本手套被判为【从机】(对侧已上任主机), 回 ACK\n");
		gv_set_role("从机");
		glove_queue_pkt(GLV_PKT_ACK, NULL);
		break;
	case GLV_PKT_ACK: {
		static int ack_n = 0;              /* 小包"可重复读"语义: 同包会连读到多次 */
		if (ack_n++ == 0) printf("[fsm] STM32 已确认(0x5301, 重复读不再刷屏)\n");
		break;
	}
	case GLV_PKT_VER_RSP:
		printf("[fsm] STM32 版本: 协议v%u 固件%u.%u\n", d[0], d[1], d[2]);
		break;
	case GLV_PKT_START_REQ:
		if (role)  *role  = d[0];
		if (hands) *hands = d[1];
		return 1;
	default:
		printf("[fsm] 未处理小包 0x%04X\n", head);
	}
	return 0;
}

/* ---------- 相机帧对回调(sink 线程): 对齐 + 落盘 + 视图 ---------- */
static void pair_cb(const cam_pair_t *p, void *user)
{
	(void)user;
	uint32_t cyc = 0; int64_t resid = 0;
	int ok = (align_lookup(p->pts0, &cyc, &resid) == 0);
	rec_on_pair(p, ok, cyc, resid);
	gv_cam_pair(p->pair_seq, (long long)p->dpts_us);
}

static uint64_t now_ms(void)
{
	struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000 + (uint64_t)t.tv_nsec / 1000000;
}

int fsm_run(const fsm_cfg_t *cfg)
{
	unsigned seg = 0;
	int role = 0, hands = -1, cam_on = 0;
	enum { ST_WAIT_READY, ST_WAIT_FRAME, ST_RUNNING, ST_PAUSED } st = ST_WAIT_READY;

	/* ============ SELFCHECK(契约 1.1: 只探相机在位, 不启动) ============ */
	printf("══════ 自检 ══════\n");
	if (cfg->ext_dev) {
		if (ext_uart_open(cfg->ext_dev, cfg->ext_baud, cfg->ext_trig) == 0)
			glove_set_edge_cb(ext_uart_on_edge, NULL);   /* PA1 沿 → 触发对端(SPI 之前) */
		else
			printf("[fsm] ★外接串口打不开, 本次无外接关节数据★(控制台是否已让出 UART0? 见 dts 变体)\n");
	}
	if (!cfg->no_cam) {
		int n = cam_probe_i2c();
		printf("[fsm] 相机在位: %d/2 %s\n", n, n == 2 ? "✓" : "★缺相机, 继续跑但记录在案★");
	}
	/* 存储自检: 落盘目标必须在 SD 卡上且剩余空间够。失败 = 自检未完成 →
	 * 不发 0x5501(STM32 LED 不亮, 工人可见)、拒绝开始采集; 之后每秒复检, 插卡即自动恢复。 */
	int storage_ok = 1; char why[256];
	if (cfg->rec_dir) {
		if (rec_check_storage(cfg->rec_dir, cfg->rec.min_free_gb, why, sizeof(why)) != 0) {
			storage_ok = 0;
			printf("[fsm] ★存储自检失败: %s★\n", why);
			printf("      → 视为自检未完成: 不发 0x5501, 拒绝开始采集; 插好 SD 卡后自动恢复\n");
		} else if (rec_open(cfg->rec_dir, &cfg->rec) != 0) {
			storage_ok = 0;
			printf("[fsm] ★落盘目录不可用★ 拒绝开始采集\n");
		}
	}
	if (storage_ok) {
		glove_queue_pkt(GLV_PKT_RV_OK, NULL);      /* 0x5501: RV 自检完成, 尽早发 */
		printf("[fsm] 已发 0x5501(RV 自检完成)。等待: STM32 自检包 / 按键 / 0xC301\n");
		printf("      (按键短按=宣告主机; 无按键脚时 kill -USR2 %d 模拟)\n", getpid());
	}

	uint64_t t_resend = now_ms(), t_frame0 = 0;

	while (!g_ev_quit) {
		/* ---------- 通用事件 ---------- */
		if (g_ev_long) { printf("[fsm] 长按 → 结束\n"); break; }
		if (g_ev_recheck) {
			printf("[fsm] 发 0x5F01 重新自检, 进程将重启\n");
			glove_queue_pkt(GLV_PKT_RECHECK, NULL);
			uint16_t h, d[3]; glove_txn_poll(&h, d);   /* 立刻投递 */
			rec_close(); if (cam_on) cam_stop();
			return 2;
		}

		switch (st) {
		/* ============ 等就绪: 自检包/按键/0xC301 ============ */
		case ST_WAIT_READY: {
			int r = poll_dispatch(&role, &hands);
			/* 存储未就绪: 每秒复检, 插好卡即恢复并补发 0x5501 */
			if (!storage_ok && now_ms() - t_resend > 1000) {
				t_resend = now_ms();
				if (rec_check_storage(cfg->rec_dir, cfg->rec.min_free_gb, why, sizeof(why)) == 0 &&
				    rec_open(cfg->rec_dir, &cfg->rec) == 0) {
					storage_ok = 1;
					glove_queue_pkt(GLV_PKT_RV_OK, NULL);
					printf("[fsm] 存储就绪 ✓ 已发 0x5501(RV 自检完成)\n");
				}
			}
			/* 0x5501 无回执 → STM32 未活时每秒重发(幂等), 见到 0x5401 后再补 3 次停 */
			static int extra = 3;
			if (storage_ok && now_ms() - t_resend > 1000 && (!g_stm_alive || extra-- > 0)) {
				glove_queue_pkt(GLV_PKT_RV_OK, NULL);
				t_resend = now_ms();
			}
			if (g_ev_short) {
				g_ev_short = 0; g_loop_break = 0;
				printf("[fsm] 按键 → 发 0x5101(宣告本手套为主机)\n");
				glove_queue_pkt(GLV_PKT_IAM_MASTER, NULL);
			}
			int go = (r == 1);
			if (cfg->auto_mode && r == 2) {            /* 台架: STM32 直通已在吐数据帧 */
				printf("[fsm] -A 台架直通: 检测到数据帧\n");
				role = 1; hands = 0; go = 1;
			}
			if (go && !storage_ok) {
				printf("[fsm] ★收到开采请求但存储未就绪(%s), 拒绝★\n", why);
				for (int k = 0; k < 30 && !g_ev_quit && !g_ev_long; k++) {   /* 消费掉可重复读的 0xC301 */
					uint16_t h, dd[3]; glove_txn_poll(&h, dd); usleep(100*1000);
				}
				break;
			}
			if (go) {
				gv_set_role(role == 1 ? "主机" : "从机");
				printf("[fsm] 0xC301 开采请求: 角色=%s 模式=%s → 启动相机"
				       "(cam0=主机, 起振即向全系统发 XVS+XHS)\n",
				       role == 1 ? "主机" : "从机", hands == 1 ? "双手" : "单手");
				if (!cfg->no_cam) {
					if (cam_start(&cfg->cam) != 0) {
						cam_stop();          /* ★清干净半初始化的 ISP/VI, 否则重试更乱★ */
						printf("[fsm] ★相机启动失败★(常见: /dev/mpi 未加载 → 先 insmod_ko.sh)\n");
						printf("      冷却 3s 后可重试; 长按结束 / 0x5F01 重来\n");
						/* 冷却期把当前 0xC301 消费掉几拍, 避免"可重复读"导致的疯狂重试 */
						for (int k = 0; k < 30 && !g_ev_quit && !g_ev_long; k++) {
							uint16_t h, dd[3]; glove_txn_poll(&h, dd); usleep(100*1000);
						}
						break;
					}
					cam_on = 1;
				}
				glove_queue_pkt(GLV_PKT_START, NULL);  /* 0xC101 = 相机就绪回执 */
				t_frame0 = now_ms();
				st = ST_WAIT_FRAME;
				printf("[fsm] 已回 0xC101, 等第一个数据帧"
				       "(cam0 的 XVS 已起振, STM32 捕获到即逐拍装帧)…\n");
			} else {
				usleep(100 * 1000);
			}
			break;
		}

		/* ============ 等数据帧 = 采集开始的最终确认 ============ */
		case ST_WAIT_FRAME: {
			int r = poll_dispatch(NULL, NULL);
			if (r == 2) {
				seg++;
				align_reset();               /* 每次放号重新标定(XVS 重启过则网格换了) */
				if (cfg->rec_dir) rec_segment_start();
				glove_stats_reset();
				printf("══════ RUNNING(段 %u) ══════\n", seg);
				st = ST_RUNNING;
			} else if (now_ms() - t_frame0 > 40000) {
				printf("[fsm] ★40s 没等到数据帧★ 回就绪态(STM32 侧 30s 超时会自动回自检)\n");
				st = ST_WAIT_READY;
			} else {
				usleep(50 * 1000);
			}
			break;
		}

		/* ============ 运行: 沿驱动读数据帧 ============ */
		case ST_RUNNING: {
			static uint8_t raw[GLV_FRAME_LEN];
			static glove_frame_t f;
			static uint64_t t_stat = 0;
			if (cfg->rec_dir && rec_space_low()) {   /* flush 线程报空间不足 → 结束采集 */
				printf("[fsm] ★SD 卡空间不足, 停止采集★(换卡后重新上电)\n");
				g_ev_long = 1;
				break;
			}
			g_loop_break = 0;
			int err = glove_read_frame(&f, raw, &g_loop_break);
			if (err < 0) {                       /* 被事件打断或底层错 */
				if (g_ev_short) {
					g_ev_short = 0;
					printf("[fsm] 按键 → 暂停(0xC201, XVS 不停)\n");
					glove_queue_pkt(GLV_PKT_STOP, NULL);
					uint16_t h, d[3]; glove_txn_poll(&h, d);   /* 立即投递 */
					rec_segment_stop();
					st = ST_PAUSED;
				}
				break;                           /* 长按/退出/recheck 由循环头处理 */
			}
			if (err & GLV_ERR_MAGIC) {         /* 尸检: 前5个坏帧打头部 */
				static int autop = 0;
				if (autop++ < 5)
					printf("[fsm] [坏帧尸检] 头8字节: %02X %02X %02X %02X %02X %02X %02X %02X  PA1沿时间戳=%s\n",
					       raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6], raw[7],
					       f.t_edge_ns ? "有(沿触发)" : "无(电平兜底=PA1没在动)");
			}
			if (err == 0 || !(err & (GLV_ERR_MAGIC | GLV_ERR_LENGTH | GLV_ERR_CRC))) {
				align_on_glove(f.cycle, f.t_edge_ns);
				rec_on_glove(raw, f.cycle, f.t_edge_ns);
				if (ext_uart_enabled()) {          /* 本拍的外接 21 路关节(trig: 等回帧≤10ms) */
					ext_frame_t ef; ext_uart_get_last(&ef);
					rec_on_ext(f.cycle, &ef);
				}
				gv_on_frame(&f, (uint32_t)err, glove_stats());
				if (cfg->verify_fake) {
					glove_check_t ck;
					glove_check_fake(&f, &ck);
					if (ck.err)
						printf("[fsm] ★假数据校验失败★ cycle=%u: %s\n",
						       f.cycle, glove_err_str(ck.err));
				}
			}
			if (now_ms() - t_stat > 1000 && !gv_active()) {
				t_stat = now_ms();
				const glove_stats_t *s = glove_stats();
				double off_ms, std_us, drift; uint32_t n, slips;
				int locked = align_status(&off_ms, &std_us, &n, &drift, &slips);
				printf("[手套] %.1fHz 有效%llu/%llu cycle=%u 丢%llu crc错%llu "
				       "空帧%llu magic错%llu 小包%llu | [对齐] %s offset=%.2fms σ=%.0fµs "
				       "n=%u 漂移%+.1fµs/s%s\n",
				       s->rate_hz, (unsigned long long)s->ok,
				       (unsigned long long)s->reads, s->last_cycle,
				       (unsigned long long)s->cycle_dropped,
				       (unsigned long long)s->crc_err,
				       (unsigned long long)s->all_zero,
				       (unsigned long long)s->magic_err,
				       (unsigned long long)s->small_pkts,
				       locked ? "已标定" : "标定中", off_ms, std_us, n, drift,
				       slips ? " ★有滑移★" : "");
				if (cfg->rec_dir)
					printf("       [盘] 剩余 %.1fGB, 已开 %u 段\n", rec_free_gb(), rec_segment_count());
				if (ext_uart_enabled()) {
					const ext_stats_t *e = ext_uart_stats();
					printf("       [外接] 触发%llu 收%llu 缺%llu 迟弃%llu xor错%llu 重同步%llu 序号跳%llu 延时avg=%.2fms max=%.2fms\n",
					       (unsigned long long)e->trig_sent, (unsigned long long)e->frames_ok,
					       (unsigned long long)e->missing, (unsigned long long)e->late_drop,
					       (unsigned long long)e->xor_err, (unsigned long long)e->resync,
					       (unsigned long long)e->seq_gap, e->lat_avg_us / 1000.0, e->lat_max_us / 1000.0);
				}
			}
			break;
		}

		/* ============ 暂停: 轮询等恢复 ============ */
		case ST_PAUSED: {
			int r = poll_dispatch(NULL, NULL);
			if (g_ev_short) {
				g_ev_short = 0; g_loop_break = 0;
				printf("[fsm] 按键 → 恢复(0xC101)\n");
				glove_queue_pkt(GLV_PKT_START, NULL);
			}
			if (r == 2) {                        /* 数据帧回来了 = 恢复成功 */
				seg++;
				if (cfg->rec_dir) rec_segment_start();
				printf("══════ RUNNING(段 %u, 恢复) ══════\n", seg);
				st = ST_RUNNING;                 /* 对齐不重标: XVS 没停, 网格没变 */
			} else {
				usleep(200 * 1000);
			}
			break;
		}
		}
	}

	/* ============ SHUTDOWN ============ */
	printf("══════ 收尾 ══════\n");
	rec_close();
	ext_uart_close();
	if (cam_on) cam_stop();
	gv_finish();
	const glove_stats_t *s = glove_stats();
	printf("[fsm] 总账: 读%llu 有效%llu cycle %u→%u 丢%llu crc错%llu RUNNING次数%u 落盘段数%u\n",
	       (unsigned long long)s->reads, (unsigned long long)s->ok,
	       s->first_cycle, s->last_cycle,
	       (unsigned long long)s->cycle_dropped, (unsigned long long)s->crc_err, seg,
	       cfg->rec_dir ? rec_segment_count() : 0u);
	return 0;
}

/* pair 回调注册入口(main 在 cam 模块可用时调用) */
void fsm_bind_cam(void) { cam_set_pair_cb(pair_cb, NULL); }
