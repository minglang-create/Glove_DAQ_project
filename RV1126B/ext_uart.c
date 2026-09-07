/* =============================================================================
 * ext_uart.c —— 外接关节 ADC 串口链路实现(设计见 ext_uart.h)
 * 线程: RX 线程(收字节/成帧/校验/配对) + FSM 线程(触发、取帧)。共享态一把锁 + 条件变量。
 * ============================================================================= */
#include "ext_uart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <termios.h>
#include <pthread.h>

static int  g_fd = -1;
static int  g_trig = 1;
static pthread_t g_thr;
static volatile int g_run = 0;

static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  g_cv;
/* trig 模式: 当前待答触发; free 模式: 最新帧 */
static uint64_t    g_t_trig_us = 0;
static int         g_answered = 0;     /* 0=等待 1=已答 -1=超时已计缺失 */
static ext_frame_t g_frame;
static ext_stats_t g_st;
static int64_t     g_lat_ring[64]; static int g_lat_n = 0, g_lat_i = 0;
static uint16_t    g_last_seq = 0; static int g_have_seq = 0;

static uint64_t now_us(void)
{
	struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

static speed_t baud_const(int b)
{
	switch (b) {
	case 115200: return B115200; case 230400: return B230400;
	case 460800: return B460800; case 921600: return B921600;
	case 1500000: return B1500000; default: return 0;
	}
}

/* ---- 一帧到齐(RX 线程): 校验 → 配对/更新。返回 1=合法帧已处理 0=XOR 错(调用方前进 1 字节重同步) ---- */
static int on_frame(const uint8_t *b, uint64_t t)
{
	uint8_t x = 0;
	for (int i = 1; i <= 44; i++) x ^= b[i];
	if (x != b[45]) { g_st.xor_err++; return 0; }

	ext_frame_t f;
	f.valid = 1;
	f.seq   = (uint16_t)(b[1] | (b[2] << 8));
	for (int i = 0; i < EXT_NCH; i++) f.ch[i] = (uint16_t)(b[3 + i * 2] | (b[4 + i * 2] << 8));
	f.t_rx_us = t;

	pthread_mutex_lock(&g_mtx);
	if (g_have_seq && (uint16_t)(g_last_seq + 1) != f.seq) g_st.seq_gap++;
	g_last_seq = f.seq; g_have_seq = 1;

	if (g_trig) {
		int64_t lat = (int64_t)(t - g_t_trig_us);
		if (g_t_trig_us && g_answered == 0 && lat >= 0 && lat <= EXT_WINDOW_MS * 1000) {
			f.latency_us = lat;
			g_frame = f; g_answered = 1; g_st.frames_ok++;
			g_lat_ring[g_lat_i] = lat; g_lat_i = (g_lat_i + 1) % 64; if (g_lat_n < 64) g_lat_n++;
			if (lat > g_st.lat_max_us) g_st.lat_max_us = lat;
			pthread_cond_broadcast(&g_cv);
		} else {
			g_st.late_drop++;              /* 迟到/无人问 → 丢, 绝不顶到下一拍 */
		}
	} else {
		f.latency_us = 0;
		g_frame = f; g_st.frames_ok++;
		pthread_cond_broadcast(&g_cv);
	}
	pthread_mutex_unlock(&g_mtx);
	return 1;
}

static void *rx_thread(void *arg)
{
	(void)arg;
	uint8_t buf[256]; int n = 0;
	while (g_run) {
		struct pollfd p = { .fd = g_fd, .events = POLLIN };
		int pr = poll(&p, 1, 200);
		if (pr <= 0) continue;
		ssize_t r = read(g_fd, buf + n, sizeof(buf) - (size_t)n);
		if (r <= 0) continue;
		uint64_t t = now_us();
		n += (int)r;
		int i = 0;
		while (n - i >= EXT_FRAME_LEN) {
			if (buf[i] != EXT_HEAD) { i++; g_st.resync++; continue; }
			/* XOR 错 = 这个 0xA5 多半是数据里的假头 → 只前进 1 字节, 别吞掉紧随其后的真帧 */
			i += on_frame(buf + i, t) ? EXT_FRAME_LEN : 1;
		}
		/* 没成帧的尾巴挪到头部, 继续攒 */
		if (i > 0) { memmove(buf, buf + i, (size_t)(n - i)); n -= i; }
		if (n >= (int)sizeof(buf)) n = 0;      /* 不可能发生的溢出兜底 */
	}
	return NULL;
}

int ext_uart_open(const char *dev, int baud, int trig_mode)
{
	speed_t sp = baud_const(baud);
	if (!sp) { printf("[ext] 不支持的波特率 %d\n", baud); return -1; }
	g_fd = open(dev, O_RDWR | O_NOCTTY | O_CLOEXEC);
	if (g_fd < 0) { printf("[ext] 打不开 %s: %s\n", dev, strerror(errno)); return -1; }
	struct termios tio;
	if (tcgetattr(g_fd, &tio) != 0) { printf("[ext] tcgetattr: %s\n", strerror(errno)); close(g_fd); g_fd = -1; return -1; }
	cfmakeraw(&tio);                          /* 8N1 raw: 不做任何字符处理 */
	tio.c_cflag |= CLOCAL | CREAD;
	tio.c_cflag &= ~(CSTOPB | CRTSCTS);
	cfsetispeed(&tio, sp); cfsetospeed(&tio, sp);
	tio.c_cc[VMIN] = 1; tio.c_cc[VTIME] = 0;
	if (tcsetattr(g_fd, TCSANOW, &tio) != 0) { printf("[ext] tcsetattr: %s\n", strerror(errno)); close(g_fd); g_fd = -1; return -1; }
	tcflush(g_fd, TCIOFLUSH);

	pthread_condattr_t ca; pthread_condattr_init(&ca);
	pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
	pthread_cond_init(&g_cv, &ca); pthread_condattr_destroy(&ca);

	g_trig = trig_mode; memset(&g_st, 0, sizeof(g_st)); memset(&g_frame, 0, sizeof(g_frame));
	g_run = 1;
	if (pthread_create(&g_thr, NULL, rx_thread, NULL) != 0) { g_run = 0; close(g_fd); g_fd = -1; return -1; }
	printf("[ext] %s @%d 8N1, 模式=%s (%s)\n", dev, baud,
	       g_trig ? "trig" : "free",
	       g_trig ? "PA1 沿发 0x00 触发对端采样, 一问一答" : "对端自由发, 每拍取最新");
	return 0;
}

void ext_uart_close(void)
{
	if (g_run) { g_run = 0; pthread_join(g_thr, NULL); }
	if (g_fd >= 0) { close(g_fd); g_fd = -1; }
}

int ext_uart_enabled(void) { return g_fd >= 0; }

void ext_uart_on_edge(void *user)
{
	(void)user;
	if (g_fd < 0 || !g_trig) return;
	pthread_mutex_lock(&g_mtx);
	if (g_t_trig_us && g_answered == 0) g_st.missing++;   /* 上一拍没等到也没人取 */
	g_t_trig_us = now_us(); g_answered = 0;
	g_st.trig_sent++;
	pthread_mutex_unlock(&g_mtx);
	static const uint8_t z = 0x00;
	if (write(g_fd, &z, 1) != 1) { /* tty 写缓冲, 几乎不会失败 */ }
}

int ext_uart_get_last(ext_frame_t *out)
{
	memset(out, 0, sizeof(*out));
	if (g_fd < 0) return -1;
	pthread_mutex_lock(&g_mtx);
	if (g_trig) {
		if (g_t_trig_us == 0) { pthread_mutex_unlock(&g_mtx); return -1; }
		struct timespec dl;
		uint64_t dl_us = g_t_trig_us + (uint64_t)EXT_WINDOW_MS * 1000;
		dl.tv_sec = (time_t)(dl_us / 1000000ull); dl.tv_nsec = (long)((dl_us % 1000000ull) * 1000);
		while (g_answered == 0)
			if (pthread_cond_timedwait(&g_cv, &g_mtx, &dl) == ETIMEDOUT) break;
		if (g_answered == 1) { *out = g_frame; }
		else if (g_answered == 0) { g_answered = -1; g_st.missing++; }   /* 超时: 计一次缺失 */
	} else {
		if (g_frame.valid) {
			*out = g_frame;
			out->latency_us = (int64_t)(now_us() - g_frame.t_rx_us);   /* 帧龄 */
			if (out->latency_us > 100000) out->valid = 0;               /* >100ms 的旧帧不算 */
		}
	}
	pthread_mutex_unlock(&g_mtx);
	return out->valid ? 0 : -1;
}

const ext_stats_t *ext_uart_stats(void)
{
	pthread_mutex_lock(&g_mtx);
	double s = 0; for (int i = 0; i < g_lat_n; i++) s += (double)g_lat_ring[i];
	g_st.lat_avg_us = g_lat_n ? s / g_lat_n : 0;
	pthread_mutex_unlock(&g_mtx);
	return &g_st;
}
