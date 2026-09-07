/* =============================================================================
 * glove_link.c —— 手套链路实现, 协议 v2(布局/事务模型见 glove_link.h + PROTOCOL_V2.md)
 * ============================================================================= */
#include "glove_link.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <linux/gpio.h>

/* ---------- 大端取/存 ---------- */
static inline uint16_t be16(const uint8_t *p) { return (uint16_t)((p[0] << 8) | p[1]); }
static inline uint32_t be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8)  | (uint32_t)p[3];
}
static inline void wbe16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)(v >> 8); p[1] = (uint8_t)v; }

/* ---------- CRC-16/IBM(=ARC): 字节流, 反射 0xA001, init 0 ---------- */
uint16_t glv_crc16_arc(const uint8_t *d, size_t n)
{
	uint16_t crc = 0x0000;
	for (size_t i = 0; i < n; i++) {
		crc ^= d[i];
		for (int b = 0; b < 8; b++)
			crc = (crc & 1) ? (uint16_t)((crc >> 1) ^ 0xA001) : (uint16_t)(crc >> 1);
	}
	return crc;
}

/* ---------- 内部状态 ---------- */
static int      g_fd      = -1;
static int      g_line_fd = -1;
static uint32_t g_hz      = 5000000;
static int      g_first   = 1;
static int      g_have_prev_cycle = 0;
static uint32_t g_prev_cycle = 0;

/* 待捎带的主→从小包 FIFO(深度8): 多个包各占一次事务的 MOSI 头部, 不覆盖不丢 */
#define GLV_TXQ_DEPTH 8
static uint8_t  g_txq[GLV_TXQ_DEPTH][GLV_PKT_LEN];
static int      g_txq_head = 0, g_txq_tail = 0;   /* head=下一个发, tail=下一个入 */

static glove_stats_t g_st;
static struct timespec g_rate_t0;
static uint32_t        g_rate_cnt;

static uint64_t now_us(void)
{
	struct timespec t;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000000ull + (uint64_t)t.tv_nsec / 1000ull;
}

/* ---------- 自检: CRC 向量 + 帧几何 + 小包几何 ---------- */
int glove_selftest(void)
{
	int bad = 0;
	uint16_t v = glv_crc16_arc((const uint8_t *)"123456789", 9);
	if (v != 0xBB3D) {
		printf("[glove] ★CRC 自检失败★ CRC(\"123456789\")=0x%04X, 应 0xBB3D\n", v);
		bad = 1;
	}
	if (GLV_OFF_CYCLE     != GLV_OFF_LENGTH + 2 ||
	    GLV_OFF_HEARTBEAT != GLV_OFF_CYCLE + 4 ||
	    GLV_OFF_JOINT     != GLV_OFF_HEARTBEAT + 4 ||
	    GLV_OFF_TAC_STAT  != GLV_OFF_JOINT + GLV_NUM_JOINT * 4 ||
	    GLV_OFF_TACTILE   != GLV_OFF_TAC_STAT + GLV_NUM_TAC * 2 ||
	    GLV_OFF_IMU_RAW   != GLV_OFF_TACTILE + GLV_TAC_WORDS * 2 ||
	    GLV_OFF_MAG_RAW   != GLV_OFF_IMU_RAW + 6 * 2 ||
	    GLV_OFF_QUAT      != GLV_OFF_MAG_RAW + 3 * 2 ||
	    GLV_OFF_CRC       != GLV_OFF_QUAT + 4 * 2 ||
	    GLV_FRAME_LEN     != GLV_OFF_CRC + 2 ||
	    GLV_LENGTH_WORDS  != GLV_FRAME_WORDS - 2) {
		printf("[glove] ★帧几何自检失败★ 偏移表自相矛盾, 对照 PROTOCOL_V2.md\n");
		bad = 1;
	}
	if (!bad)
		printf("[glove] 自检通过: CRC 向量 0xBB3D ✓, 协议v2 帧几何 %dB ✓ "
		       "(TACTILE 字下标 %d..%d, 事务=%d words)\n",
		       GLV_FRAME_LEN, GLV_OFF_TACTILE / 2,
		       GLV_OFF_TACTILE / 2 + GLV_TAC_WORDS - 1, GLV_FRAME_WORDS);
	return bad;
}

/* ---------- spidev ---------- */
static void check_bufsiz(void)
{
	FILE *f = fopen("/sys/module/spidev/parameters/bufsiz", "r");
	long v = 0;
	if (!f) return;
	if (fscanf(f, "%ld", &v) == 1 && v < GLV_FRAME_LEN)
		printf("[glove] ★spidev bufsiz=%ld < %d★ 事务会失败! 加内核参数 spidev.bufsiz=8192\n",
		       v, GLV_FRAME_LEN);
	fclose(f);
}

static int spi_setup(const char *dev, uint32_t hz)
{
	uint8_t mode = SPI_MODE_0, bits = 8, lsb = 0;
	g_fd = open(dev, O_RDWR | O_CLOEXEC);
	if (g_fd < 0) { printf("[glove] 打不开 %s: %s\n", dev, strerror(errno)); return -1; }
	if (ioctl(g_fd, SPI_IOC_WR_MODE, &mode) < 0 ||
	    ioctl(g_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 ||
	    ioctl(g_fd, SPI_IOC_WR_LSB_FIRST, &lsb) < 0 ||
	    ioctl(g_fd, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0) {
		printf("[glove] SPI 参数设置失败: %s\n", strerror(errno));
		close(g_fd); g_fd = -1;
		return -1;
	}
	g_hz = hz;
	printf("[glove] %s 就绪: Mode0/MSB/8bit @%.2fMHz, 事务 %dB ≈%.2fms\n",
	       dev, hz / 1e6, GLV_FRAME_LEN, GLV_FRAME_LEN * 8.0 / hz * 1000.0);
	return 0;
}

/* ---------- PA1(GPIO uAPI v2, 上升沿+电平兜底) ---------- */
static uint64_t g_last_edge_ns = 0;   /* 最近一次触发读的沿时间戳(对齐引擎的原料) */

static int gpio_setup(int chip, int line)
{
	char path[32];
	snprintf(path, sizeof(path), "/dev/gpiochip%d", chip);
	int cfd = open(path, O_RDONLY | O_CLOEXEC);
	if (cfd < 0) { printf("[glove] 打不开 %s: %s\n", path, strerror(errno)); return -1; }

	struct gpio_v2_line_request req;
	memset(&req, 0, sizeof(req));
	req.offsets[0] = (uint32_t)line;
	req.num_lines  = 1;
	snprintf(req.consumer, sizeof(req.consumer), "glove_daqV4");
	req.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_RISING;
	if (ioctl(cfd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) {
		printf("[glove] PA1 申请失败(gpiochip%d line%d): %s\n", chip, line, strerror(errno));
		close(cfd);
		return -1;
	}
	close(cfd);
	g_line_fd = req.fd;
	return 0;
}

static int gpio_level(void)
{
	struct gpio_v2_line_values v = { .mask = 1, .bits = 0 };
	if (ioctl(g_line_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0) return -1;
	return (int)(v.bits & 1);
}

int glove_open(const char *spidev, uint32_t hz, int gpiochip, int gpioline)
{
	check_bufsiz();
	if (spi_setup(spidev, hz) != 0) return -1;
	if (gpio_setup(gpiochip, gpioline) != 0) { close(g_fd); g_fd = -1; return -1; }
	printf("[glove] DATA_READY(PA1)=gpiochip%d:%d(当前电平=%d) 协议v2\n",
	       gpiochip, gpioline, gpio_level());
	clock_gettime(CLOCK_MONOTONIC, &g_rate_t0);
	return 0;
}

void glove_close(void)
{
	if (g_line_fd >= 0) { close(g_line_fd); g_line_fd = -1; }
	if (g_fd >= 0) { close(g_fd); g_fd = -1; }
}

/* ---------- 小包构造/解析 ---------- */
static void pkt_build(uint8_t *buf, uint16_t head, const uint16_t data[3])
{
	wbe16(buf + 0, head);
	wbe16(buf + 2, GLV_PKT_LENGTH_VAL);
	for (int i = 0; i < 3; i++)
		wbe16(buf + 4 + i * 2, data ? data[i] : 0xFFFF);
	wbe16(buf + 10, glv_crc16_arc(buf, GLV_PKT_LEN - 2));
}

/* 返回: 1=合法小包 0=不是小包(可能空/数据包) -1=像小包但CRC错 */
static int pkt_parse(const uint8_t *buf, uint16_t *head, uint16_t data[3])
{
	uint16_t h = be16(buf + 0);
	switch (h) {
	case GLV_PKT_COMMTEST: case GLV_PKT_IAM_SLAVE: case GLV_PKT_ACK:
	case GLV_PKT_SELFTEST: case GLV_PKT_VER_RSP: case GLV_PKT_START_REQ:
		break;                                /* 已知的从→主小包 */
	default:
		return 0;
	}
	if (be16(buf + 2) != GLV_PKT_LENGTH_VAL) return -1;
	if (glv_crc16_arc(buf, GLV_PKT_LEN - 2) != be16(buf + 10)) return -1;
	if (head) *head = h;
	if (data)
		for (int i = 0; i < 3; i++) data[i] = be16(buf + 4 + i * 2);
	return 1;
}

int glove_queue_pkt(uint16_t head, const uint16_t data[3])
{
	int nxt = (g_txq_tail + 1) % GLV_TXQ_DEPTH;
	if (nxt == g_txq_head) {                        /* 满(极少见): 丢最旧腾位 */
		printf("[glove] ★发送队列满, 丢弃最旧包★\n");
		g_txq_head = (g_txq_head + 1) % GLV_TXQ_DEPTH;
	}
	pkt_build(g_txq[g_txq_tail], head, data);
	g_txq_tail = nxt;
	return 0;
}

void glove_set_cmd(uint16_t head)   /* 兼容旧 -C */
{
	if (head) glove_queue_pkt(head, NULL);
}

/* ---------- 一次固定长度全双工事务 ---------- */
/* ★协议硬约束★ 相邻两次事务之间 ≥100µs(STM32 重装 DMA 的窗口),
 * 背靠背连读会读到未装载的垃圾 → 这里强制补足间隔, 上层不用操心。 */
static uint64_t g_last_txn_end_us = 0;
#define GLV_TXN_GAP_US 100

static int spi_txn(uint8_t *rx)
{
	static uint8_t tx[GLV_FRAME_LEN];
	uint64_t now = now_us();
	if (g_last_txn_end_us && now - g_last_txn_end_us < GLV_TXN_GAP_US)
		usleep(GLV_TXN_GAP_US - (unsigned)(now - g_last_txn_end_us));
	memset(tx, 0, sizeof(tx));                 /* 全 0 = NOP */
	if (g_txq_head != g_txq_tail) {           /* 队列非空: 取一个包捎带在事务头部 */
		memcpy(tx, g_txq[g_txq_head], GLV_PKT_LEN);
		g_txq_head = (g_txq_head + 1) % GLV_TXQ_DEPTH;
	}
	struct spi_ioc_transfer tr;
	memset(&tr, 0, sizeof(tr));
	tr.tx_buf        = (unsigned long)(uintptr_t)tx;
	tr.rx_buf        = (unsigned long)(uintptr_t)rx;
	tr.len           = GLV_FRAME_LEN;
	tr.speed_hz      = g_hz;
	tr.bits_per_word = 8;
	tr.cs_change     = 0;                      /* ★整事务一个 CS 包络★ */
	int ok = (ioctl(g_fd, SPI_IOC_MESSAGE(1), &tr) == GLV_FRAME_LEN);
	g_last_txn_end_us = now_us();
	return ok ? 0 : -1;
}

int glove_txn_poll(uint16_t *head, uint16_t data[3])
{
	uint8_t rx[GLV_FRAME_LEN];
	if (spi_txn(rx) != 0) return -1;
	g_st.reads++;

	uint16_t h = be16(rx);
	if (h == 0x0000) { g_st.all_zero++; return 0; }        /* 从机没装包 */
	if (h == GLV_MAGIC) return 2;                          /* 数据包(本接口不解析) */
	int r = pkt_parse(rx, head, data);
	if (r == 1) { g_st.small_pkts++; return 1; }
	g_st.crc_err += (r == -1);
	g_st.magic_err += (r == 0);
	return -1;
}

/* ---------- 等 PA1 上升沿(首次电平兜底; 沿的内核时间戳供对齐引擎) ---------- */
static void (*g_edge_cb)(void *) = NULL;
static void *g_edge_ud = NULL;
void glove_set_edge_cb(void (*cb)(void *), void *user) { g_edge_cb = cb; g_edge_ud = user; }
static inline int ready(void) { if (g_edge_cb) g_edge_cb(g_edge_ud); return 0; }

static int wait_ready(volatile int *quit)
{
	g_last_edge_ns = 0;
	if (g_first) {
		g_first = 0;
		int lv = gpio_level();
		if (lv < 0) return -1;
		if (lv == 1) return ready();
	}
	int waited_ms = 0;
	while (!(quit && *quit)) {
		struct pollfd p = { .fd = g_line_fd, .events = POLLIN };
		int pr = poll(&p, 1, 200);
		if (pr < 0) { if (errno == EINTR) continue; return -1; }
		if (pr > 0 && (p.revents & POLLIN)) {
			struct gpio_v2_line_event ev[8];
			ssize_t n = read(g_line_fd, ev, sizeof(ev));
			if (n <= 0) return -1;
			/* 取最后一个沿的内核时间戳(积压时用最新的, 它对应我们将读的帧) */
			int cnt = (int)(n / sizeof(ev[0]));
			if (cnt > 0) g_last_edge_ns = ev[cnt - 1].timestamp_ns;
			return ready();               /* 沿到: 先触发外接设备, 再回去做 SPI */
		}
		waited_ms += 200;
		if (waited_ms % 1000 == 0) {
			if (gpio_level() == 1) return ready();
			printf("[glove] 等 PA1… 已 %ds (未开采? STM32/XVS 未跑?)\n", waited_ms / 1000);
			fflush(stdout);
		}
	}
	return -1;
}

/* ---------- 解析数据包(协议 v2, 全大端) ---------- */
static void parse(const uint8_t *b, glove_frame_t *f)
{
	f->magic     = be16(b + GLV_OFF_MAGIC);
	f->length    = be16(b + GLV_OFF_LENGTH);
	f->cycle     = be32(b + GLV_OFF_CYCLE);
	f->heartbeat = be32(b + GLV_OFF_HEARTBEAT);

	for (int i = 0; i < GLV_NUM_JOINT; i++) {
		uint32_t v = be32(b + GLV_OFF_JOINT + i * 4);
		f->joint_raw[i]     = v;
		f->joint_missing[i] = (v == GLV_JOINT_MISSING);
		f->joint_angle[i]   = v & GLV_JOINT_ANGLE_MASK;
	}
	for (int i = 0; i < GLV_NUM_TAC; i++)
		f->tac_stat[i] = be16(b + GLV_OFF_TAC_STAT + i * 2);

	const uint8_t *t = b + GLV_OFF_TACTILE;
	for (int s = 0; s < GLV_NUM_TAC; s++)
		for (int r = 0; r < GLV_TAC_ROWS; r++)
			for (int c = 0; c < GLV_TAC_COLS; c++)
				f->tac[s][r][c] =
					be16(t + ((s * GLV_TAC_ROWS + r) * GLV_TAC_COLS + c) * 2);

	for (int i = 0; i < 6; i++) f->imu_raw[i] = (int16_t)be16(b + GLV_OFF_IMU_RAW + i * 2);
	for (int i = 0; i < 3; i++) f->mag_raw[i] = (int16_t)be16(b + GLV_OFF_MAG_RAW + i * 2);
	for (int i = 0; i < 4; i++) f->quat[i]    = (int16_t)be16(b + GLV_OFF_QUAT + i * 2);

	f->crc_recv = be16(b + GLV_OFF_CRC);
	f->crc_calc = glv_crc16_arc(b, GLV_CRC_COVER);
}

int glove_read_frame(glove_frame_t *f, uint8_t *raw, volatile int *quit)
{
	uint8_t buf[GLV_FRAME_LEN];

	if (wait_ready(quit) != 0) return -1;
	if (spi_txn(buf) != 0) {
		printf("[glove] SPI 事务失败: %s\n", strerror(errno));
		return -1;
	}
	g_st.reads++;
	if (gpio_level() == 0) g_st.torn++;   /* 读期间 PA1 掉低: 撞上重建窗(疑撕帧) */

	if (raw) memcpy(raw, buf, GLV_FRAME_LEN);
	memset(f, 0, sizeof(*f));
	parse(buf, f);
	f->t_read_us = now_us();
	f->t_edge_ns = g_last_edge_ns;

	uint16_t h = be16(buf);
	if (h != GLV_MAGIC) {
		if (h == 0x0000) { g_st.all_zero++; return GLV_ERR_MAGIC; }
		if (pkt_parse(buf, NULL, NULL) == 1) {    /* 是合法小包(握手/自检阶段) */
			g_st.small_pkts++;
			return GLV_ERR_SMALLPKT;
		}
		g_st.magic_err++;
		return GLV_ERR_MAGIC;
	}

	uint32_t err = 0;
	if (f->length != GLV_LENGTH_WORDS)  { err |= GLV_ERR_LENGTH; g_st.len_err++; }
	if (f->crc_calc != f->crc_recv)     { err |= GLV_ERR_CRC;    g_st.crc_err++; }

	if (err == 0) {
		g_st.ok++;
		if (!g_st.first_cycle) g_st.first_cycle = f->cycle;
		g_st.last_cycle = f->cycle;

		if (g_have_prev_cycle) {
			uint32_t d = f->cycle - g_prev_cycle;
			if (d == 0) { g_st.cycle_dup++; err |= GLV_ERR_CYCLE; }
			else if (d > 1) {
				g_st.cycle_gap_ev++;
				g_st.cycle_dropped += (d - 1);
				err |= GLV_ERR_CYCLE;
			}
		}
		g_prev_cycle = f->cycle;
		g_have_prev_cycle = 1;

		g_rate_cnt++;
		struct timespec now;
		clock_gettime(CLOCK_MONOTONIC, &now);
		double el = (now.tv_sec - g_rate_t0.tv_sec) + (now.tv_nsec - g_rate_t0.tv_nsec) / 1e9;
		if (el >= 1.0) {
			g_st.rate_hz = g_rate_cnt / el;
			g_rate_cnt = 0;
			g_rate_t0 = now;
		}
	}
	return (int)err;
}

/* ---------- 假数据校验(PROTOCOL_V2.md §6) ---------- */
void glove_check_fake(const glove_frame_t *f, glove_check_t *out)
{
	memset(out, 0, sizeof(*out));
	out->tac_bad_idx = out->joint_bad_idx = out->stat_bad_idx = -1;

	if (f->magic != GLV_MAGIC)         out->err |= GLV_ERR_MAGIC;
	if (f->length != GLV_LENGTH_WORDS) out->err |= GLV_ERR_LENGTH;
	if (f->crc_calc != f->crc_recv)    out->err |= GLV_ERR_CRC;

	const uint16_t base = (uint16_t)(GLV_OFF_TACTILE / 2);   /* =51 */
	for (int s = 0; s < GLV_NUM_TAC && out->tac_bad_idx < 0; s++)
		for (int r = 0; r < GLV_TAC_ROWS && out->tac_bad_idx < 0; r++)
			for (int c = 0; c < GLV_TAC_COLS; c++) {
				int k = (s * GLV_TAC_ROWS + r) * GLV_TAC_COLS + c;
				uint16_t want = (uint16_t)(base + k);
				if (f->tac[s][r][c] != want) {
					out->err |= GLV_ERR_TACTILE;
					out->tac_bad_idx = k;
					out->tac_expect = want;
					out->tac_got = f->tac[s][r][c];
					break;
				}
			}

	for (int i = 0; i < GLV_NUM_JOINT; i++) {
		uint32_t want = ((f->cycle & 0xFFFu) + ((uint32_t)i << 12)) & 0x7FFFFFu;
		if (f->joint_angle[i] != want) {
			out->err |= GLV_ERR_JOINT;
			out->joint_bad_idx = i;
			out->joint_expect = want;
			out->joint_got = f->joint_angle[i];
			break;
		}
	}

	for (int i = 0; i < GLV_NUM_TAC; i++) {
		uint16_t want = (uint16_t)(0xA500 | i);
		if (f->tac_stat[i] != want) {
			out->err |= GLV_ERR_TAC_STAT;
			out->stat_bad_idx = i;
			out->stat_expect = want;
			out->stat_got = f->tac_stat[i];
			break;
		}
	}

	for (int i = 0; i < 6; i++)
		if (f->imu_raw[i] != (int16_t)(1000 + i)) { out->err |= GLV_ERR_IMU_RAW; break; }
	for (int i = 0; i < 3; i++)
		if (f->mag_raw[i] != (int16_t)(2000 + i)) { out->err |= GLV_ERR_MAG_RAW; break; }

	if (f->quat[0] != 16384) out->err |= GLV_ERR_QUAT;

	uint32_t must_online = 0x0Fu | (1u << GLV_HB_IMU_BIT);
	if (f->heartbeat & must_online) out->err |= GLV_ERR_HEARTBEAT;

	if (out->err) g_st.pattern_err++;
}

const glove_stats_t *glove_stats(void) { return &g_st; }

void glove_stats_reset(void)
{
	memset(&g_st, 0, sizeof(g_st));
	g_have_prev_cycle = 0;
	g_rate_cnt = 0;
	clock_gettime(CLOCK_MONOTONIC, &g_rate_t0);
}

const char *glove_err_str(uint32_t err)
{
	static char s[200];
	s[0] = 0;
	if (!err) { snprintf(s, sizeof(s), "OK"); return s; }
	if (err & GLV_ERR_MAGIC)     strncat(s, "MAGIC ",     sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_LENGTH)    strncat(s, "LENGTH ",    sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_CRC)       strncat(s, "CRC ",       sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_CYCLE)     strncat(s, "CYCLE ",     sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_TACTILE)   strncat(s, "TACTILE ",   sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_JOINT)     strncat(s, "JOINT ",     sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_TAC_STAT)  strncat(s, "TAC_STAT ",  sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_QUAT)      strncat(s, "QUAT ",      sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_HEARTBEAT) strncat(s, "HEARTBEAT ", sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_IMU_RAW)   strncat(s, "IMU_RAW ",   sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_MAG_RAW)   strncat(s, "MAG_RAW ",   sizeof(s) - strlen(s) - 1);
	if (err & GLV_ERR_SMALLPKT)  strncat(s, "小包(非数据帧) ", sizeof(s) - strlen(s) - 1);
	return s;
}
