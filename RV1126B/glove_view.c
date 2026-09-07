/* =============================================================================
 * glove_view.c —— 终端可视化实现
 *
 * 【为什么 IMU 视图长这样】验证四元数数据对不对, 按"最灵敏"排序看这几个量:
 *   1) 模长 |q| —— 单位四元数必须 ≈1.000。Q14 换算错、字节序错、丢字段, 首先在这里
 *      露馅(比看数值好看不好看灵敏得多)。★这是格式正确性的第一道关★
 *   2) 欧拉角 —— 人能直觉核对: 手平放 → 接近 0;绕某轴转 90° → 对应角变 90°。
 *   3) 抖动(最近1秒 std) —— 静止时的噪声底; 数值突然变大=受扰/丢帧/解析错位。
 *   4) 波形(sparkline) —— 一眼看出"卡住不动"(直线)和"跳变毛刺"(尖峰)。
 * ============================================================================= */
#include "glove_view.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define HIST 60                    /* 波形/统计窗口: 60 帧 = 1 秒 @60Hz */

static gv_mode_e g_mode = GV_HDR;
static int       g_hz   = 15;
static int       g_painted = 0;    /* 是否已清屏过 */

/* IMU 历史(算 std + 画波形) */
static struct {
	double roll[HIST], pitch[HIST], yaw[HIST], norm[HIST];
	int    n, head;
} g_h;

/* 相机侧统计(视图模式下由相机线程存进来, 由视图渲染, 相机自己不打印) */
static struct { double fps0, fps1, fps_pair; unsigned pair_seq; long long dpts_us; } g_cam;

static const char *MODE_NAMES[] = { "hdr", "imu", "joint", "tactile", "all" };
static char g_role[16] = "未定";
void gv_set_role(const char *r) { snprintf(g_role, sizeof(g_role), "%s", r ? r : "未定"); }

int gv_set_mode(const char *name)
{
	for (size_t i = 0; i < sizeof(MODE_NAMES) / sizeof(MODE_NAMES[0]); i++)
		if (strcmp(name, MODE_NAMES[i]) == 0) { g_mode = (gv_mode_e)i; return 0; }
	return -1;
}
int gv_is_mode_name(const char *s)
{
	for (size_t i = 0; i < sizeof(MODE_NAMES) / sizeof(MODE_NAMES[0]); i++)
		if (strcmp(s, MODE_NAMES[i]) == 0) return 1;
	return 0;
}
const char *gv_mode_name(void) { return MODE_NAMES[g_mode]; }
void gv_set_hz(int hz) { if (hz > 0 && hz <= 60) g_hz = hz; }
int  gv_active(void) { return g_mode != GV_HDR; }

void gv_cam_fps(const char *label, double fps)
{
	if (strstr(label, "cam0"))      g_cam.fps0 = fps;
	else if (strstr(label, "cam1")) g_cam.fps1 = fps;
	else                            g_cam.fps_pair = fps;
}
void gv_cam_get(double *f0, double *f1, double *fp, long long *dp)
{
	if (f0) *f0 = g_cam.fps0; if (f1) *f1 = g_cam.fps1;
	if (fp) *fp = g_cam.fps_pair; if (dp) *dp = g_cam.dpts_us;
}
void gv_cam_pair(unsigned pair_seq, long long dpts_us)
{
	g_cam.pair_seq = pair_seq;
	g_cam.dpts_us = dpts_us;
}

/* ---------- 四元数 → 欧拉角(ZYX: yaw-pitch-roll) + 模长 ---------- */
static void quat_decode(const int16_t q[4], double *roll, double *pitch, double *yaw, double *norm)
{
	double w = q[0] / 16384.0, x = q[1] / 16384.0, y = q[2] / 16384.0, z = q[3] / 16384.0;
	double n = sqrt(w * w + x * x + y * y + z * z);
	*norm = n;
	if (n > 1e-9) { w /= n; x /= n; y /= n; z /= n; }   /* 先归一化再算角, 免受幅度误差影响 */

	*roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)) * 180.0 / M_PI;
	double s = 2.0 * (w * y - z * x);
	if (s > 1.0) s = 1.0;
	if (s < -1.0) s = -1.0;                            /* 万向锁边界钳位, 防 asin 出 NaN */
	*pitch = asin(s) * 180.0 / M_PI;
	*yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)) * 180.0 / M_PI;
}

/* ---------- 小工具: 均值/标准差/极值 + ASCII 波形 ---------- */
static void stat_of(const double *ring, int n, int head, double *mean, double *std,
                    double *mn, double *mx)
{
	if (n <= 0) { *mean = *std = *mn = *mx = 0; return; }
	double s = 0, s2 = 0;
	*mn = 1e18; *mx = -1e18;
	for (int i = 0; i < n; i++) {
		double v = ring[(head - 1 - i + HIST * 2) % HIST];
		s += v; s2 += v * v;
		if (v < *mn) *mn = v;
		if (v > *mx) *mx = v;
	}
	*mean = s / n;
	double var = s2 / n - (*mean) * (*mean);
	*std = var > 0 ? sqrt(var) : 0;
}

/* 把 ring 里最近 n 个值画成 8 级方块波形(自适应量程) */
static void sparkline(const double *ring, int n, int head, char *out, size_t cap)
{
	static const char *B[8] = { "▁", "▂", "▃", "▄", "▅", "▆", "▇", "█" };
	out[0] = 0;
	if (n <= 1) return;
	double mn = 1e18, mx = -1e18;
	for (int i = 0; i < n; i++) {
		double v = ring[(head - n + i + HIST * 2) % HIST];
		if (v < mn) mn = v;
		if (v > mx) mx = v;
	}
	double span = mx - mn;
	if (span < 1e-9) span = 1e-9;              /* 全平 → 全画最低格(一条直线, 正是"卡住"的样子) */
	for (int i = 0; i < n; i++) {
		double v = ring[(head - n + i + HIST * 2) % HIST];
		int lv = (int)((v - mn) / span * 7.999);
		if (lv < 0) lv = 0;
		if (lv > 7) lv = 7;
		if (strlen(out) + 4 >= cap) break;
		strcat(out, B[lv]);
	}
}

/* ---------- 各视图 ---------- */
static void home(void)
{
	if (!g_painted) {
		printf("\033[2J\033[?25l");   /* 首次: 清全屏 + 隐藏光标(免得光标在数字间乱跳) */
		g_painted = 1;
	}
	printf("\033[H");                      /* 光标回左上, 之后逐行 \033[K */
	printf(" 本手套角色: 【%s】\033[K\n", g_role);   /* 顶行常驻: 主机/从机一眼可见 */
}
#define L "\033[K"        /* 清到行尾: 防止上一帧的长内容残留 */

/* 每屏收尾: 统一页脚(相机侧统计) + ★清掉光标以下所有残留★
 * 这是"残留"的根治手段: 无论上一屏比这屏长多少, \033[J 都会抹干净。 */
static void tail(void)
{
	printf("%s\n", L);
	printf(" 相机  cam0 %.1ffps  cam1 %.1ffps  成对 %.1ffps  dpts=%+lldus  pair=%u%s\n",
	       g_cam.fps0, g_cam.fps1, g_cam.fps_pair, g_cam.dpts_us, g_cam.pair_seq, L);
	printf(" 视图 %s(-w 切换: hdr/imu/joint/tactile/all)   Ctrl-C 退出%s\n",
	       gv_mode_name(), L);
	printf("\033[J");                 /* ★清除光标以下全部内容: 残留终结者★ */
	fflush(stdout);
}

static void view_imu(const glove_frame_t *f, uint32_t err, const glove_stats_t *st)
{
	double roll, pitch, yaw, norm;
	quat_decode(f->quat, &roll, &pitch, &yaw, &norm);

	double m_r, s_r, n_r, x_r, m_p, s_p, n_p, x_p, m_y, s_y, n_y, x_y, m_n, s_n, n_n, x_n;
	stat_of(g_h.roll,  g_h.n, g_h.head, &m_r, &s_r, &n_r, &x_r);
	stat_of(g_h.pitch, g_h.n, g_h.head, &m_p, &s_p, &n_p, &x_p);
	stat_of(g_h.yaw,   g_h.n, g_h.head, &m_y, &s_y, &n_y, &x_y);
	stat_of(g_h.norm,  g_h.n, g_h.head, &m_n, &s_n, &n_n, &x_n);

	char w1[256], w2[256], w3[256];
	int nw = g_h.n < 48 ? g_h.n : 48;
	sparkline(g_h.roll,  nw, g_h.head, w1, sizeof(w1));
	sparkline(g_h.pitch, nw, g_h.head, w2, sizeof(w2));
	sparkline(g_h.yaw,   nw, g_h.head, w3, sizeof(w3));

	int imu_online = !(f->heartbeat & (1u << GLV_HB_IMU_BIT));

	home();
	printf("═══ IMU 视图 ═══ cycle=%-10u %.1fHz  IMU心跳=%s  帧校验=%s%s\n",
	       f->cycle, st->rate_hz, imu_online ? "在线" : "★掉线★",
	       err ? glove_err_str(err) : "OK", L);
	printf("%s\n", L);
	printf(" 四元数 Q14 原始   w=%+6d   x=%+6d   y=%+6d   z=%+6d%s\n",
	       f->quat[0], f->quat[1], f->quat[2], f->quat[3], L);
	printf(" 归一化            w=%+7.4f x=%+7.4f y=%+7.4f z=%+7.4f%s\n",
	       f->quat[0] / 16384.0 / (norm > 1e-9 ? norm : 1),
	       f->quat[1] / 16384.0 / (norm > 1e-9 ? norm : 1),
	       f->quat[2] / 16384.0 / (norm > 1e-9 ? norm : 1),
	       f->quat[3] / 16384.0 / (norm > 1e-9 ? norm : 1), L);
	/* ★模长是格式正确性的第一道关: Q14/字节序/字段错位都会让它偏离 1 ★ */
	printf(" 模长 |q| = %.5f   (单位四元数应 ≈1.000)  %s   最近1s: %.5f~%.5f%s\n",
	       norm, fabs(norm - 1.0) < 0.02 ? "✓" : "★偏离!检查Q14换算/字节序★",
	       n_n, x_n, L);
	printf("%s\n", L);
	printf(" 欧拉角(ZYX)   roll=%+8.2f°   pitch=%+8.2f°   yaw=%+8.2f°%s\n",
	       roll, pitch, yaw, L);
	printf(" 最近1s 抖动   std=%7.3f°     std=%7.3f°      std=%7.3f°%s\n", s_r, s_p, s_y, L);
	printf(" 最近1s 范围   %+.2f~%+.2f    %+.2f~%+.2f     %+.2f~%+.2f%s\n",
	       n_r, x_r, n_p, x_p, n_y, x_y, L);
	printf("%s\n", L);
	printf(" roll  %s%s\n", w1, L);
	printf(" pitch %s%s\n", w2, L);
	printf(" yaw   %s%s\n", w3, L);
	printf("%s\n", L);
	printf(" 帧统计: 有效 %llu/%llu  丢帧 %llu  CRC错 %llu  重复 %llu  撕帧 %llu%s\n",
	       (unsigned long long)st->ok, (unsigned long long)st->reads,
	       (unsigned long long)st->cycle_dropped, (unsigned long long)st->crc_err,
	       (unsigned long long)st->cycle_dup, (unsigned long long)st->torn, L);
	printf("%s\n", L);
	printf(" 验证手法: ①|q|≈1 → 格式对  ②手平放 roll/pitch≈0  ③绕轴转90°看对应角变90°%s\n", L);
	printf("           ④静止时 std 就是噪声底  ⑤波形变直线=数据卡住, 有尖峰=受扰/错帧%s\n", L);
	tail();
}

static void view_joint(const glove_frame_t *f, uint32_t err, const glove_stats_t *st)
{
	home();
	printf("═══ 关节视图 ═══ cycle=%-10u %.1fHz  帧校验=%s%s\n",
	       f->cycle, st->rate_hz, err ? glove_err_str(err) : "OK", L);
	printf(" (缺失=0xFFFFFFFF 显示 ----; 心跳位=1 表示该节点无数据)%s\n", L);
	printf("%s\n", L);
	for (int row = 0; row < 4; row++) {
		printf(" ");
		for (int col = 0; col < 5; col++) {
			int i = row * 5 + col;
			int hb_bad = (f->heartbeat >> GLV_HB_JOINT_BIT(i)) & 1;
			if (f->joint_missing[i])
				printf("J%-2d ------- %s ", i + 1, hb_bad ? "✗" : " ");
			else
				printf("J%-2d %7u %s ", i + 1, f->joint_angle[i], hb_bad ? "✗" : "✓");
		}
		printf("%s\n", L);
	}
	printf("%s\n", L);
	printf(" 心跳位图 0x%08X  触觉%s%s%s%s%s  IMU%s  磁力计%s%s\n", f->heartbeat,
	       (f->heartbeat >> GLV_HB_TAC_BIT(0)) & 1 ? "✗" : "✓",
	       (f->heartbeat >> GLV_HB_TAC_BIT(1)) & 1 ? "✗" : "✓",
	       (f->heartbeat >> GLV_HB_TAC_BIT(2)) & 1 ? "✗" : "✓",
	       (f->heartbeat >> GLV_HB_TAC_BIT(3)) & 1 ? "✗" : "✓",
	       (f->heartbeat >> GLV_HB_TAC_BIT(4)) & 1 ? "✗" : "✓",
	       (f->heartbeat >> GLV_HB_IMU_BIT) & 1 ? "✗" : "✓",
	       (f->heartbeat >> GLV_HB_MAG_BIT) & 1 ? "✗(未启用)" : "✓", L);
	printf(" 有效 %llu/%llu  丢帧 %llu  CRC错 %llu%s\n",
	       (unsigned long long)st->ok, (unsigned long long)st->reads,
	       (unsigned long long)st->cycle_dropped, (unsigned long long)st->crc_err, L);
	tail();
}

static void view_tactile(const glove_frame_t *f, uint32_t err, const glove_stats_t *st)
{
	static const char *GREY[5] = { " ", "░", "▒", "▓", "█" };   /* 5 级灰阶 */
	home();
	printf("═══ 触觉视图 ═══ cycle=%-10u %.1fHz  帧校验=%s%s\n",
	       f->cycle, st->rate_hz, err ? glove_err_str(err) : "OK", L);

	/* 每片各自算量程(自适应): 不同片受力差异大, 统一量程会让轻的一片全黑 */
	uint16_t mn[GLV_NUM_TAC], mx[GLV_NUM_TAC];
	double   avg[GLV_NUM_TAC];
	for (int s = 0; s < GLV_NUM_TAC; s++) {
		mn[s] = 0xFFFF; mx[s] = 0; double sum = 0;
		for (int r = 0; r < GLV_TAC_ROWS; r++)
			for (int c = 0; c < GLV_TAC_COLS; c++) {
				uint16_t v = f->tac[s][r][c];
				if (v < mn[s]) mn[s] = v;
				if (v > mx[s]) mx[s] = v;
				sum += v;
			}
		avg[s] = sum / GLV_TAC_CELLS;
	}
	printf(" 片#  状态字   min    max    均值   (灰阶按各片自身量程自适应)%s\n", L);
	for (int s = 0; s < GLV_NUM_TAC; s++)
		printf("  %d   0x%04X  %5u  %5u  %7.1f   心跳=%s%s\n", s + 1, f->tac_stat[s],
		       mn[s], mx[s], avg[s],
		       (f->heartbeat >> GLV_HB_TAC_BIT(s)) & 1 ? "✗无数据" : "✓", L);
	printf("%s\n", L);

	/* 5 片横向并排(每片 16 列 → 共 5*16+间隔 ≈ 95 列) */
	printf("      ");
	for (int s = 0; s < GLV_NUM_TAC; s++) printf("片%-14d ", s + 1);
	printf("%s\n", L);
	for (int r = 0; r < GLV_TAC_ROWS; r++) {
		printf(" %2d  ", r);
		for (int s = 0; s < GLV_NUM_TAC; s++) {
			int span = mx[s] - mn[s];
			for (int c = 0; c < GLV_TAC_COLS; c++) {
				int lv = span > 0 ? (int)((double)(f->tac[s][r][c] - mn[s]) / span * 4.999) : 0;
				if (lv < 0) lv = 0;
				if (lv > 4) lv = 4;
				printf("%s", GREY[lv]);
			}
			printf("  ");
		}
		printf("%s\n", L);
	}
	printf("%s\n", L);
	printf(" 有效 %llu/%llu  丢帧 %llu  CRC错 %llu   按压某点应看到对应格子变亮%s\n",
	       (unsigned long long)st->ok, (unsigned long long)st->reads,
	       (unsigned long long)st->cycle_dropped, (unsigned long long)st->crc_err, L);
	tail();
}

static void view_all(const glove_frame_t *f, uint32_t err, const glove_stats_t *st)
{
	double roll, pitch, yaw, norm;
	quat_decode(f->quat, &roll, &pitch, &yaw, &norm);
	home();
	printf("═══ 总览 ═══ cycle=%-10u %.1fHz  校验=%s  心跳=0x%08X%s\n",
	       f->cycle, st->rate_hz, err ? glove_err_str(err) : "OK", f->heartbeat, L);
	printf(" IMU  |q|=%.4f%s  roll=%+7.2f° pitch=%+7.2f° yaw=%+7.2f°%s\n",
	       norm, fabs(norm - 1.0) < 0.02 ? "✓" : "★", roll, pitch, yaw, L);
	printf(" 关节 ");
	for (int i = 0; i < GLV_NUM_JOINT; i++) {
		if (f->joint_missing[i]) printf("---- ");
		else printf("%4u ", f->joint_angle[i] & 0xFFFF);
		if (i == 9) printf("%s\n      ", L);
	}
	printf("%s\n", L);
	printf(" 触觉 ");
	for (int s = 0; s < GLV_NUM_TAC; s++) {
		double sum = 0; uint16_t mx = 0;
		for (int r = 0; r < GLV_TAC_ROWS; r++)
			for (int c = 0; c < GLV_TAC_COLS; c++) {
				sum += f->tac[s][r][c];
				if (f->tac[s][r][c] > mx) mx = f->tac[s][r][c];
			}
		printf("片%d(均%.0f 峰%u) ", s + 1, sum / GLV_TAC_CELLS, mx);
	}
	printf("%s\n", L);
	printf(" 统计 有效 %llu/%llu 丢帧 %llu CRC错 %llu 重复 %llu 撕帧 %llu%s\n",
	       (unsigned long long)st->ok, (unsigned long long)st->reads,
	       (unsigned long long)st->cycle_dropped, (unsigned long long)st->crc_err,
	       (unsigned long long)st->cycle_dup, (unsigned long long)st->torn, L);
	tail();
}

/* ---------- 入口: 每帧调用, 内部限速重绘 ---------- */
void gv_on_frame(const glove_frame_t *f, uint32_t err, const glove_stats_t *st)
{
	/* IMU 历史每帧都进(统计要全采样), 但画面按 g_hz 限速 */
	double roll, pitch, yaw, norm;
	quat_decode(f->quat, &roll, &pitch, &yaw, &norm);
	g_h.roll[g_h.head] = roll; g_h.pitch[g_h.head] = pitch;
	g_h.yaw[g_h.head] = yaw;   g_h.norm[g_h.head] = norm;
	g_h.head = (g_h.head + 1) % HIST;
	if (g_h.n < HIST) g_h.n++;

	if (g_mode == GV_HDR) return;          /* hdr 模式不刷屏(状态行由主线程每秒打) */

	static struct timespec last;
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	double el = (now.tv_sec - last.tv_sec) + (now.tv_nsec - last.tv_nsec) / 1e9;
	if (el < 1.0 / g_hz) return;
	last = now;

	switch (g_mode) {
	case GV_IMU:     view_imu(f, err, st);     break;
	case GV_JOINT:   view_joint(f, err, st);   break;
	case GV_TACTILE: view_tactile(f, err, st); break;
	case GV_ALL:     view_all(f, err, st);     break;
	default: break;
	}
}

void gv_finish(void)
{
	if (g_painted) {
		printf("\033[J\033[?25h\n");   /* 清残留 + 恢复光标显示 */
		g_painted = 0;
	}
	fflush(stdout);
}
