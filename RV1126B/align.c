/* =============================================================================
 * align.c —— 相机帧 ↔ CYCLE 对齐引擎(滑动窗跟踪版, 2026-09-04)
 *
 * 【为什么需要它】RV 手上两条流用两种语言标时间: 相机帧只有 PTS(µs 时间戳),
 *   手套帧只有 CYCLE(周期号), 没有共有字段。PA1 上升沿是唯一的"翻译中介":
 *   内核给它打的时间戳与相机 PTS 同一时钟, 而它触发读回的帧里写着 CYCLE
 *   → 每个沿产出一对 (CYCLE, T), 串成锚点表; 相机帧按 PTS 最近邻查表得 CYCLE。
 *
 * 【为什么从"一次标定"升级成"滑窗跟踪"】
 *   主机手套: 相机与 CYCLE 同源(都由本机 cam0 晶振驱动)→ offset 恒定, 标定一次够。
 *   从机手套(TXS 隔断方案): 相机跟【本机】cam0 晶振, CYCLE 跟【主机】cam0 晶振
 *   → 两颗独立晶振差几 ppm, 相位每秒漂几 µs、累积不封顶。固定 offset 半小时后
 *   偏差会越过半周期(8.3ms)判决门槛 → 指派开始出错。
 *   故: 锁定后持续用【窗内残差中位数】微调 offset, 让它跟着漂移走。
 *   同源情况下窗内中位数≈0 → offset 自然不动, 主机手套零副作用。
 *
 * 【健壮性设计】
 *   - 中位数而非均值: 个别异常帧(丢帧/调度尖峰)不会拖偏 offset;
 *   - 每次调整限幅(TRK_MAX_STEP_US): 一批坏样本也只能挪一点点;
 *   - 调整后把窗内旧残差整体平移: 保持统计连续, 且避免重复过冲;
 *   - 残差中位数越过 0.6 个半周期 = 已发生错拍指派 → 计一次 slip(可观测)。
 * ============================================================================= */
#include "align.h"
#include <math.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>

#define A_RING          256   /* 锚点环: 256 拍 ≈ 4.3s */
#define A_CALIB          90   /* 初始标定样本数(1.5s @60Hz), 取中位数 */
#define R_WIN           300   /* 残差滑动窗(5s): 供跟踪与 σ 统计 */
#define TRK_EVERY        60   /* 每 60 帧(≈1s)做一次跟踪微调 */
#define TRK_MAX_STEP_US 500   /* 单次调整上限: 漂移是 µs/s 级, 500µs 绰绰有余 */
#define A_PERIOD_US   16667   /* 标称周期(60Hz); 只用于"半周期"门槛判断 */
#define A_HALF_US   (A_PERIOD_US / 2)

static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;

/* 锚点表: PA1 沿时间戳 ↔ 该拍 CYCLE */
static struct { uint32_t cycle; uint64_t t_us; } g_ring[A_RING];
static int      g_n = 0, g_head = 0;

/* 初始标定 */
static int64_t  g_calib[A_CALIB];
static int      g_ncal = 0;
static int64_t  g_offset_us = 0;
static int      g_locked = 0;

/* 滑窗跟踪 + 统计 */
static int64_t  g_rwin[R_WIN];
static int      g_rn = 0, g_rhead = 0;
static uint32_t g_since_trk = 0;
static uint64_t g_last_trk_us = 0;
static double   g_drift_us_per_s = 0;
static uint32_t g_trk_updates = 0, g_slips = 0;
static uint64_t g_resid_n = 0;        /* 累计查表次数 */

void align_reset(void)
{
	pthread_mutex_lock(&g_mtx);
	g_n = g_head = g_ncal = g_locked = 0;
	g_offset_us = 0;
	g_rn = g_rhead = 0;
	g_since_trk = 0; g_last_trk_us = 0;
	g_drift_us_per_s = 0;
	g_trk_updates = g_slips = 0;
	g_resid_n = 0;
	pthread_mutex_unlock(&g_mtx);
}

void align_on_glove(uint32_t cycle, uint64_t edge_ns)
{
	if (!edge_ns) return;                 /* 电平兜底路径无沿时间戳, 不入锚点 */
	pthread_mutex_lock(&g_mtx);
	g_ring[g_head].cycle = cycle;
	g_ring[g_head].t_us  = edge_ns / 1000ull;
	g_head = (g_head + 1) % A_RING;
	if (g_n < A_RING) g_n++;
	pthread_mutex_unlock(&g_mtx);
}

static int cmp64(const void *a, const void *b)
{
	int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
	return (x > y) - (x < y);
}

/* 取窗内中位数(不破坏原窗) */
static int64_t win_median(void)
{
	static int64_t tmp[R_WIN];
	if (g_rn <= 0) return 0;
	memcpy(tmp, g_rwin, (size_t)g_rn * sizeof(tmp[0]));
	qsort(tmp, (size_t)g_rn, sizeof(tmp[0]), cmp64);
	return tmp[g_rn / 2];
}

int align_lookup(uint64_t pts_us, uint32_t *cycle, int64_t *residual_us)
{
	pthread_mutex_lock(&g_mtx);
	if (g_n == 0) { pthread_mutex_unlock(&g_mtx); return -1; }

	/* 最近邻: 找使 |pts - 锚点 - offset| 最小的锚点 */
	int64_t best = INT64_MAX; uint32_t bc = 0;
	for (int i = 0; i < g_n; i++) {
		int idx = (g_head - 1 - i + 2 * A_RING) % A_RING;
		int64_t d = (int64_t)pts_us - (int64_t)g_ring[idx].t_us - g_offset_us;
		if (llabs(d) < llabs(best)) { best = d; bc = g_ring[idx].cycle; }
		/* 锚点已比 pts 老 100ms 以上, 再往前扫没意义 */
		if ((int64_t)g_ring[idx].t_us + g_offset_us < (int64_t)pts_us - 100000) break;
	}

	if (!g_locked) {
		/* 标定期: 收集原始差(此时 offset=0, best 即原始差), 攒够取中位数锁定 */
		if (g_ncal < A_CALIB) {
			g_calib[g_ncal++] = best + g_offset_us;
			if (g_ncal == A_CALIB) {
				qsort(g_calib, A_CALIB, sizeof(int64_t), cmp64);
				g_offset_us = g_calib[A_CALIB / 2];
				g_locked = 1;
				g_last_trk_us = pts_us;
			}
		}
		pthread_mutex_unlock(&g_mtx);
		return -1;
	}

	/* ---- 已锁定: 最近锚点也隔了超过半周期 → 没有本拍的锚点(典型: 手套流已停、相机还在出帧,
	 * 或 RV 卡顿漏读了若干拍) → 归属不可靠, 不输出、不入统计, 由 recorder 记成 "-cycle"。 ---- */
	if (llabs(best) > A_HALF_US) { pthread_mutex_unlock(&g_mtx); return -1; }
	g_resid_n++;
	g_rwin[g_rhead] = best;
	g_rhead = (g_rhead + 1) % R_WIN;
	if (g_rn < R_WIN) g_rn++;

	/* ---- 滑窗跟踪: 每 TRK_EVERY 帧用窗内中位数微调 offset ---- */
	if (++g_since_trk >= TRK_EVERY && g_rn >= TRK_EVERY) {
		g_since_trk = 0;
		int64_t med = win_median();

		/* 中位数越过 0.6 个半周期 = 漂移已导致错拍指派(残差即将/已经翻转) */
		if (llabs(med) > A_HALF_US * 6 / 10) g_slips++;

		int64_t step = med;
		if (step >  TRK_MAX_STEP_US) step =  TRK_MAX_STEP_US;
		if (step < -TRK_MAX_STEP_US) step = -TRK_MAX_STEP_US;

		if (step) {
			g_offset_us += step;
			/* 漂移率(µs/s) = offset 变化 / 时间变化, EMA 平滑 */
			if (g_last_trk_us && pts_us > g_last_trk_us) {
				double dt_s = (double)(pts_us - g_last_trk_us) / 1e6;
				if (dt_s > 0.2) {
					double inst = (double)step / dt_s;
					g_drift_us_per_s = (g_trk_updates == 0)
						? inst : 0.8 * g_drift_us_per_s + 0.2 * inst;
				}
			}
			g_last_trk_us = pts_us;
			g_trk_updates++;
			/* offset 挪了 step, 窗内旧残差按同量平移: 统计连续 + 防重复过冲 */
			for (int i = 0; i < g_rn; i++) g_rwin[i] -= step;
		}
	}

	if (cycle) *cycle = bc;
	if (residual_us) *residual_us = best;
	pthread_mutex_unlock(&g_mtx);
	return 0;
}

int align_status(double *offset_ms, double *resid_std_us, uint32_t *n,
                 double *drift_us_per_s, uint32_t *slips)
{
	pthread_mutex_lock(&g_mtx);
	if (offset_ms)      *offset_ms = g_offset_us / 1000.0;
	if (n)              *n = (uint32_t)g_resid_n;
	if (drift_us_per_s) *drift_us_per_s = g_drift_us_per_s;
	if (slips)          *slips = g_slips;
	if (resid_std_us) {
		/* σ 取【窗内】而非累计: 反映当前抖动, 不受早期瞬态与跟踪台阶影响 */
		if (g_rn > 1) {
			double s = 0, s2 = 0;
			for (int i = 0; i < g_rn; i++) { s += (double)g_rwin[i]; s2 += (double)g_rwin[i] * g_rwin[i]; }
			double m = s / g_rn, v = s2 / g_rn - m * m;
			*resid_std_us = v > 0 ? sqrt(v) : 0;
		} else *resid_std_us = 0;
	}
	int lk = g_locked;
	pthread_mutex_unlock(&g_mtx);
	return lk;
}
