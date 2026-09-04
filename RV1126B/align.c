#include "align.h"
#include <math.h>
#include <pthread.h>
#include <string.h>
#include <stdlib.h>

#define A_RING   256          /* 锚点环: 256 帧 ≈ 4.3s 窗口 */
#define A_CALIB  90           /* 标定样本数(1.5s), 取中位数 */

static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
static struct { uint32_t cycle; uint64_t t_us; } g_ring[A_RING];
static int      g_n = 0, g_head = 0;
static int64_t  g_calib[A_CALIB];
static int      g_ncal = 0;
static int64_t  g_offset_us = 0;
static int      g_locked = 0;
static double   g_resid_sum = 0, g_resid_sum2 = 0;
static uint32_t g_resid_n = 0;

void align_reset(void)
{
	pthread_mutex_lock(&g_mtx);
	g_n = g_head = g_ncal = g_locked = 0;
	g_offset_us = 0;
	g_resid_sum = g_resid_sum2 = 0; g_resid_n = 0;
	pthread_mutex_unlock(&g_mtx);
}

void align_on_glove(uint32_t cycle, uint64_t edge_ns)
{
	if (!edge_ns) return;
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

int align_lookup(uint64_t pts_us, uint32_t *cycle, int64_t *residual_us)
{
	pthread_mutex_lock(&g_mtx);
	if (g_n == 0) { pthread_mutex_unlock(&g_mtx); return -1; }

	/* 找与 (pts - offset) 最近的锚点(锚点单调, 顺扫最近 A_RING 个即可) */
	int64_t best = INT64_MAX; uint32_t bc = 0; uint64_t bt = 0;
	for (int i = 0; i < g_n; i++) {
		int idx = (g_head - 1 - i + 2 * A_RING) % A_RING;
		int64_t d = (int64_t)pts_us - (int64_t)g_ring[idx].t_us - g_offset_us;
		if (llabs(d) < llabs(best)) { best = d; bc = g_ring[idx].cycle; bt = g_ring[idx].t_us; }
		if ((int64_t)g_ring[idx].t_us + g_offset_us < (int64_t)pts_us - 100000) break;
	}
	(void)bt;
	if (!g_locked) {
		/* 标定期: 收集 (pts - 最近锚点) 原始差, 攒够取中位数 */
		if (g_ncal < A_CALIB) {
			g_calib[g_ncal++] = best + g_offset_us;   /* 还原成原始差 */
			if (g_ncal == A_CALIB) {
				qsort(g_calib, A_CALIB, sizeof(int64_t), cmp64);
				g_offset_us = g_calib[A_CALIB / 2];
				g_locked = 1;
			}
		}
		pthread_mutex_unlock(&g_mtx);
		return -1;
	}
	g_resid_sum += (double)best; g_resid_sum2 += (double)best * best; g_resid_n++;
	if (cycle) *cycle = bc;
	if (residual_us) *residual_us = best;
	pthread_mutex_unlock(&g_mtx);
	return 0;
}

int align_status(double *offset_ms, double *resid_std_us, uint32_t *n)
{
	pthread_mutex_lock(&g_mtx);
	if (offset_ms) *offset_ms = g_offset_us / 1000.0;
	if (n) *n = g_resid_n;
	if (resid_std_us) {
		if (g_resid_n > 1) {
			double m = g_resid_sum / g_resid_n;
			double v = g_resid_sum2 / g_resid_n - m * m;
			*resid_std_us = v > 0 ? sqrt(v) : 0;
		} else *resid_std_us = 0;
	}
	int lk = g_locked;
	pthread_mutex_unlock(&g_mtx);
	return lk;
}
