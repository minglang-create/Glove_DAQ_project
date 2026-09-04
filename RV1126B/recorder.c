#include "recorder.h"
#include "glove_link.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <sys/stat.h>

static char  g_base[256];
static FILE *g_h[2], *g_pairs, *g_gbin, *g_gcsv;
static uint64_t g_off[2], g_goff;
static int   g_active = 0;

int rec_open(const char *base)
{
	snprintf(g_base, sizeof(g_base), "%s", base);
	if (mkdir(g_base, 0755) != 0) {
		struct stat st;
		if (stat(g_base, &st) != 0 || !S_ISDIR(st.st_mode)) {
			printf("[rec] 建不了目录 %s\n", g_base);
			return -1;
		}
	}
	return 0;
}

int rec_segment_start(unsigned seg)
{
	if (g_active) rec_segment_stop();
	struct timespec t; clock_gettime(CLOCK_BOOTTIME, &t);
	char dir[320];
	snprintf(dir, sizeof(dir), "%s/seg_%03u_%ld", g_base, seg, (long)t.tv_sec);
	if (mkdir(dir, 0755) != 0) { printf("[rec] 建段目录失败 %s\n", dir); return -1; }

	char p[384];
	snprintf(p, sizeof(p), "%s/cam0.h265", dir); g_h[0] = fopen(p, "wb");
	snprintf(p, sizeof(p), "%s/cam1.h265", dir); g_h[1] = fopen(p, "wb");
	snprintf(p, sizeof(p), "%s/pairs.csv", dir); g_pairs = fopen(p, "w");
	snprintf(p, sizeof(p), "%s/glove.bin", dir); g_gbin = fopen(p, "wb");
	snprintf(p, sizeof(p), "%s/glove.csv", dir); g_gcsv = fopen(p, "w");
	if (!g_h[0] || !g_h[1] || !g_pairs || !g_gbin || !g_gcsv) {
		printf("[rec] 段文件打开失败\n"); rec_segment_stop(); return -1;
	}
	fprintf(g_pairs, "pair_seq,seq0,seq1,pts0,pts1,dpts_us,cycle,residual_us,off0,len0,off1,len1\n");
	fprintf(g_gcsv, "cycle,edge_ns,off\n");
	g_off[0] = g_off[1] = g_goff = 0;
	g_active = 1;
	printf("[rec] 开段 %s\n", dir);
	return 0;
}

void rec_on_pair(const cam_pair_t *p, int calib_ok, uint32_t cycle, int64_t residual_us)
{
	if (!g_active) return;
	fwrite(p->d0, 1, p->l0, g_h[0]);
	fwrite(p->d1, 1, p->l1, g_h[1]);
	fprintf(g_pairs, "%u,%u,%u,%llu,%llu,%lld,%s%u,%lld,%llu,%u,%llu,%u\n",
		p->pair_seq, p->seq0, p->seq1,
		(unsigned long long)p->pts0, (unsigned long long)p->pts1,
		(long long)p->dpts_us,
		calib_ok ? "" : "-",              /* 标定未完成的帧 cycle 前带负号标记 */
		cycle, (long long)residual_us,
		(unsigned long long)g_off[0], p->l0,
		(unsigned long long)g_off[1], p->l1);
	g_off[0] += p->l0; g_off[1] += p->l1;
}

void rec_on_glove(const uint8_t *raw, uint32_t cycle, uint64_t edge_ns)
{
	if (!g_active) return;
	fwrite(raw, 1, GLV_FRAME_LEN, g_gbin);
	fprintf(g_gcsv, "%u,%llu,%llu\n", cycle,
		(unsigned long long)edge_ns, (unsigned long long)g_goff);
	g_goff += GLV_FRAME_LEN;
}

void rec_segment_stop(void)
{
	if (!g_active) return;
	g_active = 0;                          /* 先置0: sink 线程立刻停写 */
	FILE *f[5] = { g_h[0], g_h[1], g_pairs, g_gbin, g_gcsv };
	g_h[0] = g_h[1] = g_pairs = g_gbin = g_gcsv = NULL;
	for (int i = 0; i < 5; i++) if (f[i]) fclose(f[i]);
	printf("[rec] 段已收口\n");
}

void rec_close(void) { rec_segment_stop(); }
int  rec_active(void) { return g_active; }
