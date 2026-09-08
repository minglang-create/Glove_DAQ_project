/* =============================================================================
 * recorder.c —— 分段落盘实现(设计说明见 recorder.h 头注释)
 * ============================================================================= */
#include "recorder.h"
#include "glove_link.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/statvfs.h>
#include <sys/mount.h>
#include <errno.h>
#include <dirent.h>

/* ---------- 受 g_mtx 保护的状态(热路径与 flush 线程共享) ---------- */
static pthread_mutex_t g_mtx = PTHREAD_MUTEX_INITIALIZER;
#define NF 6
static FILE    *g_f[NF];                  /* 0:cam0 1:cam1 2:pairs.csv 3:glove.bin 4:glove.csv 5:ext_joints.csv */
static uint64_t g_off[2], g_goff;
static int      g_active = 0;
static unsigned g_seg = 0;                /* 已开过的段数 */
static uint64_t g_seg_t0_ms = 0;          /* 当前段开始时刻(切段计时) */
static char     g_dir[400];               /* 当前段目录(落盘报告用) */
static char     g_prefix[24] = "000000000000";   /* 本次开机的虚拟时间 YYYYMMDDHHMM(目录名前缀) */
static char     g_names[NF][20] = { "cam0.h264", "cam1.h264", "pairs.csv", "glove.bin", "glove.csv", "ext_joints.csv" };
static char     g_namelist[128];                 /* "cam0.h264 cam1.h264 pairs.csv ..." 打印用 */
static uint64_t g_npair, g_nglove, g_next, g_next_ok;   /* 本段已写行数(落盘报告用) */

/* ---------- 只有 flush 线程/主线程碰的状态 ---------- */
static char      g_base[256];
static rec_cfg_t g_cfg;
static pthread_t g_thr;
static volatile int g_thr_run = 0;
static volatile int g_space_low = 0;
static volatile double g_free_gb = -1;

static uint64_t now_ms(void)
{
	struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)t.tv_sec * 1000 + (uint64_t)t.tv_nsec / 1000000;
}

/* ================= 虚拟时间(目录名前缀) =================
 * 板子没有 RTC, 日历时间不可信; 但落盘目录必须【跨上电不撞名、按名字能排先后】。
 * 规则(用户定): 板子自己的 /userdata/glove_vtime 记一个虚拟时间 YYYYMMDDHHMM 和写它时的
 * boot_id —— 它属于【这块采集板】(相当于记录本板一共开采过多少次), 不跟随 SD 卡走:
 *   - 文件为空/没有 → 用当前系统时间做种;
 *   - boot_id 与本次相同(长按/Ctrl-C 引起的 exec 重启)→ 同一次开机, 原样沿用, 段号接着数;
 *   - boot_id 不同(真的断电再上电)→ 在文件时间上 +10 小时;
 *   - 兜底: 算出的前缀若卡上已有同名目录(换过卡/重刷过固件把 userdata 清了)→ 继续 +10 小时直到不撞。
 * 目录名 = <前缀>_seg_<段号 3 位>, 例 202604131938_seg_001, 字典序 = 时间序。 */
static void vt_now(char out[24])
{
	time_t t = time(NULL); struct tm tm; gmtime_r(&t, &tm);
	strftime(out, 24, "%Y%m%d%H%M", &tm);
}
static int vt_add_hours(char vt[24], int hours)
{
	struct tm tm; memset(&tm, 0, sizeof(tm));
	int Y, M, D, h, m;
	if (sscanf(vt, "%4d%2d%2d%2d%2d", &Y, &M, &D, &h, &m) != 5) return -1;
	tm.tm_year = Y - 1900; tm.tm_mon = M - 1; tm.tm_mday = D; tm.tm_hour = h; tm.tm_min = m;
	time_t t = timegm(&tm) + (time_t)hours * 3600;
	gmtime_r(&t, &tm); strftime(vt, 24, "%Y%m%d%H%M", &tm);
	return 0;
}
static int prefix_used(const char *base, const char *prefix)
{
	DIR *d = opendir(base); if (!d) return 0;
	struct dirent *e; size_t n = strlen(prefix); int used = 0;
	while ((e = readdir(d)))
		if (strncmp(e->d_name, prefix, n) == 0 && strncmp(e->d_name + n, "_seg_", 5) == 0) { used = 1; break; }
	closedir(d); return used;
}
static unsigned max_seg_of(const char *base, const char *prefix)
{
	DIR *d = opendir(base); if (!d) return 0;
	struct dirent *e; size_t n = strlen(prefix); unsigned mx = 0;
	while ((e = readdir(d)))
		if (strncmp(e->d_name, prefix, n) == 0 && strncmp(e->d_name + n, "_seg_", 5) == 0) {
			unsigned v = (unsigned)atoi(e->d_name + n + 5); if (v > mx) mx = v;
		}
	closedir(d); return mx;
}
#define VTIME_PATH "/userdata/glove_vtime"   /* 板级状态: 跨上电保留; 完整重烧固件会清空(之后重新做种+防撞) */
static void vtime_init(const char *base)
{
	char bid[64] = "?", saved_vt[24] = "", saved_bid[64] = "";
	const char *path = VTIME_PATH;
	FILE *f = fopen("/proc/sys/kernel/random/boot_id", "r");
	if (f) { if (fgets(bid, sizeof(bid), f)) bid[strcspn(bid, "\n")] = 0; fclose(f); }
	f = fopen(path, "r");
	if (f) { if (fscanf(f, "%23s %63s", saved_vt, saved_bid) < 1) saved_vt[0] = 0; fclose(f); }

	if (saved_vt[0] && strcmp(saved_bid, bid) == 0) {
		snprintf(g_prefix, sizeof(g_prefix), "%s", saved_vt);      /* 同一次开机: 沿用 */
		printf("[rec] 虚拟时间 %s (同一次开机, 沿用)\n", g_prefix);
		return;
	}
	if (saved_vt[0] && vt_add_hours(saved_vt, 10) == 0) {
		snprintf(g_prefix, sizeof(g_prefix), "%s", saved_vt);      /* 新一次上电: +10h */
	} else {
		vt_now(g_prefix);                                          /* 首次/文件坏: 用当前时间做种 */
	}
	int bumps = 0;
	while (prefix_used(base, g_prefix) && bumps < 10000) { vt_add_hours(g_prefix, 10); bumps++; }   /* 防撞 */
	f = fopen(path, "w");
	if (f) { fprintf(f, "%s %s\n", g_prefix, bid); fflush(f); fsync(fileno(f)); fclose(f); }
	printf("[rec] 虚拟时间 %s (%s%s)\n", g_prefix,
	       saved_vt[0] ? "新一次上电, +10h" : "首次使用, 以当前时间做种",
	       bumps ? ", 已避开卡上同名目录" : "");
}

/* ================= 存储自检 ================= */
static double free_gb_of(const char *path)
{
	struct statvfs sv;
	if (statvfs(path, &sv) != 0) return -1;
	return (double)sv.f_bavail * (double)sv.f_frsize / (1024.0 * 1024.0 * 1024.0);
}

/* 在 /proc/mounts 里找覆盖 path 的最长挂载点; dev/mp 填结果. 找不到返回 -1.
 * 用字符串前缀匹配(而非 stat), 这样 base 目录尚未创建时也能判断它将落在哪个盘。 */
static int mount_of(const char *path, char *dev, size_t dl, char *mp, size_t ml)
{
	FILE *f = fopen("/proc/mounts", "r");
	if (!f) return -1;
	char line[512]; size_t best = 0; int found = 0;
	while (fgets(line, sizeof(line), f)) {
		char d[256], m[256];
		if (sscanf(line, "%255s %255s", d, m) != 2) continue;
		size_t n = strlen(m);
		/* m 是 path 的前缀, 且边界落在 '/' 上("/mnt/sd" 不能匹配 "/mnt/sdx") */
		if (strncmp(path, m, n) == 0 && (n == 1 || path[n] == '/' || path[n] == 0) && n >= best) {
			best = n; found = 1;
			snprintf(dev, dl, "%s", d); snprintf(mp, ml, "%s", m);
		}
	}
	fclose(f);
	return found ? 0 : -1;
}

/* 热插卡自愈: base 的父目录(/mnt/sd/daq → /mnt/sd)还没挂 SD, 而 /dev/mmcblk1p1 已出现
 * → 自己挂上(exfat, noatime)。开机后才插卡时没有任何自动挂载服务, 不做这步"插卡自动
 * 恢复"就是空话。护栏: 父目录本身已是某个挂载点(如 /userdata)时绝不往上覆盖挂载。 */
static int try_mount_sd(const char *base)
{
	char mp[256]; snprintf(mp, sizeof(mp), "%s", base);
	char *sl = strrchr(mp, '/');
	if (!sl || sl == mp) return -1;
	*sl = 0;
	struct stat st;
	if (stat("/dev/mmcblk1p1", &st) != 0 || !S_ISBLK(st.st_mode)) return -1;   /* 卡还没被内核认到 */
	char dev[256], cur[256];
	if (mount_of(mp, dev, sizeof(dev), cur, sizeof(cur)) == 0 && strcmp(cur, mp) == 0)
		return -1;                                          /* mp 已是挂载点, 不覆盖 */
	mkdir(mp, 0755);
	if (mount("/dev/mmcblk1p1", mp, "exfat", MS_NOATIME, NULL) != 0) {
		printf("[rec] 检测到 SD 卡但自动挂载 %s 失败: %s\n", mp, strerror(errno));
		return -1;
	}
	printf("[rec] 检测到 SD 卡, 已自动挂载 /dev/mmcblk1p1 → %s (exfat, noatime)\n", mp);
	return 0;
}

int rec_check_storage(const char *base, double min_free_gb, char *why, size_t wl)
{
	if (why && wl) why[0] = 0;
	if (min_free_gb <= 0) return 0;                         /* 显式关闭检查 */
	char dev[256], mp[256];
	if (mount_of(base, dev, sizeof(dev), mp, sizeof(mp)) != 0) {
		snprintf(why, wl, "找不到 %s 所在的挂载点", base); return -1;
	}
	if (strncmp(dev, "/dev/mmcblk1", 12) != 0 && try_mount_sd(base) == 0)
		mount_of(base, dev, sizeof(dev), mp, sizeof(mp));  /* 刚挂上, 重新定位 */
	if (strncmp(dev, "/dev/mmcblk1", 12) != 0) {
		snprintf(why, wl, "%s 落在 %s(%s) 而不是 SD 卡(/dev/mmcblk1*) —— SD 未挂载或未插卡",
		         base, dev, mp);
		return -1;
	}
	double fg = free_gb_of(mp);
	g_free_gb = fg;
	if (fg < 0) { snprintf(why, wl, "statvfs(%s) 失败", mp); return -1; }
	if (fg < min_free_gb) {
		snprintf(why, wl, "SD 卡剩余 %.1fGB < 阈值 %.1fGB", fg, min_free_gb); return -1;
	}
	return 0;
}

/* ================= 段文件开/关(不持锁, 慢操作) ================= */
static int open_seg_files(unsigned seg, FILE *out[NF], char *dir, size_t dl)
{
	snprintf(dir, dl, "%s/%s_seg_%03u", g_base, g_prefix, seg);
	if (mkdir(dir, 0755) != 0) { printf("[rec] 建段目录失败 %s\n", dir); return -1; }
	static const char *modes[NF] = { "wb", "wb", "w", "wb", "w", "w" };
	char p[400]; int ok = 1;
	for (int i = 0; i < NF; i++) {
		snprintf(p, sizeof(p), "%s/%s", dir, g_names[i]);
		out[i] = fopen(p, modes[i]);
		if (!out[i]) ok = 0;
	}
	if (!ok) {
		for (int i = 0; i < NF; i++) if (out[i]) { fclose(out[i]); out[i] = NULL; }
		printf("[rec] 段文件打开失败 %s\n", dir); return -1;
	}
	fprintf(out[2], "pair_seq,seq0,seq1,pts0,pts1,dpts_us,cycle,residual_us,off0,len0,off1,len1\n");
	fprintf(out[4], "cycle,edge_ns,off\n");
	fprintf(out[5], "cycle,seq,valid,latency_us");
	for (int i = 0; i < EXT_NCH; i++) fprintf(out[5], ",ch%d", i);
	fprintf(out[5], "\n");
	return 0;
}

/* fflush → fsync → fclose。fsync 让 exFAT 把文件大小写进目录项(拔卡后文件才完整可见) */
/* 落盘报告: fsync 之后统计, 打印出来的大小就是已经在卡上的大小 —— "落盘了什么"一目了然 */
static void seg_report(const char *dir, FILE *f[NF], uint64_t np, uint64_t ng, uint64_t ne, uint64_t ne_ok, double secs)
{
	const char (*names)[20] = g_names;
	double mb[NF] = {0};
	for (int i = 0; i < NF; i++) {
		struct stat st;
		if (f[i] && fstat(fileno(f[i]), &st) == 0) mb[i] = (double)st.st_size / (1024.0 * 1024.0);
	}
	printf("[rec] 段已落盘 %s  (时长 %.1f s, 共 %.1f MB)\n", dir, secs,
	       mb[0] + mb[1] + mb[2] + mb[3] + mb[4] + mb[5]);
	printf("      %-14s %8.1f MB   %-14s %8.1f MB   %-14s %llu 行\n",
	       names[0], mb[0], names[1], mb[1], names[2], (unsigned long long)np);
	printf("      %-14s %8.1f MB (%llu 帧)   %-14s %llu 行   %-14s %llu 行(有效 %llu)\n",
	       names[3], mb[3], (unsigned long long)ng, names[4], (unsigned long long)ng,
	       names[5], (unsigned long long)ne, (unsigned long long)ne_ok);
}
static void close_files(FILE *f[NF])          /* fflush + fsync(不 close, 让报告能 fstat) */
{
	for (int i = 0; i < NF; i++) { if (f[i]) { fflush(f[i]); fsync(fileno(f[i])); } }
}
static void close_files_final(FILE *f[NF])
{
	for (int i = 0; i < NF; i++) { if (f[i]) { fclose(f[i]); f[i] = NULL; } }
}

/* ================= 热路径(持锁时间 = 一次 fwrite 进页缓存) ================= */
void rec_on_pair(const cam_pair_t *p, int calib_ok, uint32_t cycle, int64_t residual_us)
{
	pthread_mutex_lock(&g_mtx);
	if (g_active) {
		fwrite(p->d0, 1, p->l0, g_f[0]);
		fwrite(p->d1, 1, p->l1, g_f[1]);
		fprintf(g_f[2], "%u,%u,%u,%llu,%llu,%lld,%s%u,%lld,%llu,%u,%llu,%u\n",
			p->pair_seq, p->seq0, p->seq1,
			(unsigned long long)p->pts0, (unsigned long long)p->pts1,
			(long long)p->dpts_us,
			calib_ok ? "" : "-",          /* 标定未完成的帧 cycle 前带负号标记 */
			cycle, (long long)residual_us,
			(unsigned long long)g_off[0], p->l0,
			(unsigned long long)g_off[1], p->l1);
		g_off[0] += p->l0; g_off[1] += p->l1;
		g_npair++;
	}
	pthread_mutex_unlock(&g_mtx);
}

void rec_on_ext(uint32_t cycle, const ext_frame_t *f)
{
	pthread_mutex_lock(&g_mtx);
	if (g_active) {
		fprintf(g_f[5], "%u,%u,%d,%lld", cycle, f->seq, f->valid, (long long)f->latency_us);
		for (int i = 0; i < EXT_NCH; i++)
			fprintf(g_f[5], f->valid ? ",%u" : ",", f->ch[i]);    /* 缺失拍: 通道列留空 */
		fputc('\n', g_f[5]);
		g_next++; if (f->valid) g_next_ok++;
	}
	pthread_mutex_unlock(&g_mtx);
}

void rec_on_glove(const uint8_t *raw, uint32_t cycle, uint64_t edge_ns)
{
	pthread_mutex_lock(&g_mtx);
	if (g_active) {
		fwrite(raw, 1, GLV_FRAME_LEN, g_f[3]);
		fprintf(g_f[4], "%u,%llu,%llu\n", cycle,
			(unsigned long long)edge_ns, (unsigned long long)g_goff);
		g_goff += GLV_FRAME_LEN;
		g_nglove++;
	}
	pthread_mutex_unlock(&g_mtx);
}

/* ================= 段控制(FSM 线程) ================= */
int rec_segment_start(void)
{
	if (rec_active()) rec_segment_stop();
	FILE *nf[NF] = {0}; char dir[400];
	pthread_mutex_lock(&g_mtx);
	unsigned seg = ++g_seg;
	pthread_mutex_unlock(&g_mtx);
	if (open_seg_files(seg, nf, dir, sizeof(dir)) != 0) return -1;
	pthread_mutex_lock(&g_mtx);
	memcpy(g_f, nf, sizeof(g_f));
	g_off[0] = g_off[1] = g_goff = 0;
	g_npair = g_nglove = g_next = g_next_ok = 0;
	snprintf(g_dir, sizeof(g_dir), "%s", dir);
	g_seg_t0_ms = now_ms();
	g_active = 1;
	pthread_mutex_unlock(&g_mtx);
	printf("[rec] 开段 %s\n      写入: %s\n", dir, g_namelist);
	return 0;
}

void rec_segment_stop(void)
{
	FILE *old[NF]; char dir[400]; uint64_t np, ng, ne, neok; double secs;
	pthread_mutex_lock(&g_mtx);
	if (!g_active) { pthread_mutex_unlock(&g_mtx); return; }
	g_active = 0;                          /* 先置 0: 热路径立刻停写 */
	memcpy(old, g_f, sizeof(old));
	memset(g_f, 0, sizeof(g_f));
	snprintf(dir, sizeof(dir), "%s", g_dir);
	np = g_npair; ng = g_nglove; ne = g_next; neok = g_next_ok;
	secs = (double)(now_ms() - g_seg_t0_ms) / 1000.0;
	pthread_mutex_unlock(&g_mtx);
	close_files(old);                      /* 慢操作(fsync)在锁外 */
	seg_report(dir, old, np, ng, ne, neok, secs);
	close_files_final(old);
}

/* ================= flush 线程: fsync / 切段 / 查空间 ================= */
static void do_fsync(void)
{
	int fds[NF]; int n = 0;
	pthread_mutex_lock(&g_mtx);
	if (g_active)
		for (int i = 0; i < NF; i++) {
			fflush(g_f[i]);                 /* libc 缓冲 → 页缓存(快) */
			fds[n++] = dup(fileno(g_f[i])); /* dup: 即使随后被 fclose, 副本 fd 仍有效 */
		}
	pthread_mutex_unlock(&g_mtx);
	for (int i = 0; i < n; i++) {          /* 真正等 SD 卡的慢活, 在锁外 */
		if (fds[i] >= 0) { fsync(fds[i]); close(fds[i]); }
	}
}

static void do_rotate(void)
{
	FILE *nf[NF] = {0}, *old[NF]; char dir[400];
	pthread_mutex_lock(&g_mtx);
	if (!g_active) { pthread_mutex_unlock(&g_mtx); return; }
	unsigned seg = g_seg + 1;
	pthread_mutex_unlock(&g_mtx);

	if (open_seg_files(seg, nf, dir, sizeof(dir)) != 0) return;   /* 开失败: 继续写旧段 */

	pthread_mutex_lock(&g_mtx);
	if (!g_active) {                       /* 开文件期间 FSM 已停段 → 放弃新段 */
		pthread_mutex_unlock(&g_mtx);
		for (int i = 0; i < NF; i++) if (nf[i]) fclose(nf[i]);
		rmdir(dir);
		return;
	}
	char odir[400]; uint64_t np, ng, ne, neok; double secs;
	memcpy(old, g_f, sizeof(old));
	memcpy(g_f, nf, sizeof(g_f));          /* 原子换段: 之后的记录整条落新段 */
	snprintf(odir, sizeof(odir), "%s", g_dir);
	np = g_npair; ng = g_nglove; ne = g_next; neok = g_next_ok;
	secs = (double)(now_ms() - g_seg_t0_ms) / 1000.0;
	g_off[0] = g_off[1] = g_goff = 0;
	g_npair = g_nglove = g_next = g_next_ok = 0;
	snprintf(g_dir, sizeof(g_dir), "%s", dir);
	g_seg = seg;
	g_seg_t0_ms = now_ms();
	pthread_mutex_unlock(&g_mtx);

	close_files(old);                      /* 旧段收口(fsync)在锁外 */
	seg_report(odir, old, np, ng, ne, neok, secs);
	close_files_final(old);
	printf("[rec] 切段 → %s\n      写入: %s\n", dir, g_namelist);
}

static void *flush_thread(void *arg)
{
	(void)arg;
	uint64_t last_fsync = now_ms(), last_space = 0;
	while (g_thr_run) {
		usleep(500 * 1000);
		uint64_t t = now_ms();

		if (g_cfg.fsync_sec > 0 && t - last_fsync >= (uint64_t)g_cfg.fsync_sec * 1000) {
			last_fsync = t;
			do_fsync();
		}
		if (g_cfg.rotate_min > 0) {
			pthread_mutex_lock(&g_mtx);
			int due = g_active && (t - g_seg_t0_ms >= (uint64_t)g_cfg.rotate_min * 60000);
			pthread_mutex_unlock(&g_mtx);
			if (due) do_rotate();
		}
		if (g_cfg.min_free_gb > 0 && t - last_space >= 2000) {
			last_space = t;
			double fg = free_gb_of(g_base);
			if (fg >= 0) {
				g_free_gb = fg;
				if (fg < g_cfg.min_free_gb && !g_space_low) {
					g_space_low = 1;
					printf("[rec] ★SD 剩余 %.1fGB < 阈值 %.1fGB, 请求停止采集★\n", fg, g_cfg.min_free_gb);
				}
			}
		}
	}
	return NULL;
}

/* ================= 生命周期 ================= */
int rec_open(const char *base, const rec_cfg_t *cfg)
{
	snprintf(g_base, sizeof(g_base), "%s", base);
	g_cfg = *cfg;
	snprintf(g_names[0], sizeof(g_names[0]), "cam0.%s", cfg->cam_h265 ? "h265" : "h264");
	snprintf(g_names[1], sizeof(g_names[1]), "cam1.%s", cfg->cam_h265 ? "h265" : "h264");
	snprintf(g_namelist, sizeof(g_namelist), "%s %s %s %s %s %s", g_names[0], g_names[1], g_names[2], g_names[3], g_names[4], g_names[5]);
	if (mkdir(g_base, 0755) != 0) {
		struct stat st;
		if (stat(g_base, &st) != 0 || !S_ISDIR(st.st_mode)) {
			printf("[rec] 建不了目录 %s\n", g_base); return -1;
		}
	}
	g_space_low = 0;
	g_free_gb = free_gb_of(g_base);
	vtime_init(g_base);
	pthread_mutex_lock(&g_mtx);
	g_seg = max_seg_of(g_base, g_prefix);       /* exec 重启后段号接着数, 不从 001 重来 */
	pthread_mutex_unlock(&g_mtx);
	if (g_seg) printf("[rec] 本次开机已有 %u 段, 下一段从 %03u 开始\n", g_seg, g_seg + 1);
	if (!g_thr_run) {
		g_thr_run = 1;
		if (pthread_create(&g_thr, NULL, flush_thread, NULL) != 0) {
			g_thr_run = 0; printf("[rec] flush 线程创建失败\n"); return -1;
		}
	}
	printf("[rec] 落盘 %s (剩余 %.1fGB; fsync 每 %ds, 切段每 %dmin, 空间阈值 %.1fGB)\n",
	       g_base, g_free_gb, g_cfg.fsync_sec, g_cfg.rotate_min, g_cfg.min_free_gb);
	return 0;
}

void rec_close(void)
{
	rec_segment_stop();
	if (g_thr_run) { g_thr_run = 0; pthread_join(g_thr, NULL); }
}

int rec_active(void)
{
	pthread_mutex_lock(&g_mtx); int a = g_active; pthread_mutex_unlock(&g_mtx); return a;
}
unsigned rec_segment_count(void)
{
	pthread_mutex_lock(&g_mtx); unsigned s = g_seg; pthread_mutex_unlock(&g_mtx); return s;
}
int    rec_space_low(void) { return g_space_low; }
double rec_free_gb(void)   { return g_free_gb; }
