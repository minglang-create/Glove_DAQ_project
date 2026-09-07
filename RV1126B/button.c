#include "button.h"
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/gpio.h>

#define DEBOUNCE_MS 30
#define LONG_MS     2000

static int g_fd = -1;
static pthread_t g_tid;
static volatile int g_quit = 0;
static btn_cb g_cb; static void *g_user;

static int level(void)
{
	struct gpio_v2_line_values v = { .mask = 1, .bits = 0 };
	if (ioctl(g_fd, GPIO_V2_LINE_GET_VALUES_IOCTL, &v) < 0) return -1;
	return (int)(v.bits & 1);
}

static void *btn_thread(void *arg)
{
	(void)arg;
	while (!g_quit) {
		struct pollfd p = { .fd = g_fd, .events = POLLIN };
		if (poll(&p, 1, 200) <= 0) continue;
		struct gpio_v2_line_event ev[4];
		if (read(g_fd, ev, sizeof(ev)) <= 0) continue;
		usleep(DEBOUNCE_MS * 1000);
		if (level() != 0) continue;            /* 去抖后仍需为低(按下) */
		int ms = DEBOUNCE_MS;
		while (level() == 0 && ms < LONG_MS + 200 && !g_quit) { usleep(20000); ms += 20; }
		g_cb(ms >= LONG_MS, g_user);
		while (level() == 0 && !g_quit) usleep(20000);   /* 等松手 */
		struct gpio_v2_line_event drain[8];              /* 清掉松手产生的沿 */
		struct pollfd pd = { .fd = g_fd, .events = POLLIN };
		while (poll(&pd, 1, 0) > 0) if (read(g_fd, drain, sizeof(drain)) <= 0) break;
	}
	return NULL;
}

int btn_start(int chip, int line, btn_cb cb, void *user)
{
	if (chip < 0) { printf("[btn] 未配置按键脚(用 kill -USR2 <pid> 模拟短按)\n"); return 0; }
	char path[32];
	snprintf(path, sizeof(path), "/dev/gpiochip%d", chip);
	int cfd = open(path, O_RDONLY | O_CLOEXEC);
	if (cfd < 0) { printf("[btn] 打不开 %s\n", path); return -1; }
	struct gpio_v2_line_request req;
	memset(&req, 0, sizeof(req));
	req.offsets[0] = (uint32_t)line; req.num_lines = 1;
	strcpy(req.consumer, "daq_button");
	req.config.flags = GPIO_V2_LINE_FLAG_INPUT | GPIO_V2_LINE_FLAG_EDGE_FALLING |
	                   GPIO_V2_LINE_FLAG_BIAS_PULL_UP;   /* 低有效按键 */
	if (ioctl(cfd, GPIO_V2_GET_LINE_IOCTL, &req) < 0) { printf("[btn] 申请失败\n"); close(cfd); return -1; }
	close(cfd);
	g_fd = req.fd; g_cb = cb; g_user = user; g_quit = 0;
	pthread_create(&g_tid, NULL, btn_thread, NULL);
	printf("[btn] 按键就绪 gpiochip%d:%d(低有效, 长按%dms=落盘并重新自检)\n", chip, line, LONG_MS);
	return 0;
}

void btn_stop(void)
{
	if (g_fd < 0) return;
	g_quit = 1;
	pthread_join(g_tid, NULL);
	close(g_fd); g_fd = -1;
}
