/* mdio —— 运行时读写 PHY 寄存器(含 RTL8211F 扩展页), 用 SIOCGMIIREG/SIOCSMIIREG
 * 用法: mdio <iface> r <page> <reg> | mdio <iface> w <page> <reg> <val>
 *   page=0 时不切页; RTL8211F 切页寄存器为 0x1f, 访问完自动切回页 0 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/mii.h>
#include <linux/sockios.h>

static int fd; static struct ifreq ifr; static struct mii_ioctl_data *m;
static int rd(int reg, int *out) { m->reg_num = reg; if (ioctl(fd, SIOCGMIIREG, &ifr) < 0) return -1; *out = m->val_out; return 0; }
static int wr(int reg, int val)  { m->reg_num = reg; m->val_in = val; return ioctl(fd, SIOCSMIIREG, &ifr) < 0 ? -1 : 0; }

int main(int argc, char **argv)
{
	if (argc < 5) { fprintf(stderr, "用法: %s <iface> r <page> <reg> | w <page> <reg> <val>\n", argv[0]); return 2; }
	const char *ifn = argv[1], *op = argv[2];
	int page = (int)strtol(argv[3], NULL, 0), reg = (int)strtol(argv[4], NULL, 0);
	fd = socket(AF_INET, SOCK_DGRAM, 0);
	memset(&ifr, 0, sizeof(ifr)); snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifn);
	if (ioctl(fd, SIOCGMIIPHY, &ifr) < 0) { perror("SIOCGMIIPHY"); return 1; }
	m = (struct mii_ioctl_data *)&ifr.ifr_data;
	int phy = m->phy_id, v;
	if (page && wr(0x1f, page) < 0) { perror("切页"); return 1; }
	if (op[0] == 'r') {
		if (rd(reg, &v) < 0) { perror("读"); return 1; }
		printf("phy%d page0x%x reg0x%02x = 0x%04x\n", phy, page, reg, v);
	} else {
		int val = (int)strtol(argv[5], NULL, 0);
		if (rd(reg, &v) < 0) { perror("读"); return 1; }
		if (wr(reg, val) < 0) { perror("写"); return 1; }
		int v2; rd(reg, &v2);
		printf("phy%d page0x%x reg0x%02x: 0x%04x -> 写0x%04x -> 回读0x%04x %s\n",
		       phy, page, reg, v, val, v2, v2 == val ? "✓" : "★不一致★");
	}
	if (page) wr(0x1f, 0);
	return 0;
}
