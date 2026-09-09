/* 扫 MDIO 总线: 读每个地址的 PHY ID(reg2/reg3) */
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <net/if.h>
#include <linux/mii.h>
#include <linux/sockios.h>
int main(int argc, char **argv)
{
	int fd = socket(AF_INET, SOCK_DGRAM, 0);
	struct ifreq ifr; memset(&ifr, 0, sizeof(ifr));
	snprintf(ifr.ifr_name, IFNAMSIZ, "%s", argc > 1 ? argv[1] : "end0");
	if (ioctl(fd, SIOCGMIIPHY, &ifr) < 0) { perror("SIOCGMIIPHY"); return 1; }
	struct mii_ioctl_data *m = (struct mii_ioctl_data *)&ifr.ifr_data;
	printf("  驱动绑定的地址: %d\n", m->phy_id);
	for (int a = 0; a < 8; a++) {
		m->phy_id = a; m->reg_num = 2;
		if (ioctl(fd, SIOCGMIIREG, &ifr) < 0) { printf("  地址 %d: 读失败\n", a); continue; }
		int id1 = m->val_out;
		m->phy_id = a; m->reg_num = 3;
		ioctl(fd, SIOCGMIIREG, &ifr);
		int id2 = m->val_out;
		printf("  地址 %d: ID = 0x%04x%04x %s\n", a, id1, id2,
		       (id1 == 0xffff || id1 == 0) ? "(无器件)" : "★有 PHY★");
	}
	return 0;
}
