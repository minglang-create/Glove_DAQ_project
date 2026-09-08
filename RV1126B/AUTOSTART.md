# 开机自启机制 & 固化进固件(2026-09-05 定稿)

目标:**烧录固件后上电即可用** —— 无需任何手动配置,开机自动加载内核模块、
自动启动 `glove_daq_rv`,工人按一下按键就能采集。

---

## 一、启动链(实测厘清:此板是 systemd,不是 busybox init)

```
systemd(/sbin/init→systemd)
  └─ initd-seq.service (multi-user.target.wants, 已 enabled)
      └─ /usr/bin/run_initd.sh start
          └─ 顺序执行 /etc/init.d/S??*      ★只扫这一个目录★
              └─ S21appinit  读 /etc/.rkapp = "GLOVEDAQ"
                  └─ sh /oem/usr/bin/RkLunch-GLOVEDAQ.sh
                      ├─ insmod_ko.sh + clr_unready_dev   (模块与 sensor 解锁)
                      ├─ mount -o noatime /dev/mmcblk1p1 /mnt/sd  (SD 卡 = 落盘目标, exFAT)
                      ├─ 逃生口检查 /userdata/glove_noauto  (noauto 时上面两步照做)
                      └─ 循环启动 glove_daq_rv(退出后自动重起)
```

**两处纠正过的误解**:
1. `/oem/usr/etc/init.d/` **没有任何东西扫描它** —— common.mk 注释里"S99 会被
   RkLunch 执行"是错的(基于 busybox init 的假设)。自启脚本只有放 `/etc/init.d/` 才有效。
2. 光 insmod 还不够,**必须写 `clr_unready_dev`**,否则 rkaiq 枚举不到 sensor
   (取代 rkipc 后没人替我们做这件事)。两步都在 RkLunch-GLOVEDAQ.sh 里。

---

## 二、怎么固化进固件(官方 overlay 机制)

`/etc` 在 **rootfs 分区**,手动放的文件**一重烧固件就没了**(踩过:重烧后 `.rkapp`
被重置回 `DUALRTSP`,自启的 dual_cam 抢占相机,导致 glove_daq_rv 卡在 rkaiq 初始化)。

官方做法(来源:Luckfox wiki《SDK 镜像编译》+ `build.sh:3023 post_overlay`):

```bash
# build_firmware() 会执行:
rsync $(dirname .BoardConfig.mk)/overlay/$RK_POST_OVERLAY/*  →  rootfs 暂存目录
```

本仓已备好,一条命令装进 SDK:

```bash
cd RV1126B/board_rootfs_overlay && sh install_to_sdk.sh
# 它做两件事: ①拷 etc/.rkapp(内容 GLOVEDAQ)到 SDK 的 overlay/overlay-glove-daq/
#            ②在 .BoardConfig.mk 里设 export RK_POST_OVERLAY="overlay-glove-daq"
```

`RkLunch-GLOVEDAQ.sh` 和 `glove_daq_rv` 在 **OEM 分区**,由 app 的 Makefile 自动打包,
不需要 overlay。

---

## 三、完整出固件流程(★含一个 SDK 的坑★)

```bash
# 1) 编译应用(sudo: 打包目录属 root)
cd ~/Aura-sdk/project/app/Glove_DAQ_RV1126B_SDK/RV1126B && sudo make

# 2) ★必做:把产物同步到 app_out★
sudo cp -rfa ~/Aura-sdk/project/app/out/* ~/Aura-sdk/output/out/app_out/

# 3) 打包固件(overlay 在这一步被 rsync 进 rootfs)
cd ~/Aura-sdk && sudo ./build.sh firmware
# 内核有改动才需要: sudo ./build.sh kernel  (在 firmware 之前)
```

**第 2 步为什么必须手动做**:`__PACKAGE_RESOURCES` 从 `output/out/app_out/bin` 读文件,
而 app 的 Makefile 写的是 `project/app/out/bin`。本该衔接两者的
`$(call MAROC_COPY_PKG_TO_APP_OUTPUT, ...)`(`project/app/Makefile:23`)所调用的宏
**在整个 SDK 里没有定义**(`MAROC` 疑似 `MACRO` 拼错),`$(call)` 展开为空 → 断链。
漏了这步的症状:固件里是**上一次的旧程序**,新加的脚本根本不进去。

### 出完固件必须核对(别只信编译日志)

```bash
cat  ~/Aura-sdk/output/out/rootfs_glibc_rv1126b/etc/.rkapp          # 应为 GLOVEDAQ
ls -l ~/Aura-sdk/output/out/oem/usr/bin/{glove_daq_rv,RkLunch-GLOVEDAQ.sh}  # 大小/时间应是刚编的
ls -l ~/Aura-sdk/output/image/{rootfs.img,oem.img,update.img}       # 时间戳应是刚才
```

---

## 四、工人使用 & 调试逃生口

**工人**:插好 SD 卡 → 上电 → 程序自动跑起来 → **短按开采** → **长按 2 秒**:当前段收口+fsync
(打印"数据已落盘 ✓")→ 发 0x5F01 让 STM32 回自检 → 程序自我重启回待机(等价于重新上电,不用断电)
→ **再短按 = 重新开采**(新开一段)。采集中短按被忽略(防误触);按键永不退出程序;
真要结束 = 长按看到"数据已落盘 ✓"后直接断电。日志 `/userdata/glove_daq.log`。
**SD 卡不在或剩余 <2GB = 自检失败**:程序不发 0x5501(STM32 LED 不亮),拒绝开始采集;
插好卡后每秒复检、自动恢复。**拔卡/断电前先长按(等"数据已落盘 ✓")**(段收口时 fsync;运行中每 5 秒也 fsync,
最坏丢 5 秒)。数据在 `/mnt/sd/daq/seg_*/`,每 10 分钟自动切一段,已完成的段可边录边 adb pull。

**调试**(不想让它自启抢相机):

```bash
adb shell "touch /userdata/glove_noauto && reboot"   # 只加载模块, 不起程序
adb shell "rm /userdata/glove_noauto && reboot"      # 恢复自启
# 临时停一次(不重启):
adb shell "pkill -f RkLunch-GLOVEDAQ; killall -9 glove_daq_rv"
#   ★注意★ RkLunch 里是无限重起循环, 只 kill 程序会被自动拉起, 必须连脚本一起 kill
```

---

## 五、历史包袱(已清理)

- **S89insmod_ko.sh 已删除**:它的 insmod 职责并入 RkLunch-GLOVEDAQ.sh,
  且它被打包到无人扫描的 `/oem/usr/etc/init.d/`,留着只会造成"两套并行"的混乱
  (这正是之前 `.rkapp` 选择器 + S99 点火器并存时踩过的坑)。
- `/oem/usr/etc/init.d/S99dualrtsp.sh` 来自 cam_daq/dual_cam 的打包,落在无人扫描的
  目录里无害,不属于本仓库,不动它。
- `.rkapp` 哨兵值 `MANUAL`(对应 RkLunch 脚本不存在 → S21 静默跳过、什么都不启)
  仍可用于临时全禁自启,但现在有 `/userdata/glove_noauto` 更合适(模块照常加载)。


## 六、以太网(2026-09-08)

接口名是 **`end0`**(Debian/systemd 可预测命名,不是 eth0)。V4 板 RJ45 线序镜像反接,用户自制
纠正线后可协商 1Gbps/Full,但**板→PC 方向每一帧都被 PC 网卡判为错帧**(PC ReceivedPacketErrors
增量 = 板子发出帧数;板子发 ARP 后 PC 邻居表仍 Unreachable),PC→板方向干净(广播帧完整到达)。
**把网线两头对调后故障方向不变**,强制 100M、板侧关 EEE 均无改善 → 不是线缆某一对的方向问题,
而是板子发射侧进线缆的配对/极性(RJ45 磁座接法)本身不对,线缆很难补救;根治靠 V5 改线序。
Windows 侧注意:ARP 失败后会把地址缓存为 Unreachable 并暂停发 ARP(表现为"无法访问目标主机"),
排障时先 `Remove-NetNeighbor -IPAddress <板IP>`(管理员);建议关掉网卡"环保节能/节能乙太网路"。

直连没有 DHCP,板子用 NetworkManager 配了持久固定 IP(**与 PC "以太网"网卡的 192.168.1.100 同网段**):

```bash
nmcli con add type ethernet ifname end0 con-name lab ipv4.method manual ipv4.addresses 192.168.1.2/24 ipv6.method ignore
nmcli con up lab          # 配置文件 /etc/NetworkManager/system-connections/lab.nmconnection(rootfs, 重烧丢)
# 排障用: 强制 100M  nmcli con mod lab 802-3-ethernet.speed 100 802-3-ethernet.duplex full; 恢复 speed 0 duplex ''
```
PC 侧:`ping 192.168.1.2`、`ssh root@192.168.1.2`、`scp -r root@192.168.1.2:/mnt/sd/daq/<段> .`
(Windows 防火墙"公用网络"默认不回 ICMP,板子 ping PC 不通是正常的)。
⚠ 两只手套同时上网时 IP 会冲突——量产前要按板子分配不同地址(可按序列号派生,待做)。
