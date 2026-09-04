# 开机自启机制 & 干净方案(2026-09-03)

## 真实机制(2026-09-03 实测厘清; ★此板是 systemd, 不是 busybox init★)
```
systemd(/sbin/init→systemd) → initd-seq.service (multi-user.target.wants, 已 enabled)
  → /usr/bin/run_initd.sh start
     → 顺序 . 执行 /etc/init.d/S??*   ★只扫这一个目录★
        ├─ S21appinit  ★选择器★  读 /etc/.rkapp → sh /oem/usr/bin/RkLunch-<内容>.sh
        │                        (.rkapp 缺失/空 → 默认 RKIPC_RV1126B)
        └─ S89insmod_ko.sh  我们的: 只 insmod, 不启应用
```
★重要纠正: `/oem/usr/etc/init.d/` 没有任何东西扫描它 —— common.mk 注释里"S99 会被
  RkLunch 执行"是错的(基于 busybox init 的假设)。之前 dualrtsp 自启靠的是 .rkapp
  选择器那条路, 不是 oem 下的 S99 点火器。**自启脚本必须放 /etc/init.d/(rootfs, 可写,
  掉电保留), 不是 /oem/usr/etc/init.d/。**
- `/etc/.rkapp` 与 `/etc/init.d/` 都在 **rootfs 分区**(烧 oem.img 改不到, 运行时改, 掉电保留;
  ★但重烧 rootfs.img 会覆盖★ → 量产要把 S89 纳入 rootfs 打包, 见下)。

## 踩过的坑: 两套自启并行, 互相不知道
旧方案既改 .rkapp(选择器路径), 又放 S99dualrtsp.sh 点火器(自己拉 dual_cam),
两套叠加 → "说不清谁启动了谁", 还和联调手动跑冲突。已拆除。

## 当前干净方案(联调阶段)
目标: **开机只铺地基(insmod 内核模块), 不自启任何应用**; 应用由人手动跑。
1. `/etc/init.d/S89insmod_ko.sh` —— 只调 insmod_ko.sh 加载 /dev/mpi 与 sensor .ko,
   不启动任何应用(编号 89 < 99, 先于任何应用)。逃生: touch /userdata/no_insmod。
2. `/etc/.rkapp = MANUAL` —— 哨兵值, 对应 RkLunch-MANUAL.sh 不存在 → S21 静默跳过,
   不启 rkipc 也不启我们的应用。(换句话说: 应用零自启)
3. 删掉旧的 /oem/usr/etc/init.d/S99dualrtsp.sh(本就没被执行); 不再需要 cam_daq_noauto 标记。

## ★量产注意★ S89 在 rootfs, 重烧 rootfs.img 会丢
联调期手动 adb push 到 /etc/init.d/ 即可(掉电保留)。要固化进固件, 需把 S89insmod_ko.sh
放进 rootfs overlay(SDK 的 project/../rootfs 覆盖层), 而非 oem 打包 —— 具体路径待查
SDK 的 rootfs 定制机制(通常在 sysdrv/rootfs 或 device/.../rootfs_overlay)。

结果: 每次上电 → /dev/mpi 自动就绪 → 直接手动 `glove_daq_rv ...` 即可, 不用再手敲 insmod。

## 将来量产要"开机自动跑 glove_daq_rv"时怎么做
两条路二选一(别再两套并行):
- (推荐)写 /oem/usr/bin/RkLunch-GLOVEDAQ.sh(解锁+起 glove_daq_rv), echo GLOVEDAQ > /etc/.rkapp。
  S89 仍负责 insmod。逃生口在 RkLunch 脚本头部判 /userdata/xxx_noauto。
- 或直接在 S89 之后加一个 S95glovedaq.sh 点火器。
只保留其中一条, 并在本文件记录, 避免重蹈"两套并行"的覆辙。
