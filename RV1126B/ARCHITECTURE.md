# Glove_DAQ_RV1126B_SDK —— RV 侧整机框架(2026-09-02, 按《协议契约_V1 第四次修订》)

## 一句话
每只手套 = RV1126B(双 IMX415 从机 + SPI 主机) + STM32G474(传感器主控 + SPI 从机 + XVS 时基)。
本程序 = RV 侧全部: 生命周期状态机 + 双摄 H.265 管线 + 手套链路 + 帧↔CYCLE 自动对齐 + 分段落盘。

## 模块地图(与 STM32 侧模块一一呼应)
```
main.c        总装(薄): 参数/信号/exec重启, 恒定几十行
daq_fsm       生命周期: SELFCHECK→WAIT_READY→CAM_START→WAIT_FRAME→RUNNING⇄PAUSED→SHUTDOWN
glove_link    ≙ STM32 的 glove_protocol.h+rv_link: 恒定2690B全双工事务/小包/数据帧/CRC/PA1
cam_pipeline  双摄管线(dual_cam 血统逐行同源): ISP→VI→VENC(H.265)→采集线程→配对线程→回调
align         帧↔CYCLE 自动标定: PA1沿时间戳锚定 + 中位数offset + 最近邻整数指派
recorder      分段落盘: seg_<n>_<boottime>/{cam0.h265,cam1.h265,pairs.csv,glove.bin,glove.csv}
button        按键(去抖30ms/长按2s); 无按键脚时 SIGUSR2 模拟
glove_view    终端视图(-w imu/joint/tactile/all), 独占屏幕渲染
```

## 线程模型
```
FSM 线程(main)      独占 SPI 链路(收发全在此) + 按状态选节奏:
                    非运行=100ms轮询(glove_txn_poll), 运行=PA1沿驱动(glove_read_frame)
cap0/cap1 线程      VENC取流 → 帧队列(cam_pipeline 内部)
sync 线程           最近邻配对 → pair_cb 回调(对齐查表+落盘, 文件与手套侧分工, 无锁)
button 线程         沿+电平去抖 → 置事件旗(g_loop_break 可打断 read_frame 阻塞)
```

## 生命周期(契约 §7, RV 视角)
```
上电 → 自检: 相机在位(sysfs 驱动绑定证据, 不启动/不 i2c 裸读) + 落盘目录
     → 发 0x5501(STM32 未应前每秒重发, 幂等) → 读 0xAA01/0x5401(位图展示)
按键(或 USR2) → 发 0x5101 → STM32 走 XVS 仲裁(300ms宣告/500ms窗)
收 0xC301(角色P0/单双手P1) → cam_start(≈1.5~2s, 从机模式, 无XVS先自由跑)
     → 回 0xC101 → 主机 STM32 凑齐后放号
MISO 出现数据帧(最终确认) → 段号++ → align_reset → 落盘开段 → RUNNING
RUNNING: 沿驱动读帧 → align锚点 + glove.bin + 视图 + (-V 假数据验收)
按键短按 → 0xC201 → PAUSED(XVS不停/相机锁相不丢/对齐不重标) → 短按 0xC101 恢复(新段)
长按/INT → 收尾。 USR1 → 发 0x5F01 → exec 重启自身(全新自检, 0x5501 重发)
```

## 对齐引擎为什么长这样(契约"RV 自动标定偏移"的落地)
- IMX415 从机**无 XVS 自由跑**(V2 实测): 0xC301 启动到放号之间有数量不定的自由跑帧
  → "帧数天然一致"不成立 → 偏移必须每会话实测。
- 锚 = 手套帧自带 CYCLE + 其 PA1 沿的内核时间戳(t_edge_ns); 相机 PTS 同源单调钟。
- PA1 相对 XVS 抖 3~8ms, 但 90 样本中位数后的 offset 误差 ≪ 半周期 8.3ms
  → 最近邻整数指派唯一; 之后每帧输出 residual, σ<1ms = 指派稳定的实证。
- 暂停恢复不重标(XVS 未停网格未变); 0x5F01 重启才重标。
- 标定完成前的帧对也落盘, pairs.csv 的 cycle 带 "-" 前缀标记未标定。

## 数据落盘布局
```
/userdata/daq/seg_001_<boottime>/
  cam0.h265 cam1.h265    裸码流(IPPP), pairs.csv 的 off/len 可切出任意帧
  pairs.csv              pair_seq,seq0,seq1,pts0,pts1,dpts_us,cycle,residual_us,off0,len0,off1,len1
  glove.bin              原始 2690B 数据帧 × N
  glove.csv              cycle,edge_ns,off
上位机按 cycle 联结 pairs.csv × glove.csv = 视频帧↔传感器帧对齐完成;
跨手套按 cycle 相等对齐(SEG 只在本手套内有意义)。
```

## 已知边界/待办
- 按键 GPIO 默认未配(-K chip:line; SW3=SoM球A2 的 GPIO 映射待实测) → 先用 USR2。
- 0x5501 无回执 → 重发直到见 0x5401(+3次), 依赖包幂等。
- 重新自检 = exec 进程重启(rkaiq 同进程二次初始化不稳, 用整洁重启换稳定)。
- LED 标定 0xE101 暂缓(契约 §9); CAN 发送 0xFD01 未实现。
- 台架: STM32 GLOVE_FSM_AUTOSTART=1 时本程序加 -A。


## 落盘(SD 卡, 2026-09-05)

**为什么是 SD**:`/userdata` 分区只有 975MB,按 2×10Mbps 视频 + 161KB/s 手套 ≈ 2.7MB/s
只够 6 分钟;SD 卡 SDR104(实测 198MHz)、239GB ≈ 24 小时。

**线程模型**:热路径 `rec_on_pair`(相机 sink 线程)/`rec_on_glove`(FSM 线程)只做
一把无争用互斥锁 + `fwrite` → 数据进内核页缓存(微秒级),从不直接等卡。2GB 内存、
`dirty_ratio=20%` ≈ 400MB 脏页,按 2.7MB/s 能吸收 ~2.5 分钟的卡停顿,远超 SD 卡 GC
停顿(几十~几百 ms)。**慢操作全在 flush 线程**:每 5s `fsync`(fflush 后 `dup` fd,
锁外 fsync,原 fd 被关也不影响)、每 10min 切段(锁外开新文件 → 锁内原子换指针 →
锁外收口旧段)、每 2s `statvfs` 查空间。若 fsync/切段放在热路径,SD 卡一次几百 ms
的停顿就会撞上相机队列 QCAP=8 帧(133ms)的丢帧线 —— 这是设计的硬约束。

**exFAT 两条铁律**:①文件大小只在 fsync/close 时写进目录项 → 不 fsync 就拔卡,
数据块在卡上、PC 却看到 0 字节文件,故周期 fsync 是必须项;②无日志 → 拔卡前先
长按结束。**按时间切段**是损伤隔离:掉电/拔卡只危及当前段尾巴。

**存储自检**(`rec_check_storage`):落盘目录所在文件系统的设备必须是 `/dev/mmcblk1*`
(从 `/proc/mounts` 最长前缀匹配,目录未创建也能判)且剩余 ≥ 阈值;失败 = 自检未完成
→ 不发 0x5501、收到 0xC301 也拒绝;每秒复检,插卡自动恢复。运行中空间低于阈值 →
flush 线程置旗,FSM 走"结束"路径停止采集。`-M 0` 关闭检查(调试落 eMMC 用)。
