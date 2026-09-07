# 采集数据格式说明(给上位机/后处理)

一次采集落在 SD 卡 `/mnt/sd/daq/` 下,按**段**分目录;每段最长 10 分钟(自动切段),
长按/Ctrl-C 也会收口一段。**段只是文件容器,跨段联结全靠 `cycle`(全局周期号,由 XVS 60Hz 驱动、
STM32 计数、永不清零)。**

## 目录命名:`<虚拟时间>_seg_<段号>`

板子没有 RTC,日历时间不可信,所以用**虚拟时间**做前缀(2026-09-07 定):

- 卡上 `daq/.vtime` 记一个 `YYYYMMDDHHMM` 和写它时的 `boot_id`;
- **每次真正上电** → 在文件时间上 **+10 小时**(文件为空/新卡 → 以当前系统时间做种);
- 同一次开机内(长按/Ctrl-C 引起的程序重启)前缀不变,**段号接着递增**;
- 若卡上已有同名前缀(换过卡/重刷过),自动再 +10 小时直到不撞 → 永远不会覆盖/丢段。

**按目录名排序 = 采集先后顺序**。前缀只是计数器,不代表真实日期。

```
202604131938_seg_001/   本次开机第 1 段
  cam0.h265             左相机 H.265 裸码流(Annex-B, 每帧顺序追加)   ffplay -f hevc cam0.h265
  cam1.h265             右相机 H.265 裸码流
  pairs.csv             相机索引: 一行 = 一对已配好的左右帧 + 它属于哪个 cycle + 在码流里的位置
  glove.bin             手套原始数据: 每帧 2690 字节顺序追加(SPI 协议 v2 布局, 大端)
  glove.csv             手套索引: 一行 = 一帧 → cycle / PA1 时间戳 / 在 glove.bin 的偏移
  ext_joints.csv        外接转接板 21 路 ADC(仅 -U 启用时有内容)
  glove_decoded.csv     (由 daq_check.py --export 生成)解码后的关节/IMU/四元数, Excel 直接开
202604131938_seg_002/   同一次开机第 2 段(10 分钟自动切段, 或长按后再开采)
202604140538_seg_001/   下一次上电(+10h)的第 1 段
.vtime                  虚拟时间状态文件(勿删; 删了会以当前时间重新做种并自动避开已有目录)
```

## pairs.csv —— 相机帧 → cycle

| 列 | 含义 |
|---|---|
| `pair_seq` | 配对序号(从相机启动起计) |
| `seq0` `seq1` | 左/右相机各自的帧序号 |
| `pts0` `pts1` | 左/右帧的 PTS(µs, CLOCK_MONOTONIC) |
| `dpts_us` | pts1−pts0。硬同步下恒定 −10~0 µs(实测 −6 µs σ 0.6 µs)——**这个数飘了 = 相机失锁** |
| `cycle` | **这对帧属于第几拍**。带 `-` 前缀 = 归属不可靠:开头 1.5 s 标定期,或附近没有手套锚点(手套流已停/RV 漏拍) |
| `residual_us` | 对齐残差(帧时刻与本拍锚点的偏差,已扣固定 offset)。正常 σ≈170 µs;`|residual|>8333` 不会出现(超过就会被标 `-`) |
| `off0` `len0` `off1` `len1` | 该帧在 cam0/cam1.h265 里的**字节偏移与长度** → 可以精确切出"第 N 拍的画面" |

## glove.csv —— 手套帧 → cycle

| 列 | 含义 |
|---|---|
| `cycle` | 本帧周期号(来自帧内 CYCLE 字段) |
| `edge_ns` | 触发本次读取的 **PA1 上升沿内核时间戳**(ns),与相机 PTS 同一时钟——对齐的锚点 |
| `off` | 本帧在 glove.bin 的字节偏移(= 序号 × 2690) |

## glove.bin —— 每帧 2690 字节(字段全大端)

| 字节偏移 | 字段 | 说明 |
|---|---|---|
| 0 | MAGIC u16 | 恒 `0x6A01` |
| 2 | LENGTH u16 | 恒 `0x053F` |
| 4 | CYCLE u32 | 周期号 |
| 8 | HEARTBEAT u32 | 位图,**bit=1 表示异常/缺席**:bit0~19 关节 1~20,bit20~24 触觉 1~5,bit25 IMU,bit26 磁力计,bit27 PD |
| 12 | JOINT[20] u32 | 每路 4 字节:`0xFFFFFFFF`=缺失;否则角度 = `raw & 0x7FFFFF` |
| 92 | TAC_STAT[5] u16 | 5 块触觉阵列状态 |
| 102 | TACTILE[5][16][16] u16 | 5 块 16×16 触觉阵列,共 2560 字节 |
| 2662 | IMU_RAW[6] i16 | ax ay az gx gy gz(原始计数;az≈−4096 = 1g 朝下) |
| 2674 | MAG_RAW[3] i16 | mx my mz |
| 2680 | QUAT[4] i16 | 四元数 w x y z,**Q14**(÷16384 得小数),模长必≈1.000 |
| 2688 | CRC u16 | CRC-16/ARC(poly 0x8005 反射 0xA001, init 0)覆盖 0~2687 字节 |

## 怎么把相机和手套对上

```
pairs.csv 里 cycle = N 的那一行  ←→  glove.csv 里 cycle = N 的那一行
  用 off0/len0 从 cam0.h265 切出画面     用 off 从 glove.bin 读 2690 字节解码
两只手套之间: cycle 相等 = 同一时刻(XVS 同源)
```
每一拍**至多**一对相机帧、一帧手套数据。相机比手套早启动几秒是正常的(前面那些对帧 cycle 带 `-`)。

## 验收:`tools/daq_check.py`

```bash
python daq_check.py D:\glove_data\seg_001_154            # 体检报告
python daq_check.py D:\glove_data\seg_001_154 --export   # 另存 glove_decoded.csv 画图
python daq_check.py D:\glove_data\seg_001_154 --frame 3000   # 看第 3000 帧解码值
python daq_check.py D:\glove_data                        # 整个目录逐段体检
```
它复算每帧 CRC、检查 cycle 连续性/丢拍、PA1 间隔、四元数模长、码流偏移自洽、双摄 dpts、
对齐残差、pairs↔glove 联结覆盖率、外接 ADC 有效率——任何一项不对都会用 ✗/! 标出来。
