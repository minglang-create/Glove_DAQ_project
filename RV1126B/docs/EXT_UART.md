# 外接转接板 21 路关节 ADC —— 串口链路(2026-09-07)

## 物理链路

| 项 | 值 |
|---|---|
| RV 侧口 | **UART0 复用 m2** = SoC **GPIO0_B3(TX)/GPIO0_B4(RX)** = SoM 球 **A3/A4** = 立贴座 RVDB(1=TX 3=RX 2/4/5=GND),SoC IO 电平直出,无电平转换。依据 MYZR 核心板原理图 `MYZR-RV1126B-LB221-REVA-OPEN.pdf` |
| 对端 | 转接板 STM32:**只能 TX 给我们**;我们的 TX 接到它一个普通 GPIO(不是 RX)→ 它收不了字节,但能看到电平沿 |
| 参数 | 460800 8N1 |
| 代价 | **UART0 原是内核控制台(fiq_debugger)**。让给数据后串口控制台消失,调试只剩 adb |

## 同步方案(方案一 trig 为主,方案二 free 备用)

**trig(一问一答,确定性)**:每拍 PA1 沿一到、**SPI 事务之前**,RV 发一个 `0x00`。8N1 的 0x00
= 起始位 + 8 个 0 = 连续 **9 个位时间的低电平**(460800 下 ≈19.5µs)。对端 EXTI 下降沿 → 采 21 路
(<1ms)→ 立刻回 46B 帧(1.0ms)。回来的帧就是本拍的,归属由构造保证;采样点 = PA1 + 几十 µs,
抖动 ≈ PA1 的 σ(实测 173µs)。若在 SPI 之后才发,会白白晚 4.3ms 且叠加事务抖动——所以钩子
打在 `glove_link` 的 `wait_ready()` 返回处(`glove_set_edge_cb`)。

RV 侧规则:回帧必须在触发后 **10ms** 内到,否则本拍记缺失(`valid=0`)、迟到帧**丢弃**,绝不
顶到下一拍。

**free(备用)**:对端 300Hz 自由发,RV 只留最新,每拍取最新并记帧龄。零硬件差别,
触发链不稳时 `-U ...:free` 一键切换。

## 帧格式(对端定义,46 字节,小端)

```
[0]=0xA5 | [1..2]=seq(u16,每帧+1,65535 回 0) | [3..44]=ch[0..20](u16×21) | [45]=XOR(字节 1..44)
ch: 12bit ADC 0~4095;0xFFFF = 该 ADC 本轮没采到(DMA 超时),通常整组同时出现
```

## 对端固件必须配合的三条

1. **触发门限**:线路**安静 ≥10ms 之后的单个下降沿**才算触发。u-boot 开机仍会在 TX 上喷一小段日志
   (连续比特流,沿间隔几 µs),这条规则天然滤掉它;RV 接管后 TX 空闲恒高,只有 0x00 会过。
2. **忙则跳过**:上一帧还没发完又来触发 → 跳过本拍,别排队(排队会让帧和拍错位)。
3. **seq 每发一帧 +1**(不是每触发 +1),RV 用它的连续性统计 `seq_gap`。

## 启用 / 恢复控制台

内核侧用独立 dtsi + 主 dts 末尾一行 include 切换(**必须在末尾**:它要覆盖 `chosen` 里的 bootargs):

```
sysdrv/source/kernel/arch/arm64/boot/dts/rockchip/
  rv1126b-luckfox-aura-uart0-data.dtsi   关 fiq_debugger / 开 &uart0(★m2=GPIO0_B3/B4★) / bootargs 去 console+earlycon
  rv1126b-luckfox-aura.dts 末尾:  #include "...-uart0-data.dtsi"   ← 注释掉 = 恢复串口控制台
```
改完 `sudo ./build.sh kernel && sudo ./build.sh firmware`,烧 boot.img。

应用侧:`glove_daq_rv -U /dev/ttyS0[:460800][:trig|free]`,默认不启用,零副作用。
建议写进 `RkLunch-GLOVEDAQ.sh` 的启动参数。

## 落盘与诊断

每段多一个 `ext_joints.csv`:`cycle,seq,valid,latency_us,ch0..ch20`。`valid=0` 的拍通道列留空。
按 `cycle` 与 `glove.csv`/`pairs.csv` 联结。

状态行 `[外接] 触发N 收N 缺N 迟弃N xor错N 重同步N 序号跳N 延时avg/max`:
- **延时 avg** 正常应 ~1.5~2.5ms(对端采样 + 1.0ms 传输);**max** 若逼近 10ms 说明对端偶发卡顿;
- **缺** 持续增长 = 对端没收到触发(电平/门限)或回得太慢;**迟弃** = 回帧超 10ms;
- **xor错 / 重同步** 增长 = 线路噪声或波特率不匹配;**序号跳** = 对端跳拍(忙则跳过)。

## ★踩坑记录:UART0 的 pinctrl 绝不能抄 SoC dtsi 的默认值 m0★

SoC dtsi 里 `uart0` 节点默认 `pinctrl-0 = <&uart0m0_xfer_pins>`(GPIO2_A0/A1)。这两根在核心板上
**是 SDMMC0_D0/D1**。2026-09-07 首版 dtsi 沿用了 m0,后果:内核启动时 `rockchip-pinctrl: could not
request pin 64 (gpio2-0) from group sdmmc0-bus4-pins` → **SD 卡控制器整个消失**(`mmc1` 不存在、
`/dev/mmcblk1*` 不出现),存储自检永远失败。fiq_debugger 时代没暴露是因为它不经 pinctrl 申请引脚
(沿用 u-boot 设好的 m2)。教训:**引脚复用以核心板原理图为准,不信 SoC 默认值**;改 dts 后
先 `dmesg | grep "could not request pin"` 再谈别的。

## 验证记录

- 2026-09-07 PC 端 pty 仿真 9/9 通过:一问一答配对、坏 XOR 拒收、垃圾+假头重同步、超时判缺、
  迟到丢弃且不顶下一拍、free 取最新、统计口径。
- 板上联调:待对端固件就绪(见上"三条")。
