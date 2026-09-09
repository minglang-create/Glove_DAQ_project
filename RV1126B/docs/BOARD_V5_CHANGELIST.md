# 下一版板(V5)改板清单

> 汇总 V4 板调试中实测确认的硬件问题与改进项,按优先级排序。
> 每条都注明**为什么**(实测证据)和**怎么改**,避免下版重蹈。

---

## P0 —— 必改(功能性缺陷)

### 1. RJ45 磁座四对镜像反接 → 必须用自制换序网线(维持 P0)

**症状**:接标准网线时 PHY(RTL8211F)驱动正常挂载、MDIO 可读写(ID 0x001CC916),
`ANLPAR=0` 双向失聪、灯不亮、强制 MDI/MDIX 均无链路。**用户自制换序网线后可正常协商
1Gbps/Full 并跑通(实测 scp 拉 40MB)**,说明就是板端线序问题,不是 PHY/磁变坏。

> ★2026-09-09 重要澄清★ 期间出现的"板→PC 每帧都是 CRC 错帧、PC→板 0 错"**不是** RJ45
> 线序造成的(1000BASE-T 四对双向同时工作,线序错会两个方向一起坏),而是 **dts 里音频
> `pa-ctl-gpios` 抢了 RGMII TXD1** —— 详见本清单末尾"已修软件坑"一节。两件事互相独立。

**根因**:V4 把 TRX0..3 按"组内可互换"的传言接成了**四对镜像**
(TRX0→磁座 9/8=D 对、TRX1→6/7=C 对、TRX2→4/5=B 对、TRX3→3/2=A 对)。
MDI 规则:对内 P/N 可换(极性自纠)、A↔B + C↔D 交叉可自纠(MDIX),
但**任意对间错位(镜像 3210)无解** —— 自协商只在 A(1-2)/B(3-6) 两对上进行。
软件救援四层穷尽后否证(RTL8224 的 MDI 重映射寄存器在 8211F 不存在,
用 MMD7 EEE 做阳性对照排除了访问方法问题)。

**改法**(对照 T11106ML6GS 内部原理图):

| PHY 侧 | 接到磁座引脚 |
|---|---|
| TRX0± | RJ1 的 **1/2**(A 对) |
| TRX1± | RJ1 的 **4/5**(B 对) |
| TRX2± | RJ1 的 **6/7**(C 对) |
| TRX3± | RJ1 的 **8/9**(D 对) |

P/N 顺手接即可(极性自纠)。**RJ1.1 保持 100nF×3 到 GND,勿接 VCC。**
改好后即可用普通网线,不再依赖自制线。

### 2. XHS 未引到 STM32 → 从机手套的相机带不起来

**症状**:STM32 只供 XVS(7.25µs 精确脉冲、电平实测正确)时,从机相机
`frame amount:-1` 永不出帧;i2c 把 cam0 切 master 同时输出 XVS+XHS 后,
同网的 cam1(仍 slave)**立即出图且两颗帧数完全相同**(179=179)。

**根因**:IMX415 从机模式**启动**时必须同时收到 XVS 和 XHS 两个信号
(手册 slave 模式明文:"input XVS **and** XHS";XHS = 1H 周期行同步 ≈138kHz)。
V4 网表 `CSI0_XHS = P1.5 + P2.5`,只有两颗相机互连、**未接 STM32**,
也未过 Type-C SBU 到对侧手套。

> 注:早前"XHS 不需要"的结论是误读 —— 那个实验(主机 XHS 切 Hi-Z 后从机
> 仍出流)证明的只是"**已锁定后维持**不需要 XHS"。

**改法(二选一,推荐 A)**:

- **A. XHS 走 Type-C SBU2**(强烈推荐):USB-C 有 SBU1/SBU2 两根,V4 只用了
  SBU1 传 XVS,**SBU2 完全空着**。把主机手套的 XHS 也经电平转换送到 SBU2、
  从机手套接回自己的 XHS 网 → **四颗相机全部硬锁主机 cam0**,
  彻底消除从机手套的晶振漂移(见下方"方案对比")。代价:两颗电平转换芯片。
- **B. XHS 引到 STM32 一个定时器脚**:STM32 同时产生 XVS(60Hz)+ XHS(138kHz)
  两路严格同步信号。固件复杂度高,且 138kHz 抖动要求严苛,不推荐。

### 3. TXS0101(U18)的 OE 焊死在 VCC_1V8 → 主从角色无法软件切换

**症状/影响**:V4 当前的双手方案需要"从机手套物理隔断相机与 STM32 之间的
XVS 通路",而 OE(U18.5)与 VCC(U18.1)同接 `VCC_1V8` 网,只能靠**拆芯片或
抬起 OE 脚接地**实现,不可逆、且两只手套的板子从此不同规格。

**改法**:`U18.5(OE)` 改接 **STM32 一个 GPIO**(带下拉),
→ 主/从角色变成纯软件可选,两只手套板子完全同规格、同固件。

---

## P1 —— 建议改(显著简化软件/提升精度)

### 4. XVS 加一路到 RV1126B 的 GPIO

**为什么**:目前 RV 无法直接观测 XVS(XVS 网只接两颗相机 + TXS),
相机帧↔CYCLE 的对齐只能借道 **PA1** 作中介,而 PA1 相对 XVS 有
**3~8ms 的可变延迟**(STM32 要等 CAN 回帧 + IMU 采集)。实测 σ=173µs、
判决裕量 48σ,够用但精度受限于这个中介。

**改法**:XVS 网分一路(经电平转换)到 RV 一个可做边沿捕获的 GPIO
→ RV 直接给 XVS 边沿打内核时间戳,**对齐精度提升到 µs 级**,
且 `align.c` 可去掉 PA1 中介与滑窗跟踪逻辑(大幅简化)。

### 5. 引出一路独立 UART 座(UART2 = GPIO3_B0/B1 已在 dts 启用但未引出)

**为什么**:V4 唯一的串口座 RVDB(球 A3/A4)= UART0 = 内核控制台。外接转接板的 21 路关节
ADC 只能占用它 → **串口控制台被迫放弃**(见 EXT_UART.md)。
**改法**:把 UART2(GPIO3_B0 TX / GPIO3_B1 RX)引到一个独立 3 针座;若对端仍只能 TX,
也请把我们的 TX 接到对端一个 **EXTI 可用的 GPIO**(触发线)。这样控制台失而复得。

### 6. 相机 XHS 引脚加上拉定电平

**为什么**:从机模式下 XHS 若真悬空易受噪声干扰(V2 实验记录)。
**改法**:XHS 网加弱上拉到 1.8V(10kΩ)。

---

## 方案对比:V4 现状 vs V5(P0-2 采纳方案 A)后

| | V4(TXS 隔断方案) | V5(XHS 走 SBU2) |
|---|---|---|
| 主机手套相机 | cam0 主 + cam1 从,硬锁 dpts −6µs ✓ | 同 ✓ |
| 从机手套相机 | 自己 cam0 当局部主(**独立晶振**) | **真从机,硬锁主机 cam0** ✓ |
| 从机 CYCLE 与相机 | 两晶振,漂移几 µs/s,累积不封顶 | **同源,零漂移** |
| 软件 | 需滑窗跟踪 offset(已实现) | 固定 offset 即可 |
| 残留缺陷 | 从机每几十分钟一次帧/周期滑移(residual 列可识别) | **无** |
| 板子规格 | 主/从板不同(从机抬 OE) | **完全同规格同固件** |

---

## 附:V4 已验证正常、勿动的部分

- **双摄 MIPI**:D 档 891Mbps/27MHz,binning 1944×1097@60fps,SoT/CRC 全零;
- **相机主从硬同步**:cam0 主 / cam1 从,dpts 恒 −5~−6µs(V4 板实测通过);
- **SD3.0**:vccio_sd 用 regulator-gpio(GPIO0_A6,低=1.8V/高=3.3V)+ SDR104,
  实测 1.8V + SDR104 + 200MHz 时钟达标;
- **SPI0_M2**(J16/P5/T3/P6)+ **PA1**(球 K13=GPIO0_A4)+ **按键**(球 A2=GPIO0_A0):
  全部实测正常;
- **USB peripheral**:自研板无 HUSB311,dts 写死 `dr_mode="peripheral"` 后 adb/图传正常;
- **IMU FSYNC 挂在 XVS(3.3V)网上**:两只手套的 IMU 帧同步输入都能收到主机 XVS,
  将来 IMU 采样可硬件对齐到 XVS —— 这是 V4 的意外收获,保留。

---

## 附:已修的软件坑(不需要改板,但下版布线要避开)

### 音频 `pa-ctl-gpios` 抢了 RGMII TXD1(2026-09-09 修复)

基座 `rv1126b-luckfox-aura.dtsi` 里 `&acdcdig_dsm` 的
`pa-ctl-gpios = <&gpio5 RK_PC0 GPIO_ACTIVE_HIGH>`(音频功放使能)与 RGMII 直接冲突:

```
pin 176 = GPIO5_C0 = eth_txd1_m1   ← RGMII 的 TXD1 数据线
```

开机顺序是"以太网先 probe 套好 pinctrl → 音频后 probe",而 Rockchip pinctrl 在
`gpiod_direction_output()` 时**会把引脚 IOMUX 改回 GPIO 功能并驱动低** → 每个数据
半字节的 bit1 恒为 0 → 板子发出的每一帧到对端都是 CRC 错帧;RX 引脚没被抢,收方向完好。

**症状指纹**(以后遇到类似问题可对照):能协商 1Gbps/Full、PC→板 0 错误、板→PC 好帧 0
错帧一堆(且错帧数 > 发出帧数,一帧被拆成多个)、ping 零星或全丢、**四种 `rgmii-id/
txid/rxid/rgmii` 延迟模式全都救不回来**(因为根本不是时序问题)。

**定位方法**:`/sys/kernel/debug/pinctrl/*/pinmux-pins` 里找 RGMII 引脚,正常应是
`(GPIO UNCLAIMED)`,被抢的那根会显示 `gpio5:176`;再 `grep gpio-176 /sys/kernel/debug/gpio`
看标签(这里是 `pa-ctl`)。

**修法**:`rv1126b-luckfox-aura-v4.dtsi` 里 `&acdcdig_dsm { /delete-property/ pa-ctl-gpios; };`
(采集板不接功放)。**注意:运行时只解绑音频驱动不够** —— `ip link down/up` 不会重新套
pinctrl,必须重跑 probe(`unbind`+`bind` 以太网驱动)或改 dts 后重启。

**下版布线**:若仍要保留音频功放使能,务必换一根不与 `ethm1_*` 组冲突的 GPIO。

---

## 附:RTL8211F 的 strap(CONFIG)脚 —— 哪些必须外部电阻,哪些靠寄存器就行

**2026-09-09 对着 RTL8211F 数据手册 + 板上实测厘清。** 这些 CONFIG 脚**复用在 RX 组上**,
PHY 在自己 POR 那一刻采样它们;此时内核还没跑,**设备树/SoC 侧的内部拉都影响不到**。
不加电阻时的默认值来自 **PHY 自己在 POR 期间内建的上拉/下拉**(手册引脚表的 `PU`/`PD` 标记):

| CONFIG 脚 | 功能 | PHY 内部默认 | 寄存器能否覆盖 | 要不要外部电阻 |
|---|---|---|---|---|
| RXD3 | PHYAD[0] | **PU → 1** | ❌ 鸡生蛋(需先知地址才能通信) | 单 PHY 不用 |
| RXC | PHYAD[1] | PD → 0 | ❌ | 单 PHY 不用 |
| RXCTL | PHYAD[2] | PD → 0 | ❌ | 单 PHY 不用 |
| RXD2 | PLLOFF | PD → 0 | ✅ ALDPS 寄存器 | 不用 |
| RXD1 | **TXDLY** | PD → 0 | ✅ **实测被驱动覆盖成 1** | 不用 |
| RXD0 | RXDLY | PU → 1 | ✅ | 不用 |
| LED0 | CFG_EXT | 取决于 LED 电路 | ❌ POR 时定死 | **必须核对** |
| LED1/LED2 | CFG_LDO[1:0] | 取决于 LED 电路 | ❌ | **必须核对** |

**证物**:strap 默认给 TXDLY=0,但读 PHY 页 0xd08 寄存器 0x11 = `0x0109`(bit8=1)——
Linux realtek 驱动按 `phy-mode="rgmii-id"` 覆盖了 strap。**故 TXDLY/RXDLY/PLLOFF 三个
strap 电阻可以全省掉**,只要 dts 里 phy-mode 选对。

**PHY 地址**:默认 strap 算出 PHYAD=001=**1**;实测 `mdioscan` 显示地址 1 有 PHY
(ID 0x001CC916),地址 0 也应答——因为手册 Note 1 规定 **PHYAD=0 是 MDIO 广播,
每个 PHY 都应答**。所以早前 dts 写 `reg=<0>` 也能通,但那是广播(广播**写**会打到总线上
所有器件)。**已改为 `phy@1 / reg=<0x1>`**,冷启动实测 `PHY [stmmac-0:01]`、千兆链路、
ping 6/6 通、SSH 可连、TXDLY 仍为 0x0109。

**★V5 最要当心的一条:LED 脚就是 CFG_EXT/CFG_LDO strap,决定 RGMII I/O 的供电方式与电压★**
(内部 LDO 还是外部供电;3.3/2.5/1.8/1.5V)。手册明说 LED 输出与 CFG 脚复用、
"external combinations required for strapping and LED usage must be considered to avoid contention"。
**LED 限流电阻接 3.3V 还是接 GND,直接决定 RGMII I/O 电压档位,而这个寄存器改不了。**
V5 若改动 LED 电路,务必对着 SoC GPIO5 那组的 IO 域电压重新核算。

**手册明确要求的外部电阻**:MDIO 需 **1.5kΩ 上拉**("The MDIO pin needs a 1.5k Ohm
pull-up resistor")。现板能读写 PHY 说明勉强够用,V5 建议按手册补上。

### V5 以太网电阻清单

```
不加: RGMII 12 根数据/时钟/控制线的上下拉(全程被驱动, 永不悬空)
不加: RXD0/RXD1/RXD2 的 strap 电阻(TXDLY/RXDLY/PLLOFF 靠寄存器覆盖)
不加: RXD3/RXC/RXCTL 的 PHYAD strap(单 PHY, 默认地址 1 即可; 多 PHY 才需要)
  加: MDIO 上拉 1.5k(手册要求)
  加: PHY nRST 上拉 10k(软件跑起来之前要有确定电平)
核对: LED0/1/2 电阻接法 = CFG_EXT/CFG_LDO strap → 决定 RGMII I/O 电压, 软件救不了
可选: RGMII 串联 22~33Ω(先用 dts 驱动强度 drv_level 调, 不行再加)
必改: RJ45 磁座线序(见 P0-1)
标注: ethm1_* 那 14 根脚"以太网专用, 勿被其他外设复用"(这次 TXD1 被音频抢的教训)
```
