# STM32G474CBT6 烧录流程（PB8/PB9 复用 I2C1 专用）

> 背景：主控板 I2C1（温度传感器）占用 PB8(SCL)/PB9(SDA)。**G4 系列 PB8 兼任 BOOT0**，
> 而 I2C 必须上拉（2.2k~4.7k 到 3.3V）→ 复位瞬间 PB8 恒为高电平 → 若引脚采样开着，
> 芯片会进 ROM bootloader 而不是跑主程序。
> 解法：把 option bytes 写成 **nSWBOOT0=0、nBOOT0=1**（不采样引脚、强制从主 flash 启动）。
> 本文档规定"写 OB"如何纳入烧录流程。

## 原理三层（为什么空片不会被坑，中间态才会）

1. **SWD 烧录与 BOOT0 无关**：ST-Link 走调试口 + connect under reset，任何启动状态都能接管；
2. **空片保护**：G4 BootROM 有 "flash empty check"——主 flash 第一个字全 FF 时无视 BOOT0
   直接进 bootloader，出厂空片永远可烧录；
3. **危险的中间态**：flash 已有程序、但 OB 还没写 → empty check 失效 + 引脚采样开着 +
   PB8 被 I2C 上拉顶高 → 复位直奔 bootloader，现象是"烧完不跑"，极具迷惑性。
   **所以规则只有一条：首次烧录必须同时写 OB。**

## 首次烧录标准流程（每块新板执行一次）

方式 A：STM32CubeProgrammer 图形界面
1. ST-Link 连接（Mode: Under reset）；
2. 烧录固件 hex/bin；
3. 切到 OB (Option bytes) 页 → User Configuration：
   - `nSWBOOT0` 取消勾选（=0，不采样 PB8）
   - `nBOOT0` 勾选（=1，从主 flash 启动）
   - `DBANK` 取消勾选（=0，单 bank，**必须在烧固件之前改**，见下方说明）
   → Apply（芯片会自动 OBL 重载并复位）；
4. 断电重上电，确认程序运行（LED 有显示即为成功）。

方式 B：命令行（可写进产测脚本）
```
STM32_Programmer_CLI -c port=SWD mode=UR -ob nSWBOOT0=0 nBOOT0=1 DBANK=0 -d firmware.hex
```

### ⚠️ DBANK 陷阱（2026-09-02 实战踩坑）

G474CB（128KB，Cat.3）出厂 `DBANK=1`（双 bank）：128KB 拆成两个 64KB bank，
且**地址不连续**——Bank1 = 0x08000000~0x0800FFFF，Bank2 = 0x08040000 起，
中间是空洞。而 CubeIDE 链接脚本按 128KB 连续排布 → **镜像一旦超过 64KB**，
尾部落进空洞，烧录报"操作超出存储限制 / Error finishing flash operation"。
本项目固件已超 64KB，所以 `DBANK=0` 是必写项，顺序在烧固件之前
（否则烧录本身就失败）。改完 DBANK 建议跟一次全片擦除再烧。
副作用备忘：单 bank 模式下 flash 页大小从 2KB 变 **4KB**——将来做
CAN bootloader 等任何按页擦写的功能，页参数按 4KB 算。
固件自愈段（main.c）也已包含 DBANK=0（该段代码位于 flash 前 64KB，
两种模式映射相同，自愈安全）。

## 双保险：固件自愈（建议写进固件初始化）

固件启动早期检查 FLASH->OPTR，发现 nSWBOOT0/nBOOT0 不符就自写 OB 并触发重载。
效果：任何板子只要成功烧过一次固件（哪怕操作员忘了写 OB 而恰好复位时 PB8 为低侥幸跑起来），
即自动免疫。伪代码：

```c
/* 在 main() 早期调用一次 */
if ((FLASH->OPTR & FLASH_OPTR_nSWBOOT0) || !(FLASH->OPTR & FLASH_OPTR_nBOOT0)) {
    HAL_FLASH_Unlock(); HAL_FLASH_OB_Unlock();
    FLASH_OBProgramInitTypeDef ob = {0};
    ob.OptionType = OPTIONBYTE_USER;
    ob.USERType   = OB_USER_nSWBOOT0 | OB_USER_nBOOT0;
    ob.USERConfig = OB_BOOT0_FROM_OB | OB_nBOOT0_SET;   /* nSWBOOT0=0, nBOOT0=1 */
    HAL_FLASHEx_OBProgram(&ob);
    HAL_FLASH_OB_Launch();   /* 触发重载，芯片复位 */
}
```

## 硬件配套修改（原理图注意）

- PB8 原设计的 **10k 下拉电阻删除**：它会和 I2C 上拉分压（4.7k 上拉 + 10k 下拉 ≈ 2.2V），
  既破坏 I2C 高电平裕量，也让 BOOT 电平不清不楚。写 OB 之后 PB8 就是普通 GPIO，不需要下拉；
- I2C1 上拉：PB8/PB9 各 4.7k 到 3.3V（100kHz）；若跑 400kHz 用 2.2k；
- 救砖通道不受影响：整片擦除后 flash 变空 → empty check 生效 → 自动可进 bootloader；
  SWD 更是永远可用。

## 产线检查清单

- [ ] 烧固件与写 OB 一条命令完成（方式 B）
- [ ] 复位后 LED 正常 → 出厂
- [ ] 若"烧完不跑"：第一怀疑 OB 没写上，读 OPTR 确认

---

## 附：量产日志通道备案（未定案，2026-07-14 记录）

### 方案甲：PA14(SWCLK) 复用为 USART2_TX 单向日志
- PA14 有 USART2_TX (AF7)，已在 CubeMX 数据库核实；PA13 无 UART 数据信号，故只能 TX-only。
- **红线：烧录焊盘必须含 NRST**（5 点：SWDIO / SWCLK兼UART_TX / NRST / GND / 3V3）。
  PA14 切走后 SWD 死，唯一救援 = connect under reset（复位态引脚回 SWD 默认功能）。
- 纪律：① 调试版固件不切换（保住在线调试）；量产版上电延迟 1~2s 再切；
  ② PA14 到焊盘串 220Ω（防 ST-Link 与 UART 输出对顶）；
  ③ 开机会有一段 break/乱码（切换前 PA14=SWCLK 下拉低电平），正常现象。

### 方案乙（优先考虑）：日志走现有 SPI 链路
- STM32→RV1126B 帧内加"诊断消息类型"或帧尾诊断字段，日志经 RV 转发上位机/PC。
- 现场不拆手套即可看日志；UART 焊盘退化为"SPI 链路本身故障"时的兜底。
- 两方案不互斥，建议都留：乙做主通道，甲做最后手段。
