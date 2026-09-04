/**
  ******************************************************************************
  * @file    mcp2518.c
  * @brief   MCP2518FD 驱动实现
  *
  *          寄存器地址与位域全部取自 MCP2518FD 手册 DS20006027B，
  *          关键处标了章节/寄存器号，便于复核。
  ******************************************************************************
  */

#include "main.h"
#include "mcp2518.h"
#include "rv_link.h"
#include "glove_frame.h"
#include "frame_sync.h"
#include <string.h>

/* main.c 里由 CubeMX 生成的句柄（只取 Instance，不调 HAL_SPI_* 收发函数） */
extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi3;

/* ==========================================================================
 * MCP2518FD 寄存器（手册 §3，表 3-1 / 3-2）
 * ========================================================================== */

/* SPI 指令（§4，表 4-1）：16 位命令字 = 4 位操作码 + 12 位地址 */
#define MCP_CMD_RESET               0x0u
#define MCP_CMD_READ                0x3u
#define MCP_CMD_WRITE               0x2u

/* MCP2518FD 专有 SFR（0xE00 起） */
#define REG_OSC                     0xE00u
#define REG_IOCON                   0xE04u
#define REG_ECCCON                  0xE0Cu

/* CAN FD 控制器（0x000 起） */
#define REG_CiCON                   0x000u
#define REG_CiNBTCFG                0x004u
#define REG_CiDBTCFG                0x008u
#define REG_CiTDC                   0x00Cu
#define REG_CiINT                   0x01Cu
#define REG_CiTREC                  0x034u

/* FIFO 寄存器组：CiFIFOCONm = 0x50 + 12*m（表 3-2 实证：FIFOCON2 = 0x68） */
#define REG_FIFOCON(m)              (0x050u + 12u * (m))
#define REG_FIFOSTA(m)              (0x054u + 12u * (m))
#define REG_FIFOUA(m)               (0x058u + 12u * (m))

/* 滤波器（表 3-2 实证：FLTOBJ1 = 0x1F8 → 基址 0x1F0，步长 8） */
#define REG_FLTCON0                 0x1D0u
#define REG_FLTOBJ(f)               (0x1F0u + 8u * (f))
#define REG_MASK(f)                 (0x1F4u + 8u * (f))

/* 消息 RAM（§3.1：0x400~0xBFF，2KB） */
#define MCP_RAM_BASE                0x400u
#define MCP_RAM_SIZE                2048u

/* ---- OSC（寄存器 3-1）---- */
#define OSC_PLLEN                   (1uL << 0)
#define OSC_OSCDIS                  (1uL << 2)
#define OSC_SCLKDIV                 (1uL << 4)
#define OSC_OSCRDY                  (1uL << 10)

/* ---- IOCON（寄存器 3-2）。⚠ 勘误：只能整字写，字节写会破坏 LAT ---- */
#define IOCON_TRIS0                 (1uL << 0)   /* 1 = GPIO0 输入 */
#define IOCON_TRIS1                 (1uL << 1)
#define IOCON_LAT0                  (1uL << 8)   /* GPIO0 输出电平 */
#define IOCON_PM0                   (1uL << 24)  /* 1 = GPIO0 做 GPIO（0 = 中断脚） */
#define IOCON_PM1                   (1uL << 25)
#define IOCON_INTOD                 (1uL << 30)  /* 1 = nINT 开漏；0 = 推挽 */

/* ---- ECCCON（寄存器 3-5）---- */
#define ECCCON_ECCEN                (1uL << 0)

/* ---- CiCON（寄存器 3-7）---- */
#define CiCON_ISOCRCEN              (1uL << 5)
#define CiCON_PXEDIS                (1uL << 6)
#define CiCON_RTXAT                 (1uL << 16)  /* 1 = 重发次数由各 FIFO 的 TXAT 决定 */
#define CiCON_OPMOD_SHIFT           21u
#define CiCON_OPMOD_MASK            (7uL << 21)
#define CiCON_REQOP_SHIFT           24u
#define CiCON_MODE_NORMAL_FD        0u
#define CiCON_MODE_CONFIG           4u

/* ---- CiNBTCFG / CiDBTCFG（寄存器 3-8/3-9）----
   布局相同：BRP[31:24] TSEG1[23:16] TSEG2[14:8] SJW[6:0]，全部"减 1"编码。

   40MHz SYSCLK，无 PLL，BRP=0 → tq = 25ns，和 G4 侧（80MHz/Prescaler=2）
   的 tq 相同，位时序可以配成**完全一致**：
     仲裁 500kbps = 80 tq：TSEG1=68(69tq), TSEG2=9(10tq), SJW=9 → 采样点 87.5%
     数据 2Mbps   = 20 tq：TSEG1=15(16tq), TSEG2=2(3tq),  SJW=2 → 采样点 85%
   （2026-08 从 1M/5M 降速：5Mbps 实测在台架线束上不稳定。） */
#define NBTCFG_500K_40M             ((0uL << 24) | (68uL << 16) | (9uL << 8) | 9uL)
#define DBTCFG_2M_40M               ((0uL << 24) | (15uL << 16) | (2uL << 8) | 2uL)

/* ---- CiTDC（寄存器 3-10）----
   TDCMOD[17:16] = 10（自动：芯片实测环路延迟再加 TDCO），
   TDCO[14:8] = 17（= 数据段采样点 1+TSEG1 = 17 mtq，25ns 一格 → 425ns，
   与 G4 侧 TDC 位置一致）。 */
#define TDC_AUTO_2M                 ((2uL << 16) | (17uL << 8))

/* ---- CiINT（寄存器 3-14）：低 16 位是标志，高 16 位是对应使能 ---- */
#define CiINT_RXIF                  (1uL << 1)
#define CiINT_RXOVIF                (1uL << 11)
#define CiINT_RXIE                  (1uL << 17)
#define CiINT_RXOVIE                (1uL << 27)

/* ---- CiFIFOCONm（寄存器 3-29）---- */
#define FIFOCON_TFNRFNIE            (1uL << 0)   /* RX：非空中断使能 */
#define FIFOCON_RXOVIE              (1uL << 3)
#define FIFOCON_TXEN                (1uL << 7)   /* 1 = 该 FIFO 是发送 FIFO */
#define FIFOCON_UINC                (1uL << 8)
#define FIFOCON_TXREQ               (1uL << 9)
/* TXAT[22:21] = 00 → 不重发（一次定生死），配合 CiCON.RTXAT=1。
   SYNC 过期即作废，重发出去的旧 SYNC 比没有更糟 —— 与原生 FDCAN 侧
   AutoRetransmission=DISABLE 语义一致。 */
#define FIFOCON_FSIZE(n)            (((uint32_t)((n) - 1u)) << 24)
#define FIFOCON_PLSIZE_8B           (0uL << 29)

/* ---- CiFIFOSTAm（寄存器 3-30）---- */
#define FIFOSTA_TFNRFNIF            (1uL << 0)   /* TX：未满 / RX：非空 */
#define FIFOSTA_RXOVIF              (1uL << 3)

/* ---- 滤波器（寄存器 3-32/33/34）---- */
#define FLTCON_FLTEN                0x80u        /* 每滤波器 1 字节里的使能位 */
#define MASK_MIDE                   (1uL << 30)  /* 连 IDE 一起比对（只收标准帧） */

/* ---- 消息对象（表 3-5 / 3-6）----
   T0/R0: SID[10:0]；T1/R1: DLC[3:0], IDE(4), RTR(5), BRS(6), FDF(7)。
   载荷字节序：字节 0 在字的最低位（表 3-5：Byte0 = bits[7:0]），
   即整个对象按小端字节流读写，正好和 SPI 逐字节传输的顺序一致。 */
#define OBJ_T1_DLC(n)               ((uint32_t)(n) & 0x0Fu)
#define OBJ_T1_BRS                  (1uL << 6)
#define OBJ_T1_FDF                  (1uL << 7)

/* FIFO 编号分配 */
#define FIFO_TX_SYNC                1u
#define FIFO_RX_JOINT               2u
#define RX_OBJ_BYTES                16u          /* R0(4)+R1(4)+载荷(8)，RXTSEN=0 */

/* ==========================================================================
 * 芯片描述表
 * ========================================================================== */

typedef struct
{
  SPI_TypeDef   *spi;
  GPIO_TypeDef  *cs_port;
  uint16_t       cs_pin;
} McpChipHw_t;

Mcp_Stats_t g_mcp;

static McpChipHw_t s_hw[MCP_CHIP_CNT];   /* 在 Mcp_Init 里填（main.h 的宏此处可用） */

static volatile uint8_t s_spi_busy[MCP_CHIP_CNT] = { 0u, 0u };  /* 主循环占用 SPI 中 */
static volatile uint8_t s_staged[MCP_CHIP_CNT]   = { 0u, 0u };  /* SYNC 已预装载    */
static volatile uint8_t s_drain_req[MCP_CHIP_CNT] = { 0u, 0u }; /* nINT 置的读回请求 */

static volatile uint32_t s_window_t0   = 0u;
static volatile uint8_t  s_window_open = 0u;

static uint16_t s_fifo1_ua[MCP_CHIP_CNT];   /* FIFO1 的 RAM 地址（1 深，恒定，缓存） */

/* ==========================================================================
 * 寄存器级 SPI（理由见 mcp2518.h 头注释）
 * ========================================================================== */

/** 收发一个字节，带自旋上限。返回 0 = 超时。 */
static uint8_t spi_byte(SPI_TypeDef *spi, uint8_t tx, uint8_t *rx)
{
  uint32_t spin;

  for (spin = 0u; (spi->SR & SPI_SR_TXE) == 0u; spin++)
  {
    if (spin >= MCP_SPI_SPIN_MAX) { return 0u; }
  }
  /* 必须按字节宽度写 DR，写 16 位会一次压进两个字节 */
  *(__IO uint8_t *)&spi->DR = tx;

  for (spin = 0u; (spi->SR & SPI_SR_RXNE) == 0u; spin++)
  {
    if (spin >= MCP_SPI_SPIN_MAX) { return 0u; }
  }
  {
    const uint8_t v = *(__IO uint8_t *)&spi->DR;
    if (rx != NULL) { *rx = v; }
  }
  return 1u;
}

/**
 * @brief 一次完整 SPI 事务：CS 低 → 2 字节命令 → n 字节数据 → CS 高。
 * @param dir_read  1 = 读（数据阶段发 0x00 收数据），0 = 写
 * @return 1 = 成功
 */
static uint8_t spi_xfer(uint8_t chip, uint8_t cmd, uint16_t addr,
                        uint8_t *data, uint16_t len, uint8_t dir_read)
{
  const McpChipHw_t *hw = &s_hw[chip];
  uint8_t ok = 1u;
  uint16_t i;

  HAL_GPIO_WritePin(hw->cs_port, hw->cs_pin, GPIO_PIN_RESET);

  ok &= spi_byte(hw->spi, (uint8_t)((cmd << 4) | ((addr >> 8) & 0x0Fu)), NULL);
  ok &= spi_byte(hw->spi, (uint8_t)(addr & 0xFFu), NULL);

  for (i = 0u; (i < len) && (ok != 0u); i++)
  {
    if (dir_read != 0u)
    {
      ok &= spi_byte(hw->spi, 0x00u, &data[i]);
    }
    else
    {
      ok &= spi_byte(hw->spi, data[i], NULL);
    }
  }

  HAL_GPIO_WritePin(hw->cs_port, hw->cs_pin, GPIO_PIN_SET);

  if (ok == 0u)
  {
    g_mcp.chip[chip].spi_err++;
  }
  return ok;
}

/* 32 位寄存器读写。MCP 的多字节访问按小端排字节（LSB 在低地址）。 */
static uint8_t reg_write32(uint8_t chip, uint16_t addr, uint32_t val)
{
  uint8_t b[4] = { (uint8_t)val, (uint8_t)(val >> 8),
                   (uint8_t)(val >> 16), (uint8_t)(val >> 24) };
  return spi_xfer(chip, MCP_CMD_WRITE, addr, b, 4u, 0u);
}

static uint8_t reg_read32(uint8_t chip, uint16_t addr, uint32_t *val)
{
  uint8_t b[4] = { 0u, 0u, 0u, 0u };
  if (spi_xfer(chip, MCP_CMD_READ, addr, b, 4u, 1u) == 0u) { return 0u; }
  *val = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
         ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
  return 1u;
}

/** 单字节寄存器写（用于点火/UINC：只碰 FIFOCON 的第 1 字节，别的位不动） */
static uint8_t reg_write8(uint8_t chip, uint16_t addr, uint8_t val)
{
  uint8_t b = val;
  return spi_xfer(chip, MCP_CMD_WRITE, addr, &b, 1u, 0u);
}

/* ==========================================================================
 * SYNC 预装载
 * ========================================================================== */

/* SYNC 的 SID（协议值 0x001，与 canfd.c 一致；独立定义避免模块间依赖） */
#define CANFD_SYNC_ID_FOR_MCP       0x001u

/**
 * @brief 把 SYNC 报文写进 FIFO1 的 RAM 槽并置 UINC（入队但不请求发送）。
 * @note  FIFO1 只有 1 深，UA 恒定（初始化时缓存），不用每次读。
 *        对象 = T0(4B) + T1(4B) + 载荷 8B（PLSIZE=8，RAM 按 4 字节对齐写）。
 */
static uint8_t stage_sync(uint8_t chip)
{
  uint8_t obj[16];
  const uint32_t t0 = CANFD_SYNC_ID_FOR_MCP;                 /* SID */
  const uint32_t t1 = OBJ_T1_FDF | OBJ_T1_BRS | OBJ_T1_DLC(1u);

  memset(obj, 0, sizeof(obj));
  obj[0] = (uint8_t)t0;          obj[1] = (uint8_t)(t0 >> 8);
  obj[4] = (uint8_t)t1;
  obj[8] = 0x01u;                /* data[0] = 0x01，协议定死 */

  if (spi_xfer(chip, MCP_CMD_WRITE, s_fifo1_ua[chip], obj, 16u, 0u) == 0u)
  {
    return 0u;
  }
  /* UINC 在 FIFOCON 第 2 个字节（bit8）。单字节写只动这一个字节。 */
  if (reg_write8(chip, REG_FIFOCON(FIFO_TX_SYNC) + 1u, 0x01u) == 0u)
  {
    return 0u;
  }

  s_staged[chip] = 1u;
  g_mcp.chip[chip].restage++;
  return 1u;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

static uint8_t chip_init(uint8_t chip)
{
  uint32_t v = 0u;
  uint32_t deadline;

  /* ---- 0. 使能 SPI 外设 ----
     CubeMX 只做了 HAL_SPI_Init（配寄存器），SPE 是 HAL 收发函数进场才置的；
     我们绕开 HAL 收发，这里手动置一次。 */
  SET_BIT(s_hw[chip].spi->CR1, SPI_CR1_SPE);

  /* ---- 1. 复位 ----
     RESET 指令（C=0000, A=0）：全部寄存器回默认值并进入配置模式。 */
  {
    uint8_t dummy[1];
    if (spi_xfer(chip, MCP_CMD_RESET, 0x000u, dummy, 0u, 0u) == 0u) { return 0u; }
  }
  HAL_Delay(2u);

  /* ---- 2. SPI 通信自检 ----
     写一个已知图案到 CiFLTOBJ0（配置模式下可写、暂未使用）再读回。
     读回不对 = SPI 线/CS/时钟极性有问题，往下走全是白费。 */
  if (reg_write32(chip, REG_FLTOBJ(0u), 0x12345678u) == 0u) { return 0u; }
  if (reg_read32(chip, REG_FLTOBJ(0u), &v) == 0u)           { return 0u; }
  if (v != 0x12345678u)                                     { return 0u; }

  /* ---- 3. 时钟：40MHz 晶体直通，不用 PLL ----
     PLLEN=0，SCLKDIV=0（÷1）→ SYSCLK = 40MHz。然后等 OSCRDY。 */
  if (reg_write32(chip, REG_OSC, 0x00000000u) == 0u) { return 0u; }
  deadline = HAL_GetTick();
  do
  {
    if (reg_read32(chip, REG_OSC, &v) == 0u) { return 0u; }
    if ((v & OSC_OSCRDY) != 0u) { break; }
  } while ((uint32_t)(HAL_GetTick() - deadline) < 10u);
  if ((v & OSC_OSCRDY) == 0u) { return 0u; }

  /* ---- 4. ECC + 消息 RAM 清零 ----
     RAM 带 ECC，出厂内容随机 —— 不初始化就读会触发 ECC 错误。
     先开 ECC 再写全零，让每个字都带上正确的校验位。64 字节一段，32 段。 */
  if (reg_write32(chip, REG_ECCCON, ECCCON_ECCEN) == 0u) { return 0u; }
  {
    uint8_t zeros[64];
    uint16_t off;
    memset(zeros, 0, sizeof(zeros));
    for (off = 0u; off < MCP_RAM_SIZE; off += 64u)
    {
      if (spi_xfer(chip, MCP_CMD_WRITE, (uint16_t)(MCP_RAM_BASE + off),
                   zeros, 64u, 0u) == 0u)
      {
        return 0u;
      }
    }
  }

  /* ---- 5. CAN 控制 + 位时序 + TDC ----
     CiCON：ISOCRCEN=1（ISO 版 CRC，和 G4 一致——G4 的 FDCAN 只支持 ISO），
            PXEDIS=1（协议异常检测关，同 G4 的 ProtocolException DISABLE），
            RTXAT=1（重发策略交给各 FIFO 的 TXAT 位），
            STEF=0 TXQEN=0（不用 TEF/TXQ），REQOP=100 保持配置模式。 */
  if (reg_write32(chip, REG_CiCON,
                  CiCON_ISOCRCEN | CiCON_PXEDIS | CiCON_RTXAT |
                  ((uint32_t)CiCON_MODE_CONFIG << CiCON_REQOP_SHIFT)) == 0u)
  {
    return 0u;
  }
  if (reg_write32(chip, REG_CiNBTCFG, NBTCFG_500K_40M) == 0u) { return 0u; }
  if (reg_write32(chip, REG_CiDBTCFG, DBTCFG_2M_40M)   == 0u) { return 0u; }
  if (reg_write32(chip, REG_CiTDC,    TDC_AUTO_2M)     == 0u) { return 0u; }

  /* ---- 6. FIFO ----
     FIFO1 = TX，1 深，8B 载荷，TXAT=00（不重发，见宏处注释）。
     FIFO2 = RX，16 深，8B 载荷，非空中断 + 溢出中断。 */
  if (reg_write32(chip, REG_FIFOCON(FIFO_TX_SYNC),
                  FIFOCON_TXEN | FIFOCON_FSIZE(1u) | FIFOCON_PLSIZE_8B) == 0u)
  {
    return 0u;
  }
  if (reg_write32(chip, REG_FIFOCON(FIFO_RX_JOINT),
                  FIFOCON_TFNRFNIE | FIFOCON_RXOVIE |
                  FIFOCON_FSIZE(16u) | FIFOCON_PLSIZE_8B) == 0u)
  {
    return 0u;
  }

  /* ---- 7. 验收滤波：SID 0x100~0x107 → FIFO2 ----
     MCP 的滤波是 掩码/匹配 式，凑不出 0x101~0x104 的精确范围；
     掩码 0x7F8 放行 0x100~0x107（多出的 ID 由软件校验挡住，rx_bad 计数）。
     MIDE=1：只收标准帧（EXIDE=0）。 */
  if (reg_write32(chip, REG_FLTOBJ(0u), 0x100u) == 0u)              { return 0u; }
  if (reg_write32(chip, REG_MASK(0u), MASK_MIDE | 0x7F8u) == 0u)    { return 0u; }
  /* FLTCON0 的字节 0 管滤波器 0：FLTEN + 目标 FIFO 号 */
  if (reg_write8(chip, REG_FLTCON0, FLTCON_FLTEN | FIFO_RX_JOINT) == 0u)
  {
    return 0u;
  }

  /* ---- 8. IOCON：收发器退出待机（用户要求：上电即工作）----
     GPIO0 = 输出、驱低（MCP2542FD 的 STBY 高有效，低 = 正常工作）；
     GPIO1 = 输入（板上走线未确认，别乱驱动）；
     INTOD = 0：nINT 推挽输出（板上没有外部上拉，开漏会浮空）。
     ⚠ 勘误：IOCON 必须整字写，字节写会破坏 LAT0/LAT1。 */
  if (reg_write32(chip, REG_IOCON,
                  IOCON_PM0 | IOCON_PM1 | IOCON_TRIS1 /* LAT0=0, TRIS0=0 */) == 0u)
  {
    return 0u;
  }

  /* ---- 9. 中断：FIFO2 收帧 / 溢出 → nINT 引脚 ----
     nINT 拉低的条件：CiINT 里已使能的中断标志任意一个置位。
     触发路径：节点回帧进 FIFO2 → RXIF → nINT 低 →
       芯片0: PA10 EXTI10 → EXTI15_10_IRQHandler（优先级 1）
       芯片1: PA3  EXTI3  → EXTI3_IRQHandler（优先级 1）
     → HAL_GPIO_EXTI_Callback 置读回请求标志，真正的 SPI 读在主循环。 */
  if (reg_write32(chip, REG_CiINT, CiINT_RXIE | CiINT_RXOVIE) == 0u) { return 0u; }

  /* ---- 10. 缓存 FIFO1 的 RAM 地址并预装载第一条 SYNC ----
     FIFOUA 给的是相对 RAM 起点的偏移。FIFO1 只有 1 深，地址恒定。 */
  if (reg_read32(chip, REG_FIFOUA(FIFO_TX_SYNC), &v) == 0u) { return 0u; }
  s_fifo1_ua[chip] = (uint16_t)(MCP_RAM_BASE + (v & 0xFFFu));
  if (stage_sync(chip) == 0u) { return 0u; }

  /* ---- 11. 回读校验 ----
     位时序错了 CAN 是"半通不通"最难查，花 3 次读把它彻底排除。 */
  if (reg_read32(chip, REG_CiNBTCFG, &v) == 0u || v != NBTCFG_500K_40M) { return 0u; }
  if (reg_read32(chip, REG_CiDBTCFG, &v) == 0u || v != DBTCFG_2M_40M)   { return 0u; }

  /* ---- 12. 切 Normal FD 模式 ----
     只写 CiCON 第 4 字节（REQOP 所在），别的配置位不动。 */
  if (reg_write8(chip, REG_CiCON + 3u, CiCON_MODE_NORMAL_FD) == 0u) { return 0u; }
  deadline = HAL_GetTick();
  do
  {
    if (reg_read32(chip, REG_CiCON, &v) == 0u) { return 0u; }
    if (((v & CiCON_OPMOD_MASK) >> CiCON_OPMOD_SHIFT) == CiCON_MODE_NORMAL_FD)
    {
      return 1u;
    }
  } while ((uint32_t)(HAL_GetTick() - deadline) < 10u);

  return 0u;
}

uint8_t Mcp_Init(void)
{
  uint8_t ok_mask = 0u;
  uint8_t chip;

  memset(&g_mcp, 0, sizeof(g_mcp));

  /* 硬件表在这里填（main.h 的引脚宏在函数体里用最稳妥） */
  s_hw[0].spi = SPI2;  s_hw[0].cs_port = SPI2_CS_GPIO_Port;  s_hw[0].cs_pin = SPI2_CS_Pin;
  s_hw[1].spi = SPI3;  s_hw[1].cs_port = SPI3_CS_GPIO_Port;  s_hw[1].cs_pin = SPI3_CS_Pin;

  for (chip = 0u; chip < MCP_CHIP_CNT; chip++)
  {
    s_staged[chip]    = 0u;
    s_drain_req[chip] = 0u;
    s_spi_busy[chip]  = 0u;

    if (chip_init(chip) != 0u)
    {
      g_mcp.chip[chip].started = 1u;
      ok_mask |= (uint8_t)(1u << chip);
    }
  }

  return ok_mask;
}

void Mcp_SetXcvrStandby(uint8_t chip, uint8_t standby)
{
  uint32_t iocon = IOCON_PM0 | IOCON_PM1 | IOCON_TRIS1;

  if (chip >= MCP_CHIP_CNT)
  {
    return;
  }
  if (standby != 0u)
  {
    iocon |= IOCON_LAT0;   /* STBY 拉高 = 收发器待机 */
  }
  /* 整字写（勘误要求），其余配置位随写恢复成初始化值 */
  (void)reg_write32(chip, REG_IOCON, iocon);
}

/* ==========================================================================
 * 帧同步钩子（优先级 1 中断上下文）
 * ========================================================================== */

void Mcp_OnFrameSync(uint32_t ts)
{
  uint8_t chip;

  for (chip = 0u; chip < MCP_CHIP_CNT; chip++)
  {
    if (g_mcp.chip[chip].started == 0u)
    {
      continue;
    }

    /* 两个跳过条件：
       - 没预装载好（上一条 SYNC 还没发出去，或重装失败）
       - 主循环正占着这个 SPI（读回帧读到一半）
       都宁可丢这一拍同步也不能在 ISR 里等 —— sync_tx_fail 可见。 */
    if ((s_staged[chip] == 0u) || (s_spi_busy[chip] != 0u))
    {
      g_mcp.chip[chip].sync_tx_fail++;
      continue;
    }

    /* 点火：置 FIFOCON1 的 TXREQ（第 2 字节的 bit1）。
       3 字节 SPI 事务，10MHz 下约 2.5us + CS 开销。 */
    if (reg_write8(chip, REG_FIFOCON(FIFO_TX_SYNC) + 1u, 0x02u) != 0u)
    {
      g_mcp.chip[chip].sync_tx++;
      s_staged[chip] = 0u;   /* 用掉了，主循环负责重新装 */
    }
    else
    {
      g_mcp.chip[chip].sync_tx_fail++;
    }
  }

  g_mcp.cycles++;
  s_window_t0   = ts;
  s_window_open = 1u;
}

/* ==========================================================================
 * 主循环
 * ========================================================================== */

/** 读回一颗芯片 FIFO2 里的全部帧并写进 send 缓冲 */
static void drain_rx(uint8_t chip)
{
  uint32_t sta;
  uint32_t ua;
  uint8_t  obj[RX_OBJ_BYTES];
  uint8_t  guard;

  s_spi_busy[chip] = 1u;

  /* 一次最多 16 帧（FIFO 深度），guard 防打转 */
  for (guard = 0u; guard < 16u; guard++)
  {
    if (reg_read32(chip, REG_FIFOSTA(FIFO_RX_JOINT), &sta) == 0u)
    {
      break;
    }

    if ((sta & FIFOSTA_RXOVIF) != 0u)
    {
      g_mcp.chip[chip].rx_ovf++;
      /* 清溢出标志：写 0 到 STA 字节 0 的 RXOVIF 位（其余位只读/写 1 无效） */
      (void)reg_write8(chip, REG_FIFOSTA(FIFO_RX_JOINT), 0x00u);
    }

    if ((sta & FIFOSTA_TFNRFNIF) == 0u)
    {
      break;   /* FIFO 空 */
    }

    if (reg_read32(chip, REG_FIFOUA(FIFO_RX_JOINT), &ua) == 0u)
    {
      break;
    }
    if (spi_xfer(chip, MCP_CMD_READ,
                 (uint16_t)(MCP_RAM_BASE + (ua & 0xFFFu)),
                 obj, RX_OBJ_BYTES, 1u) == 0u)
    {
      break;
    }
    /* UINC：告诉芯片这个槽读完了（FIFOCON 第 2 字节 bit0） */
    if (reg_write8(chip, REG_FIFOCON(FIFO_RX_JOINT) + 1u, 0x01u) == 0u)
    {
      break;
    }

    /* ---- 解析（对象布局见表 3-6，全小端字节流）---- */
    {
      const uint32_t sid = (uint32_t)obj[0] | (((uint32_t)obj[1] & 0x07u) << 8);
      const uint8_t  t1  = obj[4];   /* DLC[3:0] IDE(4) RTR(5) BRS(6) FDF(7) */

      /* 掩码滤波放行了 0x100~0x107，软件把关精确范围 + 帧型 + DLC */
      if (((t1 & (uint8_t)(OBJ_T1_FDF | OBJ_T1_BRS)) != (uint8_t)(OBJ_T1_FDF | OBJ_T1_BRS)) ||
          ((t1 & 0x30u) != 0u) ||                     /* IDE=0 且 RTR=0 */
          ((t1 & 0x0Fu) != 4u) ||                     /* DLC = 4 */
          (sid < GLOVE_JOINT_CAN_ID_BASE) ||
          (sid > GLOVE_BUS_JOINT_ID_MAX(MCP_BUS_BASE + chip)))
      {
        g_mcp.chip[chip].rx_bad++;
        continue;
      }

      {
        /* 载荷从对象第 8 字节起（R0 4B + R1 4B），CAN 上小端 */
        const uint32_t raw32 = ((uint32_t)obj[8])         |
                               ((uint32_t)obj[9]  << 8u)  |
                               ((uint32_t)obj[10] << 16u) |
                               ((uint32_t)obj[11] << 24u);
        const uint32_t node = GLOVE_JOINT_NODE_FROM_ID(sid);           /* 1..4 */
        const uint32_t slot = GLOVE_JOINT_SLOT(MCP_BUS_BASE + chip, node);

        g_mcp.chip[chip].last_joint_raw[node - 1u] = raw32;
        g_mcp.chip[chip].rx_node[node - 1u]++;   /* 自检挂载检测的依据，不分窗口内外 */

        if (RvLink_FramePending() != 0u)
        {
          GloveFrame_SetJoint(RvLink_GetSendBuffer(), slot, raw32);
          RvLink_MarkNodeAlive(GLOVE_HB_JOINT_BIT(slot));
          g_mcp.chip[chip].rx_joint++;
        }
        else
        {
          g_mcp.chip[chip].rx_late++;
        }
      }
    }
  }

  s_spi_busy[chip] = 0u;
}

void Mcp_Poll(void)
{
  uint8_t chip;

  for (chip = 0u; chip < MCP_CHIP_CNT; chip++)
  {
    if (g_mcp.chip[chip].started == 0u)
    {
      continue;
    }

    /* 读回条件：nINT 请求过，或窗口开着（窗口内主动巡一遍，防边沿丢失） */
    if ((s_drain_req[chip] != 0u) || (s_window_open != 0u))
    {
      s_drain_req[chip] = 0u;
      drain_rx(chip);
    }

    /* SYNC 用掉了 → 重新预装载。查 FIFO1 状态：TFNRFNIF=1 表示"未满"，
       1 深的 TX FIFO 未满 = 空 = 上一条已经发出去了。 */
    if (s_staged[chip] == 0u)
    {
      uint32_t sta;
      s_spi_busy[chip] = 1u;
      if (reg_read32(chip, REG_FIFOSTA(FIFO_TX_SYNC), &sta) != 0u)
      {
        if ((sta & FIFOSTA_TFNRFNIF) != 0u)
        {
          (void)stage_sync(chip);
        }
        /* 还没发出去就先不装，下轮再看（总线断的情况 sync_tx_fail 会持续涨） */
      }
      s_spi_busy[chip] = 0u;
    }
  }

  /* 回帧窗口关闭 → 放行闸门（不论收没收齐，缺的由心跳位表达）。
     顺带每 64 帧抄一次错误计数器（CiTREC：REC[15:8] TEC[7:0]）——
     台面上无节点 ACK 时 TEC 会爬到 128（error passive）后稳住，属正常。 */
  if (s_window_open != 0u)
  {
    if ((uint32_t)(FrameSync_Now() - s_window_t0) >= MCP_REPLY_WINDOW_US)
    {
      static uint32_t s_trec_div = 0u;

      s_window_open = 0u;
      RvLink_ProducerDone(RV_PRODUCER_CAN_MCP);

      s_trec_div++;
      if ((s_trec_div & 0x3Fu) == 0u)
      {
        for (chip = 0u; chip < MCP_CHIP_CNT; chip++)
        {
          if (g_mcp.chip[chip].started != 0u)
          {
            uint32_t v;
            s_spi_busy[chip] = 1u;
            if (reg_read32(chip, REG_CiTREC, &v) != 0u)
            {
              g_mcp.chip[chip].trec = v;
            }
            s_spi_busy[chip] = 0u;
          }
        }
      }
    }
  }
}

/* ==========================================================================
 * EXTI 回调（弱符号覆盖）
 * ========================================================================== */

/**
 * @brief nINT 下降沿。
 * @note  触发路径：MCP 的 RXIF/RXOVIF 置位 → nINT 拉低（推挽，INTOD=0）→
 *          PA10 → EXTI15_10_IRQHandler，PA3 → EXTI3_IRQHandler（都是优先级 1）
 *          → HAL_GPIO_EXTI_IRQHandler → 本回调。
 *        只置标志。SPI 读回放主循环 —— 一次读回要几十 us 的 SPI 事务，
 *        放在优先级 1 会挡住 XVS 捕获的同级中断。
 * @note  这是 HAL 的公共弱符号；本工程当前只有 PA3/PA10 两个 EXTI 源，
 *        以后加新的 EXTI 要在这里一起分发。
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_10)        /* PA10 = 芯片 0 的 nINT */
  {
    s_drain_req[0] = 1u;
  }
  else if (GPIO_Pin == GPIO_PIN_3)    /* PA3 = 芯片 1 的 nINT */
  {
    s_drain_req[1] = 1u;
  }
  else
  {
    /* 目前没有其他 EXTI 源 */
  }
}
