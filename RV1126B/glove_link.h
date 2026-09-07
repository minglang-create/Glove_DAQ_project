/* =============================================================================
 * glove_link.h —— RV1126B ↔ STM32G474 手套链路, 协议 v2(定稿见 PROTOCOL_V2.md)
 *
 * 【事务模型 v2】每次 SPI 事务恒定 1345 words(2690B)全双工(长度永不切换):
 *   MOSI = 主→从小包(无则全0=NOP), MISO = 从→主包(小包或数据包, 无则全0)。
 *   Mode0 / MSB / 8bit / CS 整事务不抬 / 首跑 5MHz。字段全部大端。
 *
 * 【握手】PA1(DATA_READY) → RV GPIO0_A4(SoM 球 K13, 默认下拉) = gpiochip0 线 4
 *   高 = MISO 有包就绪。运行态: XVS 后 3~8ms 拉高(等CAN/IMU采集, 超时上限8ms),
 *   保持到下个 XVS。读节奏 = PA1 沿触发 + 电平/轮询兜底(非运行态 100ms 盲轮询)。
 *   ★事务间隔硬约束: 相邻两次事务 ≥100µs(STM32 DMA 重装窗), spi_txn 内强制。★
 *
 * 【CRC】CRC-16/IBM(=ARC): poly 0x8005 反射(0xA001), init 0, 无终异或,
 *   按字节流计算, 大端存放。向量: CRC("123456789")=0xBB3D。
 *
 * 【数据包 0x6A01 布局】(word:字段) 0:MAGIC 1:LENGTH=0x53F 2:CYCLE(u32)
 *   4:HEARTBEAT(u32) 6:JOINT[20](u32×20) 46:TAC_STAT[5] 51:TACTILE[5][16][16]
 *   1331:IMU_RAW[6] 1337:MAG_RAW[3] 1340:QUAT[4](Q14 wxyz) 1344:CRC
 *   ★LENGTH 语义: 不含帧头、不含自身、含 CRC(小包=0x0004, 数据包=0x053F)★
 * ============================================================================= */
#ifndef GLOVE_LINK_H
#define GLOVE_LINK_H

#include <stddef.h>
#include <stdint.h>

/* ---------- 事务/帧几何(改这里=改协议, 有开机自检断言兜底) ---------- */
#define GLV_FRAME_WORDS   1345
#define GLV_FRAME_LEN     (GLV_FRAME_WORDS * 2)      /* 2690 字节 = 恒定事务长度 */
#define GLV_MAGIC         0x6A01
#define GLV_LENGTH_WORDS  1343                       /* 0x053F: 不含帧头/自身, 含 CRC */
#define GLV_CRC_COVER     (GLV_FRAME_LEN - 2)        /* CRC 覆盖 0..2687 */

/* 数据包字段字节偏移 */
#define GLV_OFF_MAGIC        0
#define GLV_OFF_LENGTH       2
#define GLV_OFF_CYCLE        4
#define GLV_OFF_HEARTBEAT    8
#define GLV_OFF_JOINT       12
#define GLV_OFF_TAC_STAT    92
#define GLV_OFF_TACTILE    102
#define GLV_OFF_IMU_RAW   2662
#define GLV_OFF_MAG_RAW   2674
#define GLV_OFF_QUAT      2680
#define GLV_OFF_CRC       2688

#define GLV_NUM_JOINT     20
#define GLV_NUM_TAC        5
#define GLV_TAC_ROWS      16
#define GLV_TAC_COLS      16
#define GLV_TAC_CELLS   (GLV_TAC_ROWS * GLV_TAC_COLS)          /* 256 */
#define GLV_TAC_WORDS   (GLV_NUM_TAC * GLV_TAC_CELLS)          /* 1280 */

/* ---------- 小包(6 words=12B): w0头 w1=0x0004 w2..w4数据 w5=CRC ---------- */
#define GLV_PKT_WORDS      6
#define GLV_PKT_LEN        (GLV_PKT_WORDS * 2)
#define GLV_PKT_LENGTH_VAL 0x0004   /* 不含帧头/自身, 含 CRC */

#define GLV_PKT_COMMTEST   0xAA01   /* 从→主 通信测试 */
#define GLV_PKT_IAM_MASTER 0x5101   /* 主→从 是主机 */
#define GLV_PKT_IAM_SLAVE  0x5201   /* 从→主 是从机 */
#define GLV_PKT_ACK        0x5301   /* 双向  收到 */
#define GLV_PKT_SELFTEST   0x5401   /* 从→主 STM32自检(数据=心跳u32+特殊1w) */
#define GLV_PKT_RV_OK      0x5501   /* 主→从 RV自检完成 */
#define GLV_PKT_RECHECK    0x5F01   /* 主→从 重新自检 */
#define GLV_PKT_START      0xC101   /* 主→从 ★语义修订: 对0xC301的回执"相机就绪"; PAUSED态=恢复采集 */
#define GLV_PKT_START_REQ  0xC301   /* 从→主 开采请求: STM32要求RV启动相机(P0=角色1主2从, P1=1双手0单手) */
#define GLV_PKT_STOP       0xC201   /* 主→从 停采(XVS不停,停装载) */
#define GLV_PKT_LED_CAL    0xE101   /* 主→从 LED标定(数据w2=亮灯cycle数) */
#define GLV_PKT_VER_REQ    0xE201   /* 主→从 版本查询 */
#define GLV_PKT_VER_RSP    0xE301   /* 从→主 版本应答(协议版/固件主/固件次) */
#define GLV_PKT_CAN_TX     0xFD01   /* 主→从 CAN发送 ★保留未实现★ */

/* ---------- 心跳位图(自检包与数据包统一布局) ---------- */
#define GLV_HB_JOINT_BIT(i)   (i)            /* i=0..19 → 关节1..20 */
#define GLV_HB_TAC_BIT(i)     (20 + (i))     /* i=0..4  → 触觉1..5  */
#define GLV_HB_IMU_BIT        25
#define GLV_HB_MAG_BIT        26
#define GLV_HB_PD_BIT         27             /* 自检包:1=PD高压缺失; 数据帧:随保留段恒1 */

#define GLV_JOINT_MISSING     0xFFFFFFFFu
#define GLV_JOINT_ANGLE_MASK  0x007FFFFFu    /* bit[22:0] */

/* ---------- 解析结果 ---------- */
typedef struct {
	uint16_t magic;
	uint16_t length;
	uint32_t cycle;
	uint32_t heartbeat;
	uint32_t joint_raw[GLV_NUM_JOINT];
	uint32_t joint_angle[GLV_NUM_JOINT];
	uint8_t  joint_missing[GLV_NUM_JOINT];
	uint16_t tac_stat[GLV_NUM_TAC];
	uint16_t tac[GLV_NUM_TAC][GLV_TAC_ROWS][GLV_TAC_COLS];
	int16_t  imu_raw[6];                     /* ax ay az gx gy gz */
	int16_t  mag_raw[3];                     /* mx my mz(原始) */
	int16_t  quat[4];                        /* w x y z (Q14) */
	uint16_t crc_recv, crc_calc;
	uint64_t t_read_us;                      /* 本帧读完时刻(CLOCK_MONOTONIC) */
	uint64_t t_edge_ns;                      /* 触发本次读的 PA1 沿内核时间戳(0=电平兜底路径) */
} glove_frame_t;

/* 校验失败位 */
#define GLV_ERR_MAGIC     (1u << 0)
#define GLV_ERR_LENGTH    (1u << 1)
#define GLV_ERR_CRC       (1u << 2)
#define GLV_ERR_TACTILE   (1u << 3)
#define GLV_ERR_JOINT     (1u << 4)
#define GLV_ERR_TAC_STAT  (1u << 5)
#define GLV_ERR_QUAT      (1u << 6)
#define GLV_ERR_HEARTBEAT (1u << 7)
#define GLV_ERR_CYCLE     (1u << 8)
#define GLV_ERR_IMU_RAW   (1u << 9)
#define GLV_ERR_MAG_RAW   (1u << 10)
#define GLV_ERR_SMALLPKT  (1u << 11)  /* 读到的是合法【小包】而非数据包(自检/握手阶段正常现象) */

typedef struct {
	uint32_t err;
	int      tac_bad_idx;         /* 第一个错的触觉字下标, -1=全对 */
	uint16_t tac_expect, tac_got;
	int      joint_bad_idx;
	uint32_t joint_expect, joint_got;
	int      stat_bad_idx;
	uint16_t stat_expect, stat_got;
} glove_check_t;

/* ---------- 统计 ---------- */
typedef struct {
	uint64_t reads, ok;
	uint64_t magic_err, len_err, crc_err;
	uint64_t all_zero;            /* MISO 全 0(从机没装包, 非错误) */
	uint64_t small_pkts;          /* 读到小包的次数 */
	uint64_t torn;
	uint64_t pa1_backlog;         /* 一次 read 到 >1 个 PA1 沿时多出的沿数: >0 = RV 线程曾卡住漏拍(区别于 STM32 没拉 PA1) */
	uint64_t cycle_gap_ev, cycle_dropped, cycle_dup;
	uint64_t pattern_err;
	uint32_t first_cycle, last_cycle;
	double   rate_hz;
} glove_stats_t;

/* ---------- API ---------- */
uint16_t glv_crc16_arc(const uint8_t *d, size_t n);
int  glove_selftest(void);
int  glove_open(const char *spidev, uint32_t hz, int gpiochip, int gpioline);
void glove_close(void);

/* 把一个主→从小包排队, 由【下一次事务】的 MOSI 捎带(队列深度1, 后队覆盖前队并告警)。
 * data 可为 NULL(全 0xFFFF)。 */
int  glove_queue_pkt(uint16_t head, const uint16_t data[3]);

/* 非运行态用: 立即发起一次事务(不等 PA1), MOSI 带队列包(无则 NOP)。
 * 返回: 1=收到合法小包(head/data 填好) 2=收到数据包(忽略内容) 0=MISO 空 -1=CRC错/畸形 */
int  glove_txn_poll(uint16_t *head, uint16_t data[3]);

/* 运行态用: 等 PA1 沿 → 一次事务(捎带队列包) → 解析数据包。
 * 返回 0=有效数据包; >0=GLV_ERR_* 位或(含 GLV_ERR_SMALLPKT); -1=退出/底层错。 */
int  glove_read_frame(glove_frame_t *f, uint8_t *raw, volatile int *quit);

void glove_check_fake(const glove_frame_t *f, glove_check_t *out);
const glove_stats_t *glove_stats(void);
void glove_stats_reset(void);
const char *glove_err_str(uint32_t err);

/* PA1 就绪回调: 在 glove_read_frame 检测到 PA1 沿/电平就绪、【发起 SPI 事务之前】被调
 * (FSM 线程内)。用途: 让外接设备的触发与 PA1 对齐(见 ext_uart.h), 不吃 SPI 的 4.3ms。 */
void glove_set_edge_cb(void (*cb)(void *user), void *user);

/* 兼容旧 CLI(-C): head!=0 时等价于 glove_queue_pkt(head, NULL) */
void glove_set_cmd(uint16_t head);

#endif /* GLOVE_LINK_H */
