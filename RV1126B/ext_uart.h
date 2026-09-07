/* =============================================================================
 * ext_uart.h —— 外接转接板 STM32 的 21 路关节 ADC(串口, 单向回传)
 *
 * 【物理链路】RV UART0(球 A3=TX / A4=RX, RVDB 立贴座, 460800 8N1)↔ 转接板 STM32。
 *   对端只能 TX 给我们; 我们的 TX 接到对端一个普通 GPIO(不是 RX) → 对端收不了字节,
 *   但能看到电平沿。★占用 UART0 = 放弃串口控制台(adb 不受影响), 由 dts 变体切换★
 *
 * 【方案一 trig(主模式, 确定性同步)】每拍 PA1 沿一到(SPI 事务之前!)RV 发一个 0x00:
 *   8N1 的 0x00 = 起始位+8 个 0 = 连续 9 个位时间的低电平(460800 下 ≈19.5µs), 对端
 *   EXTI 下降沿 → 采 21 路(<1ms)→ 立刻回 46B 帧(1.0ms)。一问一答: 回来的帧就是
 *   本拍的, 归属由构造保证; 采样点 = PA1 + 几十 µs, 抖动 ≈ PA1 的 σ(实测 173µs)。
 *   RV 侧: 回帧须在触发后 EXT_WINDOW_MS 内到, 否则本拍记缺失、迟到帧丢弃,
 *   绝不顶到下一拍。对端配合: 线路安静 ≥10ms 后的单个脉冲才算触发(滤掉开机日志的
 *   连续比特流); 上一帧没发完又来触发则跳过本拍(别排队)。
 * 【方案二 free(备用)】对端 300Hz 自由发, RV 只留最新; 每拍取最新值并记帧龄。
 *   零硬件差别, 触发链不稳时一键切换(-U ...:free)。
 *
 * 【帧格式(对端定义, 固定 46 字节, 全小端)】
 *   [0]=0xA5  [1..2]=seq(u16, 每帧+1)  [3..44]=ch[0..20](u16×21, 12bit ADC 0~4095,
 *   0xFFFF=该 ADC 本轮没采到)  [45]=XOR(字节 1..44)
 * ============================================================================= */
#ifndef EXT_UART_H
#define EXT_UART_H
#include <stdint.h>

#define EXT_NCH        21
#define EXT_FRAME_LEN  46
#define EXT_HEAD       0xA5
#define EXT_CH_MISSING 0xFFFF
#define EXT_WINDOW_MS  10      /* trig 模式: 触发→回帧最长等待(拍周期 16.7ms 的 60%) */

typedef struct {
	int      valid;            /* 1=本拍有帧 */
	uint16_t seq;              /* 对端帧序号 */
	uint16_t ch[EXT_NCH];
	int64_t  latency_us;       /* trig: 触发→到齐; free: 取用时的帧龄 */
	uint64_t t_rx_us;          /* 帧到齐时刻(CLOCK_MONOTONIC) */
} ext_frame_t;

typedef struct {
	uint64_t trig_sent, frames_ok, xor_err, resync, late_drop, missing, seq_gap;
	double   lat_avg_us;       /* 最近 64 帧平均延时 */
	int64_t  lat_max_us;
} ext_stats_t;

/* mode: 1=trig(方案一) 0=free(方案二)。失败返回 -1。 */
int  ext_uart_open(const char *dev, int baud, int trig_mode);
void ext_uart_close(void);
int  ext_uart_enabled(void);

/* 作为 glove_link 的 PA1 沿回调注册(FSM 线程内被调): trig 模式下发 0x00 并记录触发 */
void ext_uart_on_edge(void *user);

/* FSM 读到本拍手套帧后调: trig 模式等本次触发的回帧(最多到触发+EXT_WINDOW_MS);
 * free 模式取最新。返回 0 且 out->valid=1 表示有帧。 */
int  ext_uart_get_last(ext_frame_t *out);

const ext_stats_t *ext_uart_stats(void);
#endif
