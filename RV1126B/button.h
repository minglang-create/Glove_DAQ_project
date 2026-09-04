/* =============================================================================
 * button.h —— 按键(GPIO 低有效, 30ms 去抖, 2s 长按)。chip<0 = 无按键(用信号模拟)。
 * 回调运行在按键线程, 只做置标志/queue 包这类轻活。
 * ============================================================================= */
#ifndef BUTTON_H
#define BUTTON_H
typedef void (*btn_cb)(int long_press, void *user);   /* long_press: 0=短按 1=长按 */
int  btn_start(int chip, int line, btn_cb cb, void *user);
void btn_stop(void);
#endif
