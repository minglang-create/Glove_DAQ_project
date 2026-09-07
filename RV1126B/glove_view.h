/* =============================================================================
 * glove_view.h —— 手套数据的终端可视化(与协议层解耦: 这里只管"怎么看")
 *
 * 视图用 -w 选择, 各视图都是原地刷屏(ANSI), 默认 15Hz 重绘(60Hz 全打人眼跟不上,
 * 而且会把终端刷爆)。协议解析/校验一律不在这里做, 只消费 glove_frame_t。
 *
 *   hdr      默认: 不刷屏, 每秒一行状态(适合长跑/重定向日志)
 *   imu      IMU 专用: 四元数原始+归一化+模长+欧拉角+抖动统计+波形
 *   joint    20 路关节角 + 缺失标记 + 心跳位
 *   tactile  5 片 16x16 触觉热力图(灰阶块) + 每片 min/max/均值
 *   all      帧头 + IMU + 关节 + 触觉概览(一屏总览)
 * ============================================================================= */
#ifndef GLOVE_VIEW_H
#define GLOVE_VIEW_H

#include "glove_link.h"

typedef enum { GV_HDR = 0, GV_IMU, GV_JOINT, GV_TACTILE, GV_ALL } gv_mode_e;

/* 按名字设视图; 返回 0 成功, -1 名字不认识(调用方打印可选值) */
int  gv_set_mode(const char *name);
int  gv_is_mode_name(const char *s);   /* 1 = 这是个视图名(用于参数纠错提示) */
const char *gv_mode_name(void);
void gv_set_hz(int hz);           /* 重绘频率, 默认 15 */
void gv_set_role(const char *role); /* "主机"/"从机": 显示在每个视图顶行 */

/* 每收到一帧调一次(含坏帧, err 是 GLV_ERR_* 位或)。
 * 内部自己限速重绘, 调用方不用管节奏。hdr 模式下本函数只累积不打印。 */
void gv_on_frame(const glove_frame_t *f, uint32_t err, const glove_stats_t *st);

/* ---- 视图模式下相机侧不再自己打印, 把统计交给视图统一渲染(避免刷屏被顶乱) ---- */
void gv_cam_fps(const char *label, double fps);        /* 各路实时帧率 */
void gv_cam_pair(unsigned pair_seq, long long dpts_us);/* 每配成一对帧 */
int  gv_active(void);                                  /* 1 = 当前是刷屏视图(非 hdr) */

/* 退出时恢复终端(显示光标等) */
void gv_finish(void);

#endif /* GLOVE_VIEW_H */
