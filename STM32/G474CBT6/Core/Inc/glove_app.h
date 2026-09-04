/**
  ******************************************************************************
  * @file    glove_app.h
  * @brief   应用层入口：把所有业务模块的初始化和主循环收在一处
  *
  * ============================================================================
  * 为什么要有这个文件
  * ============================================================================
  *   main.c 是 CubeMX 生成的文件。虽然 USER CODE 段在重新生成时会被保留，
  *   但只要 CubeIDE 的编辑器里还开着 main.c 的旧缓冲，一次保存就可能把
  *   外部工具的改动盖掉（我们已经踩到两次）。
  *
  *   所以约定：main.c 里**永远只有 3 行**与本项目相关的代码 ——
  *     USER CODE BEGIN Includes :  #include "glove_app.h"
  *     USER CODE BEGIN 2        :  GloveApp_Init();
  *     USER CODE BEGIN 3        :  GloveApp_Loop();
  *   后续加 FDCAN、MCP2518FD、IMU 全部往 glove_app.c 里加，再也不碰 main.c。
  *   万一 main.c 又被冲掉，只需要重新贴这 3 行。
  ******************************************************************************
  */

#ifndef GLOVE_APP_H
#define GLOVE_APP_H

/**
 * @brief  初始化全部业务模块。在 main() 里所有 MX_xxx_Init() 之后调用一次。
 * @note   内部顺序有依赖，不要改：详见 glove_app.c 的注释。
 */
void GloveApp_Init(void);

/**
 * @brief  主循环体。在 main() 的 while(1) 里反复调用。
 * @note   非阻塞。当前只做"封帧 + 装 DMA + 拉高 PA1"，约 100us，
 *         没有事情做时立即返回。
 */
void GloveApp_Loop(void);

/**
 * @brief  XVS 节拍到来时的统一分发点。由 FrameSync_Handle() 在 TIM2 捕获中断
 *         （NVIC 优先级 1）里调用。
 * @param  ts  TIM2 CCR3 硬件锁存的时间戳（1us 单位）
 * @note   中断上下文，必须极短。里面只允许做寄存器写和置标志，
 *         所有耗时的事情（算 CRC、I2C 读、解析）都放 GloveApp_Loop()。
 * @note   第 2 步会在这里加"向 5 路 CAN 广播 SYNC 帧"，而且要放在最前面 ——
 *         SYNC 的发出时刻决定了全系统的同步精度，不能被别的事情推迟。
 */
void GloveApp_OnFrameSync(uint32_t ts);

#endif /* GLOVE_APP_H */
