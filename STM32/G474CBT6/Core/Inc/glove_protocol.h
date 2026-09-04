/**
  ******************************************************************************
  * @file    glove_protocol.h
  * @brief   数据手套 STM32 <-> RV1126B SPI 帧协议定义（唯一真相源）
  *
  *          本文件只含常量和布局定义，不依赖 HAL / CMSIS，
  *          RV1126B 侧可以直接拷这个文件用，两边永远不会对不上。
  *
  * ============================================================================
  * 物理层
  * ============================================================================
  *   STM32 = SPI 从机，RV1126B = SPI 主机（RV 的 SPI0_M2）
  *   - 时钟极性/相位 : CPOL = 0（空闲低）, CPHA = 0（第一个边沿采样）→ SPI Mode 0
  *   - 数据宽度      : 16 bit / 帧
  *   - 位序          : MSB first
  *   - 片选          : 硬件 NSS，低有效（STM32 PA4）
  *   - 一次 NSS 拉低 = 一个完整数据帧（GLOVE_FRAME_WORDS 个 16bit 字）
  *
  *   字节序说明（重要）：
  *     SPI 配成 16bit + MSB first，所以每个 uint16 在线上自动是"高字节先出"。
  *     32 位量（关节角度）拆成两个 uint16 时，**高 16 位放低数组下标**，即先发高半字。
  *       word[n]   = (u32 >> 16) & 0xFFFF
  *       word[n+1] =  u32        & 0xFFFF
  *
  * ============================================================================
  * 握手（PA1，STM32 输出 → RV 输入）—— 全阶段统一语义
  * ============================================================================
  *   HIGH = STM32 有一个新装载的出站包可读（自检包/角色包/数据包都一样）
  *   LOW  = 没有新包（正在重建，或上一包已被读走且无新内容）
  *   RV 用上升沿触发读取，不需要盲轮询。
  *   运行态的节奏：XVS 同步沿 → PA1 拉低 → 组帧+CRC+装 DMA（数 ms，等生产者）
  *                → PA1 拉高直到下一个 XVS。
  *   注意：PA1 的上升沿时刻相对 XVS 有毫秒级可变延迟，**不可用作视频对齐**；
  *   视频↔数据对齐用帧内周期号 ↔ 相机帧号（两者都锚定在 XVS 上）。
  *
  * ============================================================================
  * 事务模型（方案 A：全程恒定长度，2026-09-02 定稿）
  * ============================================================================
  *   任何阶段、任何方向，每次 SPI 事务都是 **GLOVE_FRAME_WORDS(1345) 字**：
  *     MISO：运行态 = 数据帧；其余状态 = 小包占开头 6 字 + 全 0 填充
  *           （字 0 = 0x0000 不等于任何帧头，天然可辨"没有包"）
  *     MOSI：前 6 字 = 可选的 RV→STM32 小包（无命令填 0），其余填充
  *   从机长度永远不变 → 不存在失步问题；小包/数据帧靠帧头区分。
  *   RV 侧注意相邻两次事务之间留 ≥100us（STM32 重装 DMA 的窗口）。
  *
  * ============================================================================
  * 一帧延迟流水线
  * ============================================================================
  *   第 N 个 XVS 到来时，冻结的是第 N-1 个周期采集到的数据。
  *   帧内 CYCLE 字段 = 这批数据所属的周期号，RV 拿它去配对同一周期的图像。
  ******************************************************************************
  */

#ifndef GLOVE_PROTOCOL_H
#define GLOVE_PROTOCOL_H

#include <stdint.h>

/* ==========================================================================
 * 1. 帧头与版本
 * ========================================================================== */

/** 帧头魔数。低字节 0x01 即协议版本号，协议改动时递增（0x6A02, 0x6A03 ...） */
#define GLOVE_MAGIC                 0x6A01u
#define GLOVE_PROTO_VERSION         0x01u

/* ==========================================================================
 * 2. 传感器数量
 * ========================================================================== */

#define GLOVE_JOINT_CNT             20u   /**< 关节角度传感器：5 路 CAN × 4 个        */
#define GLOVE_TACTILE_CNT           5u    /**< 触觉传感器：5 个                       */
#define GLOVE_TACTILE_ROWS          16u
#define GLOVE_TACTILE_COLS          16u
#define GLOVE_TACTILE_CELLS         (GLOVE_TACTILE_ROWS * GLOVE_TACTILE_COLS)  /* 256 */

/* ==========================================================================
 * 3. 帧布局（偏移单位 = uint16 字，不是字节）
 * ========================================================================== */

#define GLOVE_OFF_MAGIC             0u      /**< 1 字：GLOVE_MAGIC                    */
#define GLOVE_OFF_LENGTH            1u      /**< 1 字：见 GLOVE_LENGTH_VALUE          */
#define GLOVE_OFF_CYCLE             2u      /**< 2 字：周期号 uint32，高字在前         */
#define GLOVE_OFF_HEARTBEAT         4u      /**< 2 字：心跳位图 uint32，高字在前       */
#define GLOVE_OFF_JOINT             6u      /**< 40 字：20 个关节 × 2 字，高字在前     */
#define GLOVE_OFF_TACTILE_STAT      46u     /**< 5 字：每个触觉节点 1 字状态           */
#define GLOVE_OFF_TACTILE           51u     /**< 1280 字：5 × 256 阵列                */
#define GLOVE_OFF_IMU_RAW           1331u   /**< 6 字：六轴原始 int16，ax ay az gx gy gz
                                                 （本周期最后一个样本，最贴近同步沿）  */
#define GLOVE_OFF_MAG               1337u   /**< 3 字：磁力计原始 int16 x y z
                                                 （桥接未启用时为 0 + 心跳位=1）       */
#define GLOVE_OFF_QUAT              1340u   /**< 4 字：int16 Q14 四元数 w,x,y,z       */
#define GLOVE_OFF_CRC               1344u   /**< 1 字：CRC-16/ARC                     */

/** 整帧字数 = 1345；字节数 = 2690 */
#define GLOVE_FRAME_WORDS           1345u
#define GLOVE_FRAME_BYTES           (GLOVE_FRAME_WORDS * 2u)

/**
 * LENGTH 字段的值（2026-09 定稿语义，全部包统一）：
 * **不含帧头、不含 LENGTH 自身、含其后全部内容（包括 CRC）**。
 * = GLOVE_FRAME_WORDS - 2 = 1343
 */
#define GLOVE_LENGTH_VALUE          (GLOVE_FRAME_WORDS - 2u)

/* ==========================================================================
 * 4. 关节角度字段
 * ========================================================================== */

/**
 * 关节角度是 32 位量，拆成 2 个 uint16 存放（高字在前）。
 *   bit[22:0]  = TMR3111 的 23 位角度原始值（有效数据）
 *   bit[31:23] = 预留，将来放诊断信息（每总线 bus-off、FIFO 丢帧计数等）
 * 节点缺失时整个 32 位填 GLOVE_JOINT_INVALID，同时心跳对应位置 1。
 */
#define GLOVE_JOINT_ANGLE_BITS      23u
#define GLOVE_JOINT_ANGLE_MASK      0x007FFFFFu
#define GLOVE_JOINT_INVALID         0xFFFFFFFFu

/** 关节 i（0..19）在帧中的字偏移（该关节的高半字） */
#define GLOVE_JOINT_WORD_OFF(i)     (GLOVE_OFF_JOINT + (i) * 2u)

/* ==========================================================================
 * 4b. 关节节点编址与映射
 * ==========================================================================
 *
 * 【三套编号，别混】
 *
 *   ① CAN 帧 ID     —— 线上的东西。每条总线内唯一、跨总线重复。
 *                      关节节点用 0x101 ~ 0x104（= 0x100 + 本总线节点号 1..4）
 *
 *   ② 全局编号       —— 给人看的（日志、花名册、装配时贴标签）。
 *                      = 总线序号 * 10 + 本总线节点号，即
 *                        第 1 路: 1, 2, 3, 4
 *                        第 2 路: 11, 12, 13, 14
 *                        第 3 路: 21, 22, 23, 24
 *                        第 4 路: 31, 32, 33, 34
 *                        第 5 路: 41, 42, 43, 44
 *                      步长 10 是为了将来某条总线加到 5~9 个节点时编号不用重排。
 *
 *   ③ 帧内槽位       —— 帧里那 20 个连续槽的下标 0..19，**紧凑无空洞**。
 *                      = 总线序号 * 4 + (本总线节点号 - 1)
 *
 * RV 侧解析只需要认 ③。① 和 ② 是 STM32 内部的事。
 *
 * 【注意】槽位总数固定 20（帧长不变）；每条总线几个节点由下面的拓扑开关
 *        决定。改动拓扑只影响 (总线,节点)→槽位 的映射，不影响 RV 解析。
 */

#define GLOVE_CAN_BUS_CNT           5u    /**< 5 路 CAN：3 原生 FDCAN + 2 颗 MCP2518FD */
#define GLOVE_JOINT_ID_STRIDE       10u   /**< 全局编号的每路步长（留余量）              */

/**
 * 传感器拓扑模式（编译期选择，2026-09-02 扩成三态）：
 *   0 = 正式手套：5 路 CAN × 4 关节节点（原方案）
 *   1 =【已废弃，勿选】曾以为"临时转接板"上挂 7+7+6 个关节协议节点，
 *       实际那块板就是模式 2 的 ADC2CAN 板（用户 2026-09-02 澄清），
 *       此模式没有对应硬件。保留仅作多节点拓扑映射的代码参考。
 *   2 = 碳膜 ADC2CAN 板（即"转接板"）：3 路原生 CAN（bus0/1/2 =
 *       FDCAN1/2/3 = 接插件 FPC1/FPC5/FPC4），
 *       触发帧 = 现有 SYNC（ID 0x001，内容不解析），板子去重后一轮采
 *       21 路 12bit ADC，每路总线回一帧 16 字节（0x201/0x202/0x203）。
 *       ADC1..20 → 关节槽 0..19；**ADC21 直接丢弃**（2026-09-02 定：
 *       对 RV 与 20 关节方案零差异，触觉字段/心跳 bit20 保持原语义）。
 * 帧布局对 RV 恒定不变（0..19 共 20 槽），只是数据来源和语义随模式变。
 */
#define GLOVE_SENSOR_MODE           2

#if (GLOVE_SENSOR_MODE == 2)
/* ---- 碳膜 ADC2CAN（协议 v1.0）---- */
#define GLOVE_A2C_GROUPS            3u     /**< 3 路总线各一组                */
#define GLOVE_A2C_CH_PER_GRP        7u     /**< 每组 7 路 ADC                 */
#define GLOVE_A2C_ID(group)         (0x200u + (group))   /**< 回帧 ID 0x201..0x203 */
#define GLOVE_A2C_ADC_MAX           4095u  /**< 12bit ADC 值域                */
/* 回帧布局：Byte0=frame_seq(u8 循环)、Byte1=group(1..3)、
   Byte2..15 = ch[0..6] uint16 小端 */

#define GLOVE_JOINTS_MAX_PER_BUS    7u
/** 有效通道数：bus0/1 各 7 路、bus2 只算 6 路（第 21 路 ADC 丢弃）；
    bus3/4（MCP）无节点。槽基仍按 ×7 排（0/7/14），bus2 槽 14..19。 */
#define GLOVE_BUS_JOINT_CNT(bus)    ((bus) <= 1u ? GLOVE_A2C_CH_PER_GRP : \
                                     ((bus) == 2u ? (GLOVE_A2C_CH_PER_GRP - 1u) : 0u))
#define GLOVE_BUS_SLOT_BASE(bus)    ((bus) <= 2u ? ((bus) * GLOVE_A2C_CH_PER_GRP) : 0xFFu)

#elif (GLOVE_SENSOR_MODE == 1)
/* ---- 临时转接板：每路节点数（三者之和必须为 20；每路上限 7 ——
   受 MCP 硬件掩码窗口 0x101~0x107 限制）。 ---- */
#define GLOVE_RIG_CNT_BUS2          7u    /**< FDCAN3 上的节点数 */
#define GLOVE_RIG_CNT_BUS3          7u    /**< MCP#1  上的节点数 */
#define GLOVE_RIG_CNT_BUS4          6u    /**< MCP#2  上的节点数 */

#define GLOVE_JOINTS_MAX_PER_BUS    7u    /**< 统计数组按最大值定长 */

/** 各总线的节点数（bus0/1 = 0，接收滤波直接关死） */
#define GLOVE_BUS_JOINT_CNT(bus)                                  \
            ((bus) == 2u ? GLOVE_RIG_CNT_BUS2 :                   \
             (bus) == 3u ? GLOVE_RIG_CNT_BUS3 :                   \
             (bus) == 4u ? GLOVE_RIG_CNT_BUS4 : 0u)

/** 各总线在帧里的槽位起点：bus2 从 0 起，bus3、bus4 依次接排 */
#define GLOVE_BUS_SLOT_BASE(bus)                                  \
            ((bus) == 2u ? 0u :                                   \
             (bus) == 3u ? GLOVE_RIG_CNT_BUS2 :                   \
             (bus) == 4u ? (GLOVE_RIG_CNT_BUS2 + GLOVE_RIG_CNT_BUS3) : 0xFFu)

#else /* 正式手套：5 路 × 4 节点 */

#define GLOVE_JOINTS_PER_BUS        4u    /**< 每路 4 个关节节点 */
#define GLOVE_JOINTS_MAX_PER_BUS    GLOVE_JOINTS_PER_BUS
#define GLOVE_BUS_JOINT_CNT(bus)    GLOVE_JOINTS_PER_BUS
#define GLOVE_BUS_SLOT_BASE(bus)    ((bus) * GLOVE_JOINTS_PER_BUS)

#endif /* GLOVE_SENSOR_MODE */

/** 关节节点在本总线上的 CAN 帧 ID 基址：ID = GLOVE_JOINT_CAN_ID_BASE + (节点号-1) */
#define GLOVE_JOINT_CAN_ID_BASE     0x101u

/** 本总线允许的最大关节 ID（随该总线节点数变） */
#define GLOVE_BUS_JOINT_ID_MAX(bus) \
            (GLOVE_JOINT_CAN_ID_BASE + GLOVE_BUS_JOINT_CNT(bus) - 1u)

/** CAN 帧 ID → 本总线节点号（1 起）。调用前需先确认 ID 在合法区间内。 */
#define GLOVE_JOINT_NODE_FROM_ID(can_id) \
            ((can_id) - GLOVE_JOINT_CAN_ID_BASE + 1u)

/** (总线序号, 本总线节点号 1 起) → 帧内槽位 0..19 */
#define GLOVE_JOINT_SLOT(bus, node) \
            (GLOVE_BUS_SLOT_BASE(bus) + ((node) - 1u))

/** (总线序号 0..4, 本总线节点号 1..4) → 全局编号（1,2,3,4,11,12,...,44） */
#define GLOVE_JOINT_GLOBAL_ID(bus, node) \
            ((bus) * GLOVE_JOINT_ID_STRIDE + (node))

/* ==========================================================================
 * 5. 触觉字段
 * ========================================================================== */

/** 触觉节点 s（0..4）的 256 字阵列起始字偏移 */
#define GLOVE_TACTILE_WORD_OFF(s)   (GLOVE_OFF_TACTILE + (s) * GLOVE_TACTILE_CELLS)

/** 触觉节点 s 的 (row, col) 单元在帧中的字偏移，行优先 */
#define GLOVE_TACTILE_CELL_OFF(s, row, col) \
            (GLOVE_TACTILE_WORD_OFF(s) + (row) * GLOVE_TACTILE_COLS + (col))

/* ==========================================================================
 * 6. 心跳位图
 * ========================================================================== */

/**
 * 心跳 = uint32 位图，**bit = 1 表示该节点本周期没有数据，bit = 0 表示数据新鲜**。
 * 每个周期重新计算（XVS 到来时整个位图复位成全 1，收到谁的数据就清谁那一位）。
 *
 *   bit  0..19 : 关节 1..20
 *   bit 20..24 : 触觉 1..5
 *   bit 25     : IMU（ICM-45686 六轴）
 *   bit 26     : 磁力计（MMC5603，经 ICM AUX 桥接；暂未启用，恒为 1）
 *   bit 27..31 : 保留，恒为 1
 */
#define GLOVE_HB_JOINT_BIT(i)       (0u  + (i))     /**< i = 0..19 */
#define GLOVE_HB_TACTILE_BIT(i)     (20u + (i))     /**< i = 0..4  */
#define GLOVE_HB_IMU_BIT            25u
#define GLOVE_HB_MAG_BIT            26u

/** 复位值：所有节点都"无数据" */
#define GLOVE_HB_ALL_MISSING        0xFFFFFFFFu

/* ==========================================================================
 * 7. 四元数字段
 * ========================================================================== */

/**
 * 姿态用单位四元数表示，避免欧拉角在俯仰过 ±90° 时的万向锁跳变。
 * 格式：4 个 int16，Q14 定点（数值 = q * 16384），顺序 w, x, y, z。
 * 静止且未初始化时输出单位四元数 (16384, 0, 0, 0)。
 */
#define GLOVE_QUAT_FRAC_BITS        14
#define GLOVE_QUAT_ONE              16384           /**< 1.0 的 Q14 表示 */
#define GLOVE_QUAT_W                0u
#define GLOVE_QUAT_X                1u
#define GLOVE_QUAT_Y                2u
#define GLOVE_QUAT_Z                3u

/* ==========================================================================
 * 8. CRC
 * ========================================================================== */

/**
 * CRC-16/ARC（又名 CRC-16/IBM、CRC-16）
 *   多项式    : 0x8005，反射式实现用 0xA001
 *   初值      : 0x0000
 *   输入反射  : 是
 *   输出反射  : 是
 *   最终异或  : 0x0000
 *   标准校验  : CRC("123456789") = 0xBB3D
 *
 * 覆盖范围：从 GLOVE_OFF_MAGIC 到 GLOVE_OFF_CRC 之前的全部内容（共 1335 字）。
 *
 * 【喂入顺序 —— 最容易出错的地方】
 *   必须按**线上字节序**喂，也就是每个 uint16 先喂高字节、再喂低字节。
 *   不能直接把 uint16 数组当 uint8 数组算（小端 CPU 会先喂低字节，结果不一样）。
 */
#define GLOVE_CRC16_POLY_REFLECTED  0xA001u
#define GLOVE_CRC16_INIT            0x0000u
#define GLOVE_CRC16_CHECK_VALUE     0xBB3Du   /**< "123456789" 的 CRC，用于自检 */

/* ==========================================================================
 * 9. SPI 事务小包（自检 / 角色分配 / 控制，双向通用）
 * ==========================================================================
 *
 * 固定 6 个 uint16：
 *   [0] 帧头（低字节 = 版本 0x01）
 *   [1] 帧长 = 4（不含帧头、不含自身、含 CRC，与数据包同一语义）
 *   [2..4] 载荷 3 字（无意义时填 0xFFFF）
 *   [5] CRC-16/ARC，覆盖 [0..4]，按线上字节序（高字节先）喂入
 *
 * 方向说明："从→主" = STM32 预装载、RV 发起读取取走；
 *           "主→从" = RV 在任意事务的 MOSI 前 6 字里携带。
 */

#define GLOVE_SPKT_WORDS            6u
#define GLOVE_SPKT_LEN_VALUE        4u
#define GLOVE_SPKT_OFF_HDR          0u
#define GLOVE_SPKT_OFF_LEN          1u
#define GLOVE_SPKT_OFF_P0           2u
#define GLOVE_SPKT_OFF_P1           3u
#define GLOVE_SPKT_OFF_P2           4u
#define GLOVE_SPKT_OFF_CRC          5u

/* ---- 包头一览（低字节 0x01 = 版本）---- */
#define GLOVE_PKT_COMMTEST          0xAA01u   /**< 通信测试包    从→主，载荷全 0xFFFF   */
#define GLOVE_PKT_YOU_ARE_MASTER    0x5101u   /**< 是主机        主→从                  */
#define GLOVE_PKT_I_AM_SLAVE        0x5201u   /**< 是从机        从→主                  */
#define GLOVE_PKT_ACK               0x5301u   /**< 主从机收到    双向                    */
#define GLOVE_PKT_SELFTEST          0x5401u   /**< STM32 自检包  从→主，载荷见下         */
#define GLOVE_PKT_RV_ST_DONE        0x5501u   /**< RV 自检完成包 主→从                  */
#define GLOVE_PKT_RETEST            0x5F01u   /**< 重新自检包    主→从                  */
#define GLOVE_PKT_ACQ_START         0xC101u   /**< 开采包        主→从。2026-09-02 起
                                                   语义 = 响应 0xC301"相机已启动就绪"；
                                                   PAUSED 态 = 恢复采集                 */
#define GLOVE_PKT_ACQ_STOP          0xC201u   /**< 停采包        主→从                  */
#define GLOVE_PKT_ACQ_REQ           0xC301u   /**< 开采请求包    从→主（2026-09-02 新增）
                                                   STM32 请求 RV 启动相机并回 0xC101。
                                                   P0 = 本机角色（1=主机 2=从机），
                                                   P1 = 1 双手模式 / 0 单手模式         */
#define GLOVE_PKT_LED_CAL           0xE101u   /**< LED 标定包    主→从（预留：V4 无
                                                   MCU 可控 LED，硬件确认前不实现）  */
#define GLOVE_PKT_VER_QUERY         0xE201u   /**< 版本查询包    主→从                  */
#define GLOVE_PKT_VER_REPLY         0xE301u   /**< 版本应答包    从→主，载荷 =
                                                   协议版本 / 固件主版本 / 固件次版本  */
#define GLOVE_PKT_CAN_SEND          0xFD01u   /**< CAN 发送包    主→从（预留，未实现）   */

/** 固件版本（版本应答包回带；协议版本 = 帧头低字节 = GLOVE_PROTO_VERSION） */
#define GLOVE_FW_VER_MAJOR          1u
#define GLOVE_FW_VER_MINOR          0u

/*
 * 自检包载荷（2026-09-02 与 RV 侧统一极性）：
 *   P0 = 异常位图高 16 位，P1 = 低 16 位。
 *   **bit = 1 表示该项异常/缺席 —— 与数据帧心跳位图同一极性、同一布局**：
 *   bit0..19 关节、20..24 触觉（未实现，恒 1）、25 IMU、26 磁力计（未实现，恒 1）、
 *   bit27 = TYPE-C PD 高压缺失（VBUS 分压 ADC ≤ 阈值）、bit28..31 恒 0。
 *   （数据帧心跳的 bit27..31 维持保留恒 1，两包仅 27..31 的填充不同。）
 *   P2 = 特殊状态字：bit0 = 握手检测到从机（主机释放宣告后在窗口内
 *        看到线被从机拉低），其余保留
 */
#define GLOVE_ST_PD_BIT             27u

/* ==========================================================================
 * 9b. XVS 线上的握手信令（2026-09-02 第四次修订：帧数一致方案）
 * ==========================================================================
 *
 * 两手套之间只有一条线：PB10 经互连 Type-C 的 SBU 芯直连对侧 PB10，
 * 同一网络也接着相机的 XVS 输入（开漏线与）。IMX415 下降沿触发拍摄，
 * 释放（上升沿）不触发 —— 且握手期间相机根本还没启动（见下），
 * 线上信令绝不会产生假帧。
 *
 * 全流程（相机启动挪到握手之后，两边相机都从第一个下降沿开始拍 →
 * **帧数天然一致**；双方 CYCLE 也从同一个沿起算 = 1，同源同号）：
 *
 *   ① 主机（按键侧）：拉低宣告 GLOVE_XVS_CLAIM_MS → 释放 →
 *      在 GLOVE_XVS_HANDSHAKE_MS 窗口内看线有没有被从机拉低
 *      （低累计 ≥ GLOVE_XVS_SLAVE_ACK_MS 判"从机在"）；
 *   ② 从机：见线低 > GLOVE_XVS_DECLARE_MS 判从 → 等线回高 →
 *      **立刻拉低占住线**（= 应答，同时表示"我在准备相机"）→
 *      发 0xC301 请自己的 RV 启动相机 → RV 回 0xC101 →
 *      释放线并切输入捕获（至少拉 GLOVE_XVS_SLAVE_HOLD_MIN_MS，
 *      保证主机看得见）；
 *   ③ 主机：握手成功即发 0xC301 请自己的 RV 启动相机；
 *      等 ①线被从机释放 且 ②自己 RV 回 0xC101，两者齐后再延
 *      GLOVE_XVS_START_DELAY_MS（从机理论上先就绪，纯裕量）→
 *      放第一个 XVS 脉冲 = 双方 CYCLE 1，直接进入采集；
 *      握手窗口超时 = 单手模式：发 0xC301，RV 回 0xC101 即放号。
 *
 *   放号之后线上只允许 IMX415 规格的 60Hz 波形，任何信令禁止。
 *   停采/恢复不走线（0xC201/0xC101 事务包）；重新自检 0x5F01 才停号。
 */

#define GLOVE_XVS_DECLARE_MS        100u   /**< 低 >100ms = 对面主机在宣告       */
#define GLOVE_XVS_CLAIM_MS          300u   /**< 主机宣告脉宽（原 2s；相机启动已
                                                挪到握手后，只需覆盖从机判据）   */
#define GLOVE_XVS_HANDSHAKE_MS      500u   /**< 主机释放后等从机拉低的窗口，
                                                超时 = 单手模式                  */
#define GLOVE_XVS_SLAVE_ACK_MS      20u    /**< 窗口内低累计 ≥ 此值 = 从机在     */
#define GLOVE_XVS_SLAVE_HOLD_MIN_MS 100u   /**< 从机应答至少拉低这么久（防 RV
                                                回包过快导致主机漏检）           */
#define GLOVE_XVS_START_DELAY_MS    300u   /**< 双手：两条件齐后 → 放号的裕量
                                                延时（从机切捕获只需微秒级）     */
#define GLOVE_XVS_RELEASE_TIMEOUT_MS 5000u /**< 从机等主机宣告结束的超时
                                                （主机死拉 → 重新自检）          */

/* ==========================================================================
 * 10. 布局自检（编译期）
 * ========================================================================== */

#if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
_Static_assert(GLOVE_OFF_LENGTH       == GLOVE_OFF_MAGIC       + 1u,
               "MAGIC 占 1 字");
_Static_assert(GLOVE_OFF_CYCLE        == GLOVE_OFF_LENGTH      + 1u,
               "LENGTH 占 1 字");
_Static_assert(GLOVE_OFF_HEARTBEAT    == GLOVE_OFF_CYCLE       + 2u,
               "CYCLE 占 2 字");
_Static_assert(GLOVE_OFF_JOINT        == GLOVE_OFF_HEARTBEAT   + 2u,
               "HEARTBEAT 占 2 字");
_Static_assert(GLOVE_OFF_TACTILE_STAT == GLOVE_OFF_JOINT       + GLOVE_JOINT_CNT * 2u,
               "JOINT 占 40 字");
_Static_assert(GLOVE_OFF_TACTILE      == GLOVE_OFF_TACTILE_STAT + GLOVE_TACTILE_CNT,
               "TACTILE_STAT 占 5 字");
_Static_assert(GLOVE_OFF_IMU_RAW      == GLOVE_OFF_TACTILE
                                         + GLOVE_TACTILE_CNT * GLOVE_TACTILE_CELLS,
               "TACTILE 占 1280 字");
_Static_assert(GLOVE_OFF_MAG          == GLOVE_OFF_IMU_RAW     + 6u,
               "IMU_RAW 占 6 字");
_Static_assert(GLOVE_OFF_QUAT         == GLOVE_OFF_MAG         + 3u,
               "MAG 占 3 字");
_Static_assert(GLOVE_OFF_CRC          == GLOVE_OFF_QUAT        + 4u,
               "QUAT 占 4 字");
_Static_assert(GLOVE_FRAME_WORDS      == GLOVE_OFF_CRC         + 1u,
               "CRC 占 1 字，整帧 1345 字");
#if (GLOVE_SENSOR_MODE == 2)
_Static_assert(GLOVE_A2C_GROUPS * GLOVE_A2C_CH_PER_GRP == GLOVE_JOINT_CNT + 1u,
               "碳膜板 3x7=21 路：前 20 路进关节槽，第 21 路丢弃");
_Static_assert(GLOVE_BUS_JOINT_CNT(0u) + GLOVE_BUS_JOINT_CNT(1u) +
               GLOVE_BUS_JOINT_CNT(2u) == GLOVE_JOINT_CNT,
               "有效通道 7+7+6 = 20，恰好填满关节槽");
#elif (GLOVE_SENSOR_MODE == 1)
_Static_assert(GLOVE_RIG_CNT_BUS2 + GLOVE_RIG_CNT_BUS3 + GLOVE_RIG_CNT_BUS4
               == GLOVE_JOINT_CNT,
               "临时转接板三路节点数之和必须等于 20");
_Static_assert((GLOVE_RIG_CNT_BUS2 <= 7u) && (GLOVE_RIG_CNT_BUS3 <= 7u) &&
               (GLOVE_RIG_CNT_BUS4 <= 7u),
               "每路上限 7：MCP 硬件掩码窗口只放行 0x101~0x107");
#else
_Static_assert(GLOVE_CAN_BUS_CNT * GLOVE_JOINTS_PER_BUS == GLOVE_JOINT_CNT,
               "5 路 x 4 个 = 20 个关节，槽位映射才是紧凑无空洞的");
#endif
_Static_assert(GLOVE_JOINTS_MAX_PER_BUS <= GLOVE_JOINT_ID_STRIDE,
               "全局编号步长必须不小于每路节点数，否则编号会跨路撞车");
#endif

#endif /* GLOVE_PROTOCOL_H */
