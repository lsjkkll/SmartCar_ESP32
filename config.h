/* ============================================================
 * config.h —— 全部引脚与可调参数（调参就改这个文件！）
 * 华东交通大学双基智能车 · ESP32-S3-N8R8 + Arduino IDE
 *
 * ★ 本版适配 ESP32-S3：S3 的 ADC 在 GPIO1~20，26~37 被 Flash/PSRAM
 *   占用，19/20 是 USB，43/44 是串口——引脚表与经典 ESP32 完全不同！
 * ============================================================ */
#pragma once
#include <Arduino.h>

/* ---------- 八路灰度模块（亚博 YB-MVX05，数字接口） ----------
 * 模块内部 CD4051 模拟开关分时复用：AD0/AD1/AD2 选通道，OUT 读 0/1。
 * 模块 5V 供电（OUT 高电平≈5V，经厂家板上分压/限流可直读，
 * 不放心可在 OUT 串 1k 电阻）。安装高度 18mm 最佳。 */
#define GRAY_NUM 8                              // 8 路全用
const uint8_t PIN_GRAY_AD0 = 6;                 // 通道地址 bit0
const uint8_t PIN_GRAY_AD1 = 7;                 // 通道地址 bit1
const uint8_t PIN_GRAY_AD2 = 15;                // 通道地址 bit2
const uint8_t PIN_GRAY_OUT = 16;                // 数字输出
#define GRAY_LINE_LEVEL 1                       // 压线时 OUT 电平：1=高 0=低
                                                // （厂家默认线上灯亮=高电平）
const uint8_t GRAY_SETTLE_US = 30;              // 选通后等待(us)，官方示例 50

/* ---------- 电磁电感：4 路模拟，接 ADC1（不开 WiFi，ADC2 也能用） ---------- */
#define EM_NUM 4
const uint8_t EM_PIN[EM_NUM] = {1, 2, 4, 5};    // 左到右：L0 L1 | L2 L3

/* ---------- 电池电压监测（可选）：100k+100k 分压后接 GPIO8 ----------
 * 不接分压就把 VBAT_EN 设 0，相关补偿自动关闭 */
#define VBAT_EN 0
const uint8_t PIN_VBAT = 8;
const float   VBAT_REF = 7.6f;                  // 标定此电压下的车速为基准
const float   VBAT_LOW = 6.6f;                  // 低于此值蜂鸣报警（2S 过放保护）

/* ---------- 超声波 HC-SR04（ECHO 必须 1k/2k 分压进 3.3V！） ---------- */
const uint8_t PIN_TRIG = 17;
const uint8_t PIN_ECHO = 18;
const float   OBSTACLE_CM = 18.0f;              // 近于此值判障碍

/* ---------- 电机驱动 TB6612（STBY 接 3.3V！4WD：同侧两电机并联） ---------- */
const uint8_t PIN_PWMA = 9,  PIN_AIN1 = 10, PIN_AIN2 = 11; // 左电机组
const uint8_t PIN_PWMB = 12, PIN_BIN1 = 13, PIN_BIN2 = 14; // 右电机组

/* ---------- 其他外设 ---------- */
const uint8_t PIN_BUZZER = 38;                  // KY-006 无源蜂鸣器（tone 驱动）
const uint8_t PIN_BTN    = 21;                  // 按键（另一端接 GND）
const uint16_t BUZZ_FREQ = 2700;                // 无源蜂鸣器谐振频率(Hz)

/* ---------- 控制周期 ---------- */
const uint16_t CTRL_PERIOD_MS = 5;              // 5ms 一拍 = 200Hz
const float    CTRL_PERIOD_S  = 0.005f;

/* ---------- PID 初值（调参顺序：先 P 后 D 最后 I，详见指南） ----------
 * PID_D_ALPHA：微分项一阶低通系数(0~1)，数字灰度误差是阶梯状的，
 * 微分不滤波会输出尖刺；0.3 ≈ 截止频率 24Hz @200Hz 控制频率 */
const float PID_KP = 0.8f, PID_KI = 0.05f, PID_KD = 1.2f;
const float PID_D_ALPHA = 0.3f;
const int   TURN_MAX = 95;                      // 转向量最大差速幅度

/* 速度自适应转向：转向差速随车速反比缩放（车速越快打角越柔，防甩尾）
 * SPEED_REF 是调参时的基准车速（PWM 值） */
#define ADAPT_TURN 1
const int SPEED_REF = 90;

/* ---------- 速度（PWM 0~255） ---------- */
const uint8_t SPEED_CRUISE = 120;               // 光电直道
const uint8_t SPEED_SLOW   = 90;                // 元素区
const uint8_t SPEED_EM     = 85;                // 电磁段（跷跷板安全）
const uint8_t SPEED_GARAGE = 80;                // 出库/入库

/* ---------- 光电/电磁切换阈值（滞回 + 确认计数防抖） ---------- */
const float    EM_ENTER_TH = 60.0f;             // 强于此 → 进电磁段
const float    EM_EXIT_TH  = 30.0f;             // 弱于此 → 回光电段
const uint16_t BLEND_MS    = 600;               // 误差混合过渡时长

/* ---------- 元素时序参数（比赛现场重点调这些） ----------
 * 时长 = 元素尺寸 / 车速。改了速度这些都要重算！
 * 假设车速约 0.45m/s（SPEED_SLOW 档） */
const uint16_t CROSS_MS        = 100;  // 全白短于此 = 十字（2cm 线宽约 45ms）
const uint16_t LONGWHITE_MS    = 120;  // 全白长于此 = 长白线/环岛口/岔路口
const uint16_t GARAGE_IGNORE_MS= 2500; // 出库后多久内忽略长全白（车库旁长白线）
const uint16_t LOST_DASH_MS    = 260;  // 丢线长于此 = 盲道（虚线断口约 220ms）
const uint16_t BLIND_MS        = 700;  // 盲道直行时长（30cm 约 660ms）
const uint16_t HOLD_AFTER_CROSS_MS = 120; // 过十字后直行保持，防甩尾

/* 直角过弯：最小开环时长 + 传感器闭环提前出弯 + 最长兜底 + 速度补偿
 * 时长按 SPEED_REF 标定，运行时自动乘以 SPEED_REF/当前车速 */
const uint16_t HARDTURN_MS     = 350;  // 标定车速下的最长开环时长
const uint16_t HARDTURN_MIN_MS = 120;  // 最小开环时长（此前不允许提前出弯）

/* 六边形环岛（每段 35cm/内角120°，规则） */
const int8_t   ISLAND_DIR   = -1;      // 进环打角方向：-1 左 +1 右（现场定）
const uint16_t ISLAND_ENTRY_MS = 300;  // 进环开环时长
const uint16_t ISLAND_MIN_MS = 4000;   // 环内至少待这么久才允许出环
const uint16_t ISLAND_EXIT_MS = 500;   // 出环直行时长

/* 岔路（保底策略：拐进去掉个头，不吃 300 秒重罚）
 * T 口 + 50cm 直道 + 六边形 + 末端 50cm 正方形（规则） */
const int8_t   FORK_DIR     = 1;       // +1 右转进岔道 / -1 左
const uint16_t FORK_TRACK_MS = 1400;   // 进岔道后循迹时长
const uint16_t FORK_SPIN_MS  = 700;    // 原地掉头时长
const uint16_t FORK_SPIN_PWM = 110;    // 掉头轮速
const uint16_t FORK_EXIT_MS  = 500;    // 穿回 T 口直行时长

/* 障碍绕行（方向要避开内角锥桶！规则：锥桶距线≥15cm 设在内角） */
const int8_t   OB_AVOID_DIR = 1;       // +1 向右绕 / -1 向左
const uint16_t OB_ARC_MS    = 500;     // 绕障开环弧线时长

/* 电磁圆环（半径 25cm，规则） */
const uint16_t RING_TURN_MS = 400;     // 满幅打角持续此时长 = 进圆环
const uint16_t RING_LAP_MS  = 4200;    // 绕一圈时长（2πR/v≈1.57m/车速，现场标定！）
const uint16_t RING_EXIT_MS = 600;     // 环口直行冲出时长

/* ---------- 环岛/岔路特征区分（代替纯硬编码顺序表） ----------
 * FEATURE_DETECT=1：长全白前 250ms 内分析传感器特征：
 *   - 出现"两组分离亮灯"（中间主线+侧边支线同时亮）→ 岔路
 *   - 误差单调大幅扫向一侧 → 环岛
 *   置信度不足时回退 LONG_WHITE_PLAN 顺序表（双保险）
 * FEATURE_DETECT=0：纯顺序表（保守，比赛前若特征法不稳就关掉） */
#define FEATURE_DETECT 1
const uint8_t  FEAT_HIST_MS  = 250;    // 特征分析回溯窗口
const float    FEAT_SWEEP_TH = 0.55f;  // 误差扫动幅度阈值（判环岛）

/* ---------- 长全白事件兜底顺序表：现场按实际赛道顺序改！ ----------
 * 0=按环岛处理  1=按岔路处理  2=直接直行放弃该元素 */
#define PLAN_ISLAND 0
#define PLAN_FORK   1
#define PLAN_IGNORE 2
const uint8_t LONG_WHITE_PLAN[] = {PLAN_ISLAND, PLAN_FORK, PLAN_IGNORE};
const uint8_t LW_PLAN_NUM = 3;

/* ---------- 鲁棒性 ---------- */
const uint16_t GRAY_STUCK_MS  = 2000;  // 灰度图样卡死此时长 → 判掉线
const uint16_t EM_CLIP_CNT    = 200;   // 电感连续削顶/贴地次数 → 判掉线
const uint16_t US_STALE_MS    = 1000;  // 超声波无新回波时长 → 判掉线

/* ---------- 调试开关：1 开 0 关（同时只开一个） ---------- */
#define DEBUG_PLOT 1     // 串口绘图器模式（看曲线）
#define DEBUG_TEXT 0     // 文本模式（看数值）
