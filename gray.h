/* ============================================================
 * gray.h —— 亚博八路灰度模块（数字接口版）：CD4051 选通读取
 * 原理：模块 8 个探头各带比较器，压线输出固定电平；
 *       单片机用 AD0/AD1/AD2 三根线轮流选通道，从 OUT 读 0/1。
 * 优点：不怕环境光（比较器阈值已调好在板上），无需软件校准；
 * 代价：只有 0/1 两种状态，误差是阶梯状的（PID 微分已加滤波）。
 * 约定：误差 -1.0 ~ +1.0，负=线在左该左转，正=线在右该右转
 * ============================================================ */
#pragma once
#include <Arduino.h>
#include "config.h"

class GraySensor {
public:
  void begin() {
    pinMode(PIN_GRAY_AD0, OUTPUT);
    pinMode(PIN_GRAY_AD1, OUTPUT);
    pinMode(PIN_GRAY_AD2, OUTPUT);
    pinMode(PIN_GRAY_OUT, INPUT);
  }

  /* ---------- 每个控制周期调用：8 通道轮询 ----------
   * 官方示例选通后等 50us，实测 CD4051+比较器 30us 足够稳定，
   * 一轮 8×30us ≈ 0.24ms，占 5ms 节拍的 5%。 */
  void readAll() {
    uint8_t pattern = 0;
    for (uint8_t i = 0; i < GRAY_NUM; i++) {
      select_(i);
      delayMicroseconds(GRAY_SETTLE_US);
      if (digitalRead(PIN_GRAY_OUT) == GRAY_LINE_LEVEL)
        pattern |= (1 << i);                    // bit i = 第 i 路压线
    }
    lastPattern_ = pattern;

    /* 掉线检测：图样完全不变累计计时（行驶中探头不可能一直不变） */
    if (pattern != prevPattern_) { prevPattern_ = pattern; sameSince_ = millis(); }
    stuck_ = (millis() - sameSince_ > GRAY_STUCK_MS);

    onCnt_ = 0;
    for (uint8_t i = 0; i < GRAY_NUM; i++) if (pattern & (1 << i)) onCnt_++;
  }

  /* ---------- 加权平均求偏差：-1.0 ~ +1.0 ----------
   * 8 路等间距权重；丢线时记忆误差缓慢衰减，防止钉死在弯道上 */
  float getError() {
    static const float W[GRAY_NUM] =
      {-1.0f, -0.72f, -0.44f, -0.16f, 0.16f, 0.44f, 0.72f, 1.0f};
    if (onCnt_ == 0) { lastErr_ *= 0.95f; return lastErr_; }
    float s = 0;
    for (uint8_t i = 0; i < GRAY_NUM; i++)
      if (lastPattern_ & (1 << i)) s += W[i];
    lastErr_ = s / onCnt_;
    return lastErr_;
  }

  /* ---------- 元素特征检测（状态机靠它们做判断） ---------- */
  bool allWhite() { return onCnt_ >= 7; }        // ≥7 路压线：十字/长白线/环岛口
  bool allLost()  { return onCnt_ == 0; }        // 全丢：盲道/虚线断口
  bool hardLeft() {                              // 左直角：左半区亮 右半区灭
    uint8_t l = 0, r = 0;
    for (uint8_t i = 0; i < 4; i++) { if (lastPattern_ & (1 << i)) l++; }
    for (uint8_t i = 4; i < 8; i++) { if (lastPattern_ & (1 << i)) r++; }
    return l >= 3 && r == 0;
  }
  bool hardRight() {                             // 右直角：右半区亮 左半区灭
    uint8_t l = 0, r = 0;
    for (uint8_t i = 0; i < 4; i++) { if (lastPattern_ & (1 << i)) l++; }
    for (uint8_t i = 4; i < 8; i++) { if (lastPattern_ & (1 << i)) r++; }
    return r >= 3 && l == 0;
  }

  /* 中间两路压线：直角闭环出弯的"已回正"判据 */
  bool centerOn() { return (lastPattern_ & 0x18) != 0; }   // bit3|bit4

  /* "两组分离亮灯"检测：岔路口主线+支线同时可见的特征
   * 数亮灯块的个数，≥2 块说明同时看到两条线 */
  uint8_t groupCount() {
    uint8_t groups = 0;
    bool inGroup = false;
    for (uint8_t i = 0; i < GRAY_NUM; i++) {
      bool on = (lastPattern_ & (1 << i)) != 0;
      if (on && !inGroup) { groups++; inGroup = true; }
      if (!on) inGroup = false;
    }
    return groups;
  }

  uint8_t pattern() const { return lastPattern_; } // 调试用（串口打二进制）
  uint8_t onCount() const { return onCnt_; }
  bool    stuck()   const { return stuck_; }       // 疑似掉线

  /* ---------- 发车前自检（ST_CHECK 用） ----------
   * 统计每一路是否既亮过又灭过——手拿障碍物在每个探头前晃一遍，
   * 5 秒内没翻动过的路就是死路（断线/探头坏了） */
  void resetSeen() { seenOn_ = 0; seenOff_ = 0; }
  void accumSeen() {
    seenOn_  |= lastPattern_;
    seenOff_ |= (~lastPattern_) & 0xFF;
  }
  uint8_t seenCount() const {                    // "活"的路数 0~8
    uint8_t alive = seenOn_ & seenOff_, cnt = 0;
    for (uint8_t i = 0; i < GRAY_NUM; i++) if (alive & (1 << i)) cnt++;
    return cnt;
  }
  uint8_t deadMask() const { return (~(seenOn_ & seenOff_)) & 0xFF; }

  int getNorm(uint8_t i) const {                   // 兼容旧调试代码
    return (lastPattern_ & (1 << i)) ? 1000 : 0;
  }

private:
  void select_(uint8_t ch) {                       // CD4051 通道地址
    digitalWrite(PIN_GRAY_AD0, ch & 0x01);
    digitalWrite(PIN_GRAY_AD1, (ch >> 1) & 0x01);
    digitalWrite(PIN_GRAY_AD2, (ch >> 2) & 0x01);
  }
  uint8_t lastPattern_ = 0, prevPattern_ = 0;
  uint8_t onCnt_ = 0;
  uint8_t seenOn_ = 0, seenOff_ = 0;
  float   lastErr_ = 0;
  bool    stuck_ = false;
  unsigned long sameSince_ = 0;
};
