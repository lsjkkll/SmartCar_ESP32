/* ============================================================
 * filters.h —— 两个开箱即用的滤波器
 * ============================================================ */
#pragma once
#include <Arduino.h>

/* ---------- 滑动平均滤波：最简单最实用，灰度就用它 ----------
 * 原理：保存最近 N 次读数，返回平均值。偶尔一次毛刺
 * 会被 N-1 个正常值"稀释"，输出立刻平滑下来。
 */
class MovAvg {
public:
  explicit MovAvg(uint8_t win = 8) : win_(win) {}   // win=窗口长度，8 或 16
  float update(float v) {
    buf_[idx_] = v;                    // 新值覆盖最老的值（环形队列）
    idx_ = (idx_ + 1) % win_;
    if (cnt_ < win_) cnt_++;
    float s = 0;
    for (uint8_t i = 0; i < cnt_; i++) s += buf_[i];
    return s / cnt_;                   // 返回平均值
  }
private:
  float buf_[16] = {0};
  uint8_t idx_ = 0, cnt_ = 0, win_;
};

/* ---------- 一维简化卡尔曼：响应比滑动平均快，电感用它 ----------
 * 原理（不用怕，就三步）：
 *   1. 预测：不确定度 p 变大一点（过程噪声 q）
 *   2. 增益：k = p / (p + r)，观测噪声 r 越大越不信新读数
 *   3. 修正：估计值 x 向新读数 z 靠拢 k 比例
 * q 调大→更跟手但降噪弱；r 调大→更平滑但滞后。
 */
class Kalman1D {
public:
  Kalman1D(float q = 0.02, float r = 6.0) : q_(q), r_(r) {}
  float update(float z) {
    if (!init_) { x_ = z; init_ = true; return x_; }  // 第一次直接采用
    p_ += q_;                          // 预测
    float k = p_ / (p_ + r_);          // 卡尔曼增益
    x_ += k * (z - x_);                // 修正
    p_ *= (1 - k);                     // 修正后更自信
    return x_;
  }
private:
  float q_, r_, x_ = 0, p_ = 1;
  bool init_ = false;
};
