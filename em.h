/* ============================================================
 * em.h —— 电磁：毛刺剔除 + 卡尔曼 + 峰值归一化 + 差比和
 * 约定：L0、L1 装左侧，L2、L3 装右侧（左右别装反！）
 * ============================================================ */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "filters.h"

class EmSensor {
public:
  void begin() {
    analogReadResolution(12);                  // 0~4095
    for (uint8_t i = 0; i < EM_NUM; i++) {
      analogSetPinAttenuation(EM_PIN[i], ADC_11db);  // 满量程≈3.3V
      peak_[i] = 1.0f;                 // 归一化基准，防除零
      kal_[i]  = Kalman1D(0.02, 6.0);  // 每路一个卡尔曼
      clipCnt_[i] = 0;
    }
  }

  /* ---------- 每周期调用：读数 + 两级净化 ---------- */
  void readAll() {
    for (uint8_t i = 0; i < EM_NUM; i++) {
      float raw = (float)analogRead(EM_PIN[i]);
      /* 第一级：物理限幅剔毛刺。
       * 电机大电流会打出 4090+ 或个位数的瞬时野值，
       * 超出合理范围直接丢弃，用上次读数顶替。
       * 同时统计连续削顶次数：连续 200 拍(1秒)都在削顶，
       * 说明不是毛刺而是断线/短路，置故障标志。 */
      if (raw > 4050.0f || raw < 3.0f) {
        raw = lastRaw_[i];
        if (clipCnt_[i] < EM_CLIP_CNT) clipCnt_[i]++;
        if (clipCnt_[i] >= EM_CLIP_CNT) fault_ = true;
      } else if (clipCnt_[i] > 0) clipCnt_[i]--;
      lastRaw_[i] = raw;
      /* 第二级：卡尔曼平滑，压掉电机换向的高频抖动 */
      float v = kal_[i].update(raw);
      /* 峰值归一化基准：见新高就抬升（只升不降） */
      if (v > peak_[i]) peak_[i] = v;
      val_[i] = v;
    }
    /* 每 500ms 让基准整体衰减 1%：防止基准永远卡死在
     * 历史最高点（比如某次贴线特别近），导致归一化吃不满 */
    if (millis() - lastDecay_ > 500) {
      lastDecay_ = millis();
      for (uint8_t i = 0; i < EM_NUM; i++) peak_[i] *= 0.99f;
    }
  }

  /* ---------- 信号总强度：判断是否进入电磁段 ---------- */
  float strength() {
    return (val_[0] + val_[1] + val_[2] + val_[3]) / 4.0f;
  }

  /* ---------- 差比和求偏差：-1.0 ~ +1.0 ----------
   * 先把每路除以自己的历史峰值（消除电感个体差异），
   * 左两路相加 vs 右两路相加，再套差比和公式。
   * 正 = 线在车右侧，与灰度误差方向约定一致。 */
  float getError() {
    float l = val_[0] / peak_[0] + val_[1] / peak_[1];
    float r = val_[2] / peak_[2] + val_[3] / peak_[3];
    float sum = l + r;
    if (sum < 0.05f) return lastErr_;  // 信号太弱：记忆上次误差
    lastErr_ = (r - l) / sum;          // 差比和核心公式
    return lastErr_;
  }

  float getVal(uint8_t i) const { return val_[i]; }  // 调试用
  bool  fault() const { return fault_; }             // 疑似掉线/短路

private:
  Kalman1D kal_[EM_NUM];
  float val_[EM_NUM]     = {0};
  float lastRaw_[EM_NUM] = {0};
  float peak_[EM_NUM]    = {1};
  uint16_t clipCnt_[EM_NUM] = {0};
  bool  fault_ = false;
  float lastErr_ = 0;
  unsigned long lastDecay_ = 0;
};
