/* ============================================================
 * pid.h —— 增量式 PID（输出限幅天然抗积分饱和）
 * 公式：Δu = Kp*(e-e1) + Ki*e*dt + Kd*d_f
 *       out = out + Δu（夹在 [outMin, outMax] 内）
 * 改进：微分项 d=(e-2e1+e2)/dt 对阶梯状数字误差会输出尖刺，
 *       加一阶低通 d_f = α·d + (1-α)·d_f 把尖刺抹平。
 * dt 用真实周期传入，以后改控制频率不用重调参数。
 * ============================================================ */
#pragma once
#include <Arduino.h>

class IncPID {
public:
  void begin(float kp, float ki, float kd, float outMin, float outMax,
             float dAlpha = 1.0f) {           // dAlpha=1 时等于不滤波
    kp_ = kp; ki_ = ki; kd_ = kd; dAlpha_ = dAlpha;
    outMin_ = outMin; outMax_ = outMax;
  }
  /* err：当前偏差；dt：控制周期（秒），如 0.005 */
  float update(float err, float dt) {
    float dRaw = (err - 2*e1_ + e2_) / dt;         // 原始微分（对噪声敏感）
    dFilt_ = dAlpha_ * dRaw + (1.0f - dAlpha_) * dFilt_;  // 一阶低通
    float dOut = kp_ * (err - e1_)                 // 比例：纠当前偏差
               + ki_ * err * dt                    // 积分：纠长期偏置
               + kd_ * dFilt_;                     // 微分（已滤波）：抑制趋势
    out_ += dOut;
    /* 输出限幅 = 抗积分饱和：增量式没有独立的积分项，
     * 历史累积都藏在 out_ 里，夹住 out_ 就等于夹住了积分。 */
    if (out_ > outMax_) out_ = outMax_;
    if (out_ < outMin_) out_ = outMin_;
    e2_ = e1_;                         // 历史误差滚动前移
    e1_ = err;
    return out_;
  }
  /* 切换状态 / 切换传感器时调用，清掉历史包袱防突跳 */
  void reset() { out_ = 0; e1_ = 0; e2_ = 0; dFilt_ = 0; }
private:
  float kp_ = 0, ki_ = 0, kd_ = 0, dAlpha_ = 1;
  float outMin_ = -1, outMax_ = 1;
  float e1_ = 0, e2_ = 0, out_ = 0, dFilt_ = 0;
};
