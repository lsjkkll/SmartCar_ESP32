/* ============================================================
 * motor.h —— TB6612FNG 双电机驱动（兼容 Core 2.x / 3.x）
 * 接线：PWMA=9, AIN1=10, AIN2=11（左电机组，4WD 同侧两电机并联）
 *       PWMB=12, BIN1=13, BIN2=14（右电机组）
 *       STBY 接 3.3V，VM 接电池 7.4V，VCC 接 3.3V，GND 共地
 * 注意：TT 马达堵转电流约 1.3A，同侧两台并联后翻倍，
 *       TB6612 单通道持续上限 1.2A——原地掉头别超过 1 秒！
 * ============================================================ */
#pragma once
#include <Arduino.h>
#include "config.h"

class MotorDriver {
public:
  void begin() {
    pinMode(PIN_AIN1, OUTPUT); pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_BIN1, OUTPUT); pinMode(PIN_BIN2, OUTPUT);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    // ---- Core 3.x：直接把 PWM 绑定到引脚 ----
    ledcAttach(PIN_PWMA, 2000, 8);     // 引脚, 频率2000Hz, 8位分辨率
    ledcAttach(PIN_PWMB, 2000, 8);
#else
    // ---- Core 2.x：先创建通道再绑引脚 ----
    ledcSetup(0, 2000, 8);             // 通道0, 2000Hz, 8位
    ledcSetup(1, 2000, 8);             // 通道1
    ledcAttachPin(PIN_PWMA, 0);
    ledcAttachPin(PIN_PWMB, 1);
#endif
    setWheels(0, 0);                   // 上电先停车
  }

  /* 直接设置左右轮：-255 ~ +255，正数前进 */
  void setWheels(int left, int right) {
    driveOne(PIN_AIN1, PIN_AIN2, PIN_PWMA, 0, left);
    driveOne(PIN_BIN1, PIN_BIN2, PIN_PWMB, 1, right);
  }

private:
  void driveOne(uint8_t in1, uint8_t in2, uint8_t pwmPin,
                uint8_t ch, int spd) {
    bool fwd = (spd >= 0);
    digitalWrite(in1, fwd ? HIGH : LOW);   // 方向脚决定正反转
    digitalWrite(in2, fwd ? LOW : HIGH);
    uint32_t duty = (uint32_t)constrain(abs(spd), 0, 255);
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    ledcWrite(pwmPin, duty);               // 3.x 按引脚写占空比
#else
    ledcWrite(ch, duty);                   // 2.x 按通道写占空比
#endif
  }
};
