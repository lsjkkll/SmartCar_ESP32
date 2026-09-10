/* ============================================================
 * ultrasonic.h —— HC-SR04 非阻塞测距
 * 原理：TRIG 发 10us 高电平 -> 模块发 8 个超声脉冲 ->
 *       ECHO 输出一段高电平，宽度 = 声波往返时间
 * 我们用"变更中断"同时捕获 ECHO 的上升沿和下降沿，
 * 两个时刻之差就是往返时间。主循环完全不用等。
 * ============================================================ */
#pragma once
#include <Arduino.h>
#include "config.h"

// ---- 中断里使用的全局变量：volatile 表示随时会被中断改写 ----
volatile unsigned long usRise = 0;   // 上升沿时刻（微秒）
volatile float usDist = 999.0f;      // 最新距离(cm)，999 = 无回波/超时
volatile bool  usNew  = false;       // 有没有新数据待处理
volatile unsigned long usLastEcho = 0; // 最近一次有效回波时刻（掉线检测用）

/* 中断服务函数：ESP32 上必须加 IRAM_ATTR（放在 RAM 里），
 * 否则偶发程序崩溃重启，这是 ESP32 特有的坑！ */
void IRAM_ATTR echoISR() {
  if (digitalRead(PIN_ECHO)) {
    usRise = micros();                              // 上升沿：开始计时
  } else {
    unsigned long d = micros() - usRise;            // 下降沿：算脉宽
    // 38000us 约 6.5m，超过视为超时；0.01715 = 343m/s 声速折半
    usDist = (d >= 38000 || d < 100) ? 999.0f : d * 0.01715f;
    usNew = true;
    usLastEcho = millis();                          // 记录活动时间
  }
}

class Ultrasonic {
public:
  void begin() {
    pinMode(PIN_TRIG, OUTPUT);
    pinMode(PIN_ECHO, INPUT);
    digitalWrite(PIN_TRIG, LOW);
    attachInterrupt(digitalPinToInterrupt(PIN_ECHO), echoISR, CHANGE);
  }

  /* 每 5ms 控制拍里调一次，内部自动 60ms 节流。
   * 只负责"发枪"，从不等待结果。 */
  void task() {
    if (millis() - lastTrig_ >= 60) {        // 两次测量至少隔 60ms
      lastTrig_ = millis();                    // 防止前一次回波串扰
      digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(3);
      digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10);  // 10us 触发
      digitalWrite(PIN_TRIG, LOW);
    }
  }

  /* 最新距离（cm），999 表示无效 */
  float distance() const { return usDist; }

  /* 是否有障碍：最近 3 次里至少 2 次落在盲区外、阈值内才确认。
   * 双重保险防止单次噪声误触发绕障动作。 */
  bool obstacleAhead() {
    if (usNew) {                               // 中断送来了新数据
      usNew = false;
      hIdx_ = (hIdx_ + 1) % 3;
      hist_[hIdx_] = usDist;
    }
    uint8_t closeCnt = 0;
    for (uint8_t i = 0; i < 3; i++)
      if (hist_[i] > 2.0f && hist_[i] < OBSTACLE_CM) closeCnt++;
    return closeCnt >= 2;                      // 3 中 2 才算真障碍
  }

  /* 掉线检测：超过 US_STALE_MS 没有任何回波（含超时回波），
   * 说明 ECHO 线可能松了——此时禁止触发绕障，防止幽灵障碍 */
  bool online() const {
    return (millis() - usLastEcho) < US_STALE_MS;
  }

private:
  unsigned long lastTrig_ = 0;
  float hist_[3] = {999, 999, 999};
  uint8_t hIdx_ = 0;
};
