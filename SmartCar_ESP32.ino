/* ============================================================
 * SmartCar_ESP32.ino —— 主程序：状态机 + 5ms 控制循环
 * 华东交通大学双基智能车 · ESP32-S3-N8R8 + Arduino IDE
 *
 * 使用方法：
 *   1. 本文件夹 8 个文件放在同一目录，用 Arduino IDE 打开本文件
 *   2. 开发板选 "ESP32S3 Dev Module"，波特率 115200
 *   3. 上电 -> 蜂鸣 1 声就绪（数字灰度模块无需校准！）
 *   4. 短按按键 -> 静止 2.5 秒（规则要求）-> 自动发车
 *   5. 长按按键 1.5 秒 -> 进入自检：5 秒内用手在每个灰度探头前晃过，
 *      蜂鸣 N 声 = N 路活着（8 声全好），串口可看电磁/超声波状态
 * ============================================================ */
#include <Arduino.h>
#include "config.h"
#include "filters.h"
#include "motor.h"
#include "ultrasonic.h"
#include "gray.h"
#include "em.h"
#include "pid.h"

/* ==================== 状态机定义 ==================== */
enum State {
  ST_IDLE,        // 待命：短按发车 / 长按自检
  ST_CHECK,       // 传感器自检（5 秒）
  ST_WAIT_RUN,    // 静止 2.5 秒（规则要求）
  ST_GARAGE_OUT,  // 出车库直行
  ST_TRACK,       // 光电循迹
  ST_EM,          // 电磁循迹
  ST_OBSTACLE,    // 超声波绕障
  ST_FINISH,      // 终点刹车入库
  ST_STOP         // 停车
};

/* ==================== 全局对象 ==================== */
MotorDriver motor;
Ultrasonic  us;
GraySensor  gray;
EmSensor    em;
IncPID      pid;

State st = ST_IDLE;               // 当前状态
unsigned long lastTickUs = 0;     // 控制节拍（微秒级，抖动 <1ms）
unsigned long stEnter_   = 0;     // 当前状态进入时刻
bool  passedEM = false;           // 已跑完电磁段（终点判定开关）
float curErr   = 0, curTurn = 0;  // 当前误差与 PID 输出

/* ==================== 元素子状态变量 ==================== */
unsigned long whiteStart_ = 0;    // 全白事件开始时刻
unsigned long lostStart_  = 0;    // 全丢线开始时刻
unsigned long holdStraightUntil_ = 0; // 十字后直行保持截止
unsigned long blindUntil_  = 0;   // 盲道直行截止
unsigned long openLoopUntil_ = 0; // 开环动作截止
unsigned long openLoopStart_ = 0; // 开环动作起点（闭环提前出弯用）
bool  olExitOnLine_ = false;      // 开环动作允许"回到线"提前结束
int   olLeft_ = 0, olRight_ = 0;  // 开环目标轮速
uint8_t emEnterCnt_ = 0, emLostCnt_ = 0; // 光电/电磁切换确认计数
bool  inIsland_ = false;          // 六边形环岛内
unsigned long islandEnter_ = 0;
bool  inFork_ = false;            // 岔路处理中
unsigned long forkEnter_ = 0;
uint8_t obPhase_ = 0;             // 绕障阶段
unsigned long lastHardTurn_ = 0;  // 直角防重复
uint8_t hlCnt_ = 0, hrCnt_ = 0;   // 直角特征确认计数
uint8_t lwEventIdx_ = 0;          // 长全白事件序号（兜底顺序表用）
unsigned long garageTime_ = 0;    // 出库完成时刻
unsigned long blendStart_ = 0;    // 光电/电磁混合渐变起点
unsigned long ringTurnSince_ = 0; // 电磁圆环：满幅打角起点
unsigned long ringEnterAt_  = 0;  // 电磁圆环：确认进环时刻
unsigned long ringCooldown_ = 0;  // 电磁圆环：出环后冷却
uint16_t btnCnt_ = 0;             // 按键按下持续计数
unsigned long checkStart_ = 0;    // 自检开始时刻

/* ---------- 环岛/岔路特征区分：误差与灯组数环形历史 ---------- */
#define HIST_LEN 64               // 64 拍 = 320ms 回溯窗口
float   errHist_[HIST_LEN] = {0};
uint8_t grpHist_[HIST_LEN] = {0};
uint8_t histIdx_ = 0;

/* ---------- 电池电压监测（可选，config.h 里 VBAT_EN 打开） ---------- */
#if VBAT_EN
MovAvg vbatFilt(8);
float vbatComp_ = 1.0f;           // 车速补偿系数
unsigned long lastVbat_ = 0;
#endif

/* ==================== 小工具函数 ==================== */
/* 蜂鸣 n 声：KY-006 是无源蜂鸣器，必须用 tone() 给方波才会响！
 * （仅低速/静止状态使用，内部 delay 是可以的） */
void beep(int n) {
  for (int i = 0; i < n; i++) {
    tone(PIN_BUZZER, BUZZ_FREQ); delay(120);
    noTone(PIN_BUZZER);
    if (i < n - 1) delay(120);
  }
}

/* 按车速缩放的开环时长：开环位移 = 车速 × 时间，
 * 电池电压下降/车速档不同时，时间反比补偿，转过的角度才一致 */
uint16_t speedScaleMs(uint16_t baseMs, int curBase) {
  if (curBase < 1) curBase = 1;
  return (uint16_t)((uint32_t)baseMs * SPEED_REF / curBase);
}

/* 差速混合：base 基础速度，turn 转向量(-1~1, 正=右转)
 * ADAPT_TURN：车速越快转向差速越柔（高速大差速=甩尾冲出） */
void driveByTurn(int base, float turn) {
#if VBAT_EN
  base = (int)(base * vbatComp_);               // 电池补偿：电压低则补 PWM
#endif
#if ADAPT_TURN
  int tmax = TURN_MAX * SPEED_REF / (base > 1 ? base : 1);
  tmax = constrain(tmax, 40, TURN_MAX);
#else
  int tmax = TURN_MAX;
#endif
  int delta = (int)(turn * tmax);
  motor.setWheels(constrain(base + delta, -255, 255),
                  constrain(base - delta, -255, 255));
}

/* 开环动作：以固定轮速跑 ms 毫秒（期间屏蔽循迹）
 * exitOnLine=true 时：过了最短时长后，中间探头回线即提前结束
 * （直角过弯的核心改进：不再赌固定时长，而是"转到看见线为止"） */
void openLoop(int l, int r, uint16_t ms, bool exitOnLine = false) {
  olLeft_ = l; olRight_ = r;
  motor.setWheels(l, r);
  openLoopStart_ = millis();
  openLoopUntil_ = openLoopStart_ + ms;
  olExitOnLine_  = exitOnLine;
}

/* 切换状态：统一计时与 PID 复位，防止旧积分带偏新车道 */
void enter(State s) {
  st = s;
  stEnter_ = millis();
  pid.reset();
  whiteStart_ = lostStart_ = 0;
  hlCnt_ = hrCnt_ = 0;
  obPhase_ = 0;                   // ★ 修复：绕障阶段必须清零，否则第二次遇障直接跳到找线阶段
  blendStart_ = stEnter_;         // 新状态前 600ms 做误差渐变
}

/* 障碍判定：超声波掉线（ECHO 线松了）时一律报"无障碍"，
 * 防止幽灵障碍把车逼停 */
bool obstacleNow() {
  return us.online() && us.obstacleAhead();
}

/* ---------- 环岛/岔路特征分类器 ----------
 * 分析长全白之前 FEAT_HIST_MS 毫秒内的传感器行为：
 *   岔路（T 型口）：主线+支线同时可见 → 历史上出现过"两组分离亮灯"
 *   环岛：线单调地向一侧扫过去 → 误差窗口内单向大幅滑动
 * 返回 PLAN_FORK / PLAN_ISLAND / 0xFF（不确定→用兜底顺序表） */
uint8_t classifyLongWhite() {
#if FEATURE_DETECT
  bool sawTwoGroups = false;
  float eMin = 1.0f, eMax = -1.0f;
  int n = FEAT_HIST_MS / CTRL_PERIOD_MS;
  if (n > HIST_LEN) n = HIST_LEN;
  for (int i = 1; i <= n; i++) {
    uint8_t idx = (histIdx_ + HIST_LEN - i) % HIST_LEN;
    if (grpHist_[idx] >= 2) sawTwoGroups = true;
    if (errHist_[idx] < eMin) eMin = errHist_[idx];
    if (errHist_[idx] > eMax) eMax = errHist_[idx];
  }
  if (sawTwoGroups) return PLAN_FORK;             // 岔路特征明确
  if (eMax - eMin > FEAT_SWEEP_TH) return PLAN_ISLAND; // 单向大扫动=环岛
#endif
  return 0xFF;                                    // 不确定：交给顺序表
}

/* ==================== 各状态处理函数 ==================== */
/* --- 待命：短按发车，长按 1.5s 进自检 --- */
void doIdle() {
  motor.setWheels(0, 0);
  if (digitalRead(PIN_BTN) == LOW) {
    btnCnt_++;
  } else {
    if (btnCnt_ >= 300) enter(ST_CHECK);          // 长按 ≥1.5s：自检
    else if (btnCnt_ >= 5) enter(ST_WAIT_RUN);    // 短按：发车
    btnCnt_ = 0;
  }
}

/* --- 自检：5 秒内用手在每个探头前晃，蜂鸣声数 = 活的灰度路数 --- */
void doCheck() {
  motor.setWheels(0, 0);
  if (checkStart_ == 0) {
    checkStart_ = millis();
    beep(1);
    gray.resetSeen();
  }
  gray.accumSeen();
  if (millis() - checkStart_ >= 5000) {
    uint8_t alive = gray.seenCount();
    beep(alive);                                  // 8 声 = 全好
    Serial.printf("[CHECK] 灰度存活 %d/8，死路掩码 0x%02X，电磁强度 %.0f，超声波 %s\n",
                  alive, gray.deadMask(), em.strength(),
                  us.online() ? "在线" : "掉线！检查 ECHO 分压");
    checkStart_ = 0;
    enter(ST_IDLE);
  }
}

/* --- 静止等待：满足规则"静止至少2秒后自动启动" --- */
void doWaitRun() {
  motor.setWheels(0, 0);
  if (millis() - stEnter_ >= 2500) {
    beep(1);                                      // 出发提示音
    enter(ST_GARAGE_OUT);
  }
}

/* --- 出库：低速直行，越过库线与长白线，直到捕获白线 --- */
void doGarageOut() {
  motor.setWheels(SPEED_GARAGE, SPEED_GARAGE);
  if (gray.centerOn() || millis() - stEnter_ > 1500) {  // 1.5s 兜底
    garageTime_ = millis();       // 记录出库时刻（长白线免疫窗起点）
    enter(ST_TRACK);
  }
}

/* --- 光电循迹 + 全部光电元素（核心函数） --- */
void doTrack() {
  unsigned long now = millis();

  /* 每拍记录误差与灯组数历史（环岛/岔路特征区分的数据源） */
  float eRaw = gray.getError();
  errHist_[histIdx_] = eRaw;
  grpHist_[histIdx_] = gray.groupCount();
  histIdx_ = (histIdx_ + 1) % HIST_LEN;

  /* 0) 开环动作进行中：保持轮速直到到期
   *    打断条件只有两个：障碍（最高优先级）、
   *    直角类动作过了最短时长后中间探头回线（闭环提前出弯） */
  if (now < openLoopUntil_) {
    if (obstacleNow()) { enter(ST_OBSTACLE); return; }
    bool earlyExit = olExitOnLine_ &&
                     now - openLoopStart_ >= HARDTURN_MIN_MS &&
                     gray.centerOn();
    if (!earlyExit) { motor.setWheels(olLeft_, olRight_); return; }
    openLoopUntil_ = 0;                           // 提前出弯
    pid.reset();                                  // 清微分历史，防交接突跳
  }

  /* 1) 障碍最高优先级 */
  if (obstacleNow()) { enter(ST_OBSTACLE); return; }

  /* 2) 电磁段到来：连续6拍强信号确认（防单次干扰误切） */
  if (em.strength() > EM_ENTER_TH) {
    if (++emEnterCnt_ >= 6) { enter(ST_EM); return; }
  } else emEnterCnt_ = 0;

  /* 3) 全白事件：十字 / 长白线 / 环岛口 / 岔路口 */
  if (gray.allWhite()) {
    if (whiteStart_ == 0) whiteStart_ = now;
    unsigned long dur = now - whiteStart_;
    if (dur > LONGWHITE_MS) {                     // 长全白
      if (passedEM && !inIsland_ && !inFork_) {    // 终点长白线！
        enter(ST_FINISH);
        return;
      }
      if (inIsland_) {                            // 环岛内遇长全白
        if (now - islandEnter_ > ISLAND_MIN_MS) {  // 绕够了：出环
          inIsland_ = false;
          openLoop(SPEED_SLOW, SPEED_SLOW, ISLAND_EXIT_MS);
        } else {                                  // 入口边缘还没过完
          driveByTurn(SPEED_SLOW, curTurn);       // 保持当前转向继续切
        }
        return;
      }
      if (inFork_) { driveByTurn(SPEED_SLOW, curTurn); return; }
      /* 出库免疫窗内（车库旁长白线）不触发元素，直行冲过 */
      if (now - garageTime_ > GARAGE_IGNORE_MS) {
        /* 先问特征分类器，不确定再查兜底顺序表 */
        uint8_t plan = classifyLongWhite();
        uint8_t idx = lwEventIdx_;
        if (idx >= LW_PLAN_NUM) idx = LW_PLAN_NUM - 1;
        if (plan == 0xFF) plan = LONG_WHITE_PLAN[idx];
        lwEventIdx_++;
        if (plan == PLAN_ISLAND) {                // 环岛入口
          inIsland_ = true;  islandEnter_ = now;
          int t = ISLAND_DIR * 110;
          openLoop(SPEED_SLOW + t, SPEED_SLOW - t,
                   speedScaleMs(ISLAND_ENTRY_MS, SPEED_SLOW));
        } else if (plan == PLAN_FORK) {           // 岔路入口
          inFork_ = true;  forkEnter_ = now;
          int t = FORK_DIR * 110;
          openLoop(SPEED_SLOW + t, SPEED_SLOW - t,
                   speedScaleMs(ISLAND_ENTRY_MS, SPEED_SLOW));
        } else {                                  // 放弃该元素直行
          holdStraightUntil_ = now + 400;
        }
      } else {
        driveByTurn(SPEED_SLOW, 0);               // 免疫窗内：直行
      }
      return;
    }
    /* 短全白 = 十字：强制直行不许转弯 */
    if (!inIsland_ && !inFork_) driveByTurn(SPEED_SLOW, 0);
    else driveByTurn(SPEED_SLOW, curTurn);        // 岛内/岔内顶点：保持转向
    return;
  }
  /* 全白刚结束：若是短全白(十字)，补一小段直行保持，
   * 防止探头扫过十字横线边缘时误差抖动导致甩尾 */
  if (whiteStart_ != 0) {
    if (now - whiteStart_ < CROSS_MS)
      holdStraightUntil_ = now + HOLD_AFTER_CROSS_MS;
    whiteStart_ = 0;
  }

  /* 4) 全丢线：虚线断口(短) / 盲道(长) */
  if (gray.allLost()) {
    if (lostStart_ == 0) lostStart_ = now;
    if (now - lostStart_ > LOST_DASH_MS)
      blindUntil_ = now + BLIND_MS;               // 丢太久：按盲道直行
  } else lostStart_ = 0;
  if (now < blindUntil_) { driveByTurn(SPEED_SLOW, 0); return; }
  /* 丢线但未到盲道阈值：getError 的"记忆"自动保持上次误差，
   * 车会沿原方向直行冲过虚线断口，无需额外代码 */

  /* 5) 直角：半区全亮+对侧全灭，连续2拍确认，800ms 防重复
   *    开环时长按当前车速缩放，且允许"回线提前出弯"：
   *    低速多转、高速少转，转到看见线就交给 PID，不再赌固定时长 */
  if (!inIsland_ && !inFork_) {
    uint16_t ht = speedScaleMs(HARDTURN_MS, SPEED_SLOW);
    if (gray.hardLeft() && now - lastHardTurn_ > 800) {
      if (++hlCnt_ >= 2) {
        hlCnt_ = 0; lastHardTurn_ = now;
        openLoop(-60, 110, ht, true);             // 左直角：左轮反转
        return;
      }
    } else hlCnt_ = 0;
    if (gray.hardRight() && now - lastHardTurn_ > 800) {
      if (++hrCnt_ >= 2) {
        hrCnt_ = 0; lastHardTurn_ = now;
        openLoop(110, -60, ht, true);             // 右直角：右轮反转
        return;
      }
    } else hrCnt_ = 0;
  }

  /* 6) 十字后的直行保持期 */
  if (now < holdStraightUntil_) { driveByTurn(SPEED_SLOW, 0); return; }

  /* 7) 岔路三段式：循迹1400ms -> 原地掉头700ms -> 循迹1400ms -> 直行穿回T口 */
  if (inFork_) {
    unsigned long t = now - forkEnter_;
    if (t > FORK_TRACK_MS && t <= FORK_TRACK_MS + FORK_SPIN_MS) {
      motor.setWheels(-FORK_SPIN_PWM, FORK_SPIN_PWM);   // 原地掉头
      return;
    }
    if (t > 2 * FORK_TRACK_MS + FORK_SPIN_MS) {
      inFork_ = false;                            // 穿回 T 口，回主道
      openLoop(SPEED_SLOW, SPEED_SLOW, FORK_EXIT_MS);
      return;
    }
  }

  /* 8) 正常循迹 PID；切入后 600ms 内误差混合渐变（防甩尾） */
  float e = eRaw;
  float w = (now - blendStart_) / (float)BLEND_MS;
  w = constrain(w, 0.0f, 1.0f);
  if (w < 1.0f) e = e * w + em.getError() * (1.0f - w);
  curErr = e;
  curTurn = pid.update(curErr, CTRL_PERIOD_S);
  int base = (fabs(curErr) > 0.5f) ? SPEED_SLOW : SPEED_CRUISE;
  driveByTurn(base, curTurn);
}

/* --- 电磁循迹：差比和 PID + 圆环计时出环 --- */
void doEM() {
  unsigned long now = millis();

  /* 出段：弱信号连续12拍(60ms)确认，跷跷板俯仰的信号跌落扛得住 */
  if (em.strength() < EM_EXIT_TH) {
    if (++emLostCnt_ >= 12) {
      passedEM = true;                            // 从此终点判定解锁
      lwEventIdx_ = 0;                            // 重置长全白事件序号，终点长白线才能被识别
      enter(ST_TRACK);
      return;
    }
  } else emLostCnt_ = 0;

  /* 误差：入段后 600ms 内从灰度误差渐变到电感误差 */
  float e = em.getError();
  float w = (now - blendStart_) / (float)BLEND_MS;
  w = constrain(w, 0.0f, 1.0f);
  e = e * w + gray.getError() * (1.0f - w);
  curErr = e;
  curTurn = pid.update(e, CTRL_PERIOD_S);         // 先算本拍转向

  /* 圆环逻辑：满幅打角持续400ms=进环；满一圈后直行冲出 */
  if (now > ringCooldown_) {                      // 出环后5秒冷却
    if (fabs(curTurn) > 0.55f) {
      if (ringTurnSince_ == 0) ringTurnSince_ = now;
      if (ringEnterAt_ == 0 && now - ringTurnSince_ > RING_TURN_MS)
        ringEnterAt_ = now;                       // 确认进圆环
    } else ringTurnSince_ = 0;
  }
  if (ringEnterAt_ != 0) {                        // 在环上（含计时）
    if (now - ringEnterAt_ > RING_LAP_MS + RING_EXIT_MS) {
      ringEnterAt_ = 0;                           // 冲出完毕
      ringCooldown_ = now + 5000;
    } else if (now - ringEnterAt_ > RING_LAP_MS) {
      driveByTurn(SPEED_EM, 0);                   // 环口：直行冲出
      return;
    }
    /* 未到冲出时间：继续正常循迹（环就是一条线） */
  }

  driveByTurn(SPEED_EM, curTurn);                 // 跷跷板/U弯：低速硬过
}

/* --- 绕障三阶段 --- */
void doObstacle() {
  us.obstacleAhead();                             // 绕障全程保持超声波更新
  unsigned long t = millis() - stEnter_;
  if (obPhase_ == 0) {                            // 0：减速接近
    driveByTurn(60, 0);
    if (t > 100) obPhase_ = 1;
  } else if (obPhase_ == 1) {                     // 1：开环绕弧
    if (OB_AVOID_DIR > 0) motor.setWheels(110, -20);   // 右绕
    else                  motor.setWheels(-20, 110);   // 左绕
    if (t > 100 + OB_ARC_MS) obPhase_ = 2;
  } else {                                        // 2：找回白线
    float e = gray.getError();
    driveByTurn(60, e * 0.6f + OB_AVOID_DIR * 0.35f);  // 带回线偏置
    if (gray.centerOn()) {
      enter(ST_TRACK);                            // 找到线了
      return;
    }
    if (t > 100 + OB_ARC_MS + 2500) enter(ST_TRACK);   // 超时兜底
  }
}

/* --- 终点：急刹 -> 滑入车库 -> 停车长鸣 --- */
void doFinish() {
  unsigned long t = millis() - stEnter_;
  if (t < 150) {
    motor.setWheels(-100, -100);                  // 反向急刹
  } else if (t < 700) {
    motor.setWheels(SPEED_GARAGE, SPEED_GARAGE);  // 低速滑入车库
  } else {
    motor.setWheels(0, 0);                        // 停车（触发计时线圈）
    enter(ST_STOP);
  }
}

/* --- 停车庆祝 --- */
void doStop() {
  motor.setWheels(0, 0);
  if (millis() - stEnter_ < 1500) tone(PIN_BUZZER, BUZZ_FREQ);
  else noTone(PIN_BUZZER);
}

/* ==================== 调试打印 ==================== */
void debugPrint() {
  static unsigned long lastP = 0;
  if (millis() - lastP < 50) return;              // 20Hz 足够看曲线
  lastP = millis();
#if DEBUG_PLOT
  /* 串口绘图器：一行多曲线逗号分隔（误差/转向/电磁强度/距离）
   * em.strength() 除以 4000 缩放到 0~1，与误差/转向同量级方便同屏观察 */
  Serial.printf("%.2f,%.2f,%.2f,%.0f\n",
                curErr, curTurn, em.strength() / 4000.0f, us.distance());
#elif DEBUG_TEXT
  Serial.printf("[st=%d] gray=%02X err=%.2f turn=%.2f em=%.0f us=%.0f\n",
                (int)st, gray.pattern(), curErr, curTurn,
                em.strength(), us.distance());
#endif
}

/* ==================== setup / loop ==================== */
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  noTone(PIN_BUZZER);
  motor.begin();
  us.begin();
  gray.begin();
  em.begin();
#if VBAT_EN
  pinMode(PIN_VBAT, INPUT);
  analogSetPinAttenuation(PIN_VBAT, ADC_11db);
#endif
  pid.begin(PID_KP, PID_KI, PID_KD, -1.0f, 1.0f, PID_D_ALPHA);
  beep(1);                                        // 上电就绪：1 声
}

void loop() {
  us.task();                                      // 非阻塞超声波（每圈都跑，内部 60ms 节流）

  unsigned long nowUs = micros();
  if (nowUs - lastTickUs < (unsigned long)CTRL_PERIOD_MS * 1000UL) return;
  lastTickUs = nowUs;                             // 微秒级节拍，抖动 <1ms

#if VBAT_EN
  /* 100ms 读一次电池电压，算车速补偿系数 + 低电报警 */
  if (millis() - lastVbat_ > 100) {
    lastVbat_ = millis();
    float v = analogRead(PIN_VBAT) / 4095.0f * 3.3f * 2.0f;  // 100k/100k 分压
    v = vbatFilt.update(v);
    vbatComp_ = constrain(VBAT_REF / v, 0.85f, 1.2f);
    if (v < VBAT_LOW && st == ST_IDLE) tone(PIN_BUZZER, BUZZ_FREQ, 50);
  }
#endif

  gray.readAll();                                 // 传感器读取（数字轮询）
  em.readAll();
  us.obstacleAhead();                             // 更新障碍判定

  switch (st) {                                   // 状态机分派
    case ST_IDLE:       doIdle();       break;
    case ST_CHECK:      doCheck();      break;
    case ST_WAIT_RUN:   doWaitRun();    break;
    case ST_GARAGE_OUT: doGarageOut();  break;
    case ST_TRACK:      doTrack();      break;
    case ST_EM:         doEM();         break;
    case ST_OBSTACLE:   doObstacle();   break;
    case ST_FINISH:     doFinish();     break;
    case ST_STOP:       doStop();       break;
  }

  debugPrint();
}
