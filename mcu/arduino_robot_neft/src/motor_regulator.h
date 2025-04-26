#pragma once

#define DT 0.01
#define MAX_DELTA 10 //Максимальная фактическая скорость

struct PID {
  float kp, ki, kd, max_i;
  float integral = 0;
  float old_error = 0;

  PID (float p, float i, float d, float mi) : kp(p), ki(i), kd(d), max_i(mi) {}
  
  float calc(float error) {
    integral += error * DT * ki;
    integral = constrain(integral, -max_i, max_i);
    float diff = (error - old_error) * kd / DT;
    old_error = error;
    return error * kp + integral + diff;
  }
};

struct Encoder {
  int speed = 0;
  volatile long ticks = 0, prev_ticks = 0;
  byte pin_a, pin_b;
  bool invert;
  Encoder (byte pin_enc_a, byte pin_enc_b, void(*on_enc)(void), bool inverted) : pin_a(pin_enc_a), pin_b(pin_enc_b), invert(inverted){
    pinMode(pin_a, INPUT);
    pinMode(pin_b, INPUT);
    attachInterrupt(digitalPinToInterrupt(pin_a), on_enc, RISING);
  }

  void encoder_int() {
    int step = 1;
    if (invert) step*=-1;
    if (digitalRead(pin_b)) ticks+=step;
    else ticks-=step;
  }

  int calc_delta() {
    speed = ticks - prev_ticks;
    prev_ticks = ticks;
    return speed;
  }
  
};

struct Motor {
  byte dir, speed;
  bool inverted;
  Motor (byte pin_dir, byte pin_speed, bool inverted = false) : dir(pin_dir), speed(pin_speed), inverted(inverted) {
    pinMode(dir, OUTPUT);
    pinMode(speed, OUTPUT);
  }

  void set_pwmdir(int pwm_dir) {
    int pwm = constrain(abs(pwm_dir), 0, 255);
    analogWrite(speed, pwm);
    digitalWrite(dir, inverted ? !(pwm_dir > 0) : (pwm_dir > 0));
  }
};

struct Regulator { 

  Motor motor;
  Encoder encoder;
  PID pid;

  long next = 0;
  int delta = 0;
  
  
  Regulator (Motor&& motor, Encoder&& encoder, PID&& pid) : motor(motor), encoder(encoder), pid(pid) {}

  void update() {
    if (delta == 0){
      motor.set_pwmdir(0);
      next = encoder.ticks;
    }else {
      next += delta;
      motor.set_pwmdir(pid.calc(next - encoder.ticks));
      encoder.calc_delta();
    }

  }
  void set_delta(int new_delta) {
    delta = constrain(new_delta, -MAX_DELTA, MAX_DELTA);
    if(new_delta == 0) {
      next = encoder.ticks;
      delta = 0;
      //EXPEREMENTAL
//      for (int i=0, s=255; i < 15; i++, s=-s){
//        motor.set_pwmdir(s);
//        delay(5);
//      }
      motor.set_pwmdir(0);
    }
  }
};

// Копируйте этот блок после регулятора!

#define WHEEL_BASE 180.0f     // мм, расстояние между колес
#define WHEEL_DIAMETER 69.0f  // мм, диаметр колеса
#define PI 3.1415926

struct DriveBase {
  Regulator &left;
  Regulator &right;
  float wheelBase;      // мм между колесами
  float wheelDiameter;  // мм. Для перевода градусов и мм в тики энкодера
  long targetL = 0;
  long targetR = 0;  
  bool moving = false;
  bool turning = false;

  DriveBase(Regulator& l, Regulator& r, float wb = WHEEL_BASE, float wd = WHEEL_DIAMETER) 
    : left(l), right(r), wheelBase(wb), wheelDiameter(wd) {}

  // Проехать точное количество тиков (обеими колесами)
  void driveTicks(long delta_ticks, int speed = 4) {
    left.next = left.encoder.ticks;
    right.next = right.encoder.ticks;
    targetL = left.encoder.ticks + delta_ticks;
    targetR = right.encoder.ticks + delta_ticks;
    left.set_delta(speed);
    right.set_delta(speed);
    moving = true;
    turning = false;
  }

  // Проехать дистанцию в мм
  void driveDistance(float mm, int speed = 4) {
    // Количество тиков для одного мм
    float ticks_per_mm = encoderTicksPerRev() / (PI * wheelDiameter);
    long ticks = mm * ticks_per_mm;
    driveTicks(ticks, speed);
  }

  // Поворот на угол (DEG) — положит. угол по часовой
  void turnAngle(float deg, int speed = 4) {
    float ticks_per_mm = encoderTicksPerRev() / (PI * wheelDiameter);
    // arc = pi*D/360*deg; delta-ticks=arc*(ticks_per_mm)
    float arc_mm = (PI * wheelBase) * (abs(deg) / 360.0);
    long ticks = arc_mm * ticks_per_mm;
    left.next = left.encoder.ticks;
    right.next = right.encoder.ticks;
    if (deg > 0) {
      targetL = left.encoder.ticks + ticks;
      targetR = right.encoder.ticks - ticks;
      left.set_delta(speed);
      right.set_delta(-speed);
    } else {
      targetL = left.encoder.ticks - ticks;
      targetR = right.encoder.ticks + ticks;
      left.set_delta(-speed);
      right.set_delta(speed);
    }
    moving = false;
    turning = true;
  }
  // Внутри DriveBase:

  // Синхронный проезд определённого количества тиков обоими моторами
  void driveTicksSync(long delta_ticks, int speed = 4) {
    driveTicks(delta_ticks, speed);
    while (moving) {
      update();
      delay(5);
    }
  }

  // Синхронный проезд нужной дистанции в мм
  void driveDistanceSync(float mm, int speed = 4) {
    driveDistance(mm, speed);
    while (moving) {
      update();
      delay(5);
    }
  }

  // Синхронный поворот на угол deg (градусы)
  void turnAngleSync(float deg, int speed = 4) {
    turnAngle(deg, speed);
    while (turning) {
      update();
      delay(5);
    }
  }
  // Обновляйте в loop()
  void update() {
    left.update();
    right.update();

    if (moving) {
      // Достигли цели?
      bool l_stop = (abs(left.encoder.ticks - targetL) <= 3);
      bool r_stop = (abs(right.encoder.ticks - targetR) <= 3);
      if (l_stop) left.set_delta(0);
      if (r_stop) right.set_delta(0);
      if (l_stop && r_stop) moving = false;
    }
    if (turning) {
      bool l_stop = (abs(left.encoder.ticks - targetL) <= 3);
      bool r_stop = (abs(right.encoder.ticks - targetR) <= 3);
      if (l_stop) left.set_delta(0);
      if (r_stop) right.set_delta(0);
      if (l_stop && r_stop) turning = false;
    }
  }

  // Этот метод обязательно реализуйте согласно вашему энкодеру
  float encoderTicksPerRev() const {
    // Верните количество тиков на один оборот колеса.
    // Пример: return 360.0f; 
    // Если у вас 20 магнитов и 18 редуктор: 20*18 = 360 тиков на оборот
    return 11.0f;
  }

  // True — движется/поворачивается
  bool isBusy() const {
    return moving || turning;
  }
};
