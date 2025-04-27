#define dirL 7
#define speedL 6
#define speedR 5
#define dirR 4

#include "Arduino.h"
#include "Octoliner.h"

volatile long encL = 0;
volatile long encR = 0;

Octoliner octoliner;

// --- Калибровка и нормализация датчиков
int octo_min[8] = {1023,1023,1023,1023,1023,1023,1023,1023};
int octo_max[8] = {0,0,0,0,0,0,0,0};

float sensor_position[8] = {-3.5, -2.5, -1.5, -0.5, 0.5, 1.5, 2.5, 3.5}; // слева - справа

void octoliner_calibrate(int scans=200) {
  for(int i=0; i<scans; i++) {
    for(int j=0; j<8; j++) {
      int v = octoliner.analogRead(j);
      if(v < octo_min[j]) octo_min[j] = v;
      if(v > octo_max[j]) octo_max[j] = v;
    }
    delay(3);
  }
  Serial.println("Octoliner calibration done.");
  for(int i=0;i<8;i++) {
    Serial.print("min["); Serial.print(i); Serial.print("]="); Serial.print(octo_min[i]);
    Serial.print(" max["); Serial.print(i); Serial.print("]="); Serial.print(octo_max[i]); Serial.print("; ");
  }
  Serial.println();
}

float octoNorm(int idx) {
  int value = octoliner.analogRead(idx);
  if(octo_max[idx]==octo_min[idx]) return 0;
  float res = (float)(value - octo_min[idx]) / (octo_max[idx] - octo_min[idx]);
  if (res < 0) res = 0; 
  if (res > 1) res = 1;
  return res;
}

float line_position() {
  float numerator = 0, denominator = 0;
  for (int i = 0; i < 8; i++) {
    float v = octoNorm(i);
    numerator += v * sensor_position[i];
    denominator += v;
  }
  if (denominator == 0) return 0; // если не видим линию
  return numerator / denominator;
}
// --- Конец блока калибровки/нормализации -------------

void e_encL() {
  if(digitalRead(2) == digitalRead(9)) {
    encL++;
  } else {
    encL--;
  }
}
void e_encR() {
  if(digitalRead(3) == digitalRead(10)) {
    encR--;
  } else {
    encR++;
  }
}

void move_(int left, int right) {
  digitalWrite(dirL, left < 0);
  digitalWrite(dirR, right > 0);

  left = abs(constrain(left, -100, 100));
  right = abs(constrain(right, -100, 100));
  left = map(left, 0, 100, 0, 255);
  right = map(right, 0, 100, 0, 255);

  analogWrite(speedL, left);
  analogWrite(speedR, right);
}

void stop_() {
  encL = 0;
  encR = 0;
  float k = 20;
  unsigned long long point = millis();
  while(point + 400 >= millis()) {
    move_(-encL * k, -encR * k);
  }
  move_(0, 0);
}

void move_sync(int speed_) {
  float k = 1.5;
  float err = (encL - encR) * k;
  move_(speed_ - err, speed_ + err);
}

void move_cm(int speed_, int dist) {
  encL = 0;
  encR = 0;
  while(abs(encL) / 35 <= dist) {
    move_sync(speed_);
    delay(10);
  }
}

// --- PID параметры ---
// !!! Переподбери под новый способ определения ошибки (line_position возвращает в районе [-3.5;+3.5]) !!!
float Kp = 18.0;   // подбери! значения множителя сильно растут (несколько-десятков)
float Ki = 0.06;
float Kd = 26.0;

float last_err = 0;
float sum_err = 0;

// --- Новый PID по 8 датчикам
void line(int speed_) {
  float err = line_position();    // теперь ошибка определена из центра линии по 8 датчикам

  sum_err += err;
  float d_err = err - last_err;
  last_err = err;

  float pid = Kp * err + Ki * sum_err + Kd * d_err;
  
  move_(speed_ - pid, speed_ + pid);
}

void line_cm(int speed_, int dist) {
  encL = 0;
  encR = 0;
  while(encL / 35 <= dist) {
    line(speed_);
    delay(10);
  }
}

#define LINE_TRASHOLD 0.90 // !!! теперь в нормализованных величинах, НЕ в AnalogRead! (0.9 = 90% "черноты")
// bool is_cross() {
//   int16_t brightness_value[8];
//   octoliner.analogReadAll(brightness_value);
//   int cnt = 0;
//   for (int i_br_val = 0; i_br_val < 8; i_br_val++) {
//     if (brightness_value[i_br_val] > LINE_TRASHOLD) {cnt++;}
//   }
//   //logger.debug(str_out);
//   return (cnt > 5);
// }
bool is_cross() {
  return (octoNorm(0) > LINE_TRASHOLD || octoNorm(7) > LINE_TRASHOLD);
}
void cross(int n = 1){ 
  unsigned long long point = millis();
  while(n) {
    line(85);
    if(is_cross() and point <= millis()) {
      n--;
      point = millis() + 500;
    }
    delay(10);
  }
}

void turn_enc(int angle) {
  encL = 0;
  encR = 0;
  if(angle > 0) {
    while(encL / 5.35 <= angle) {
      move_(70, -70);
    }
  } else {
    while(abs(encL) / 5.35 <= abs(angle)) {
      move_(-70, 70);
    }
  }
}

void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  attachInterrupt(0, e_encL, CHANGE);
  attachInterrupt(1, e_encR, CHANGE);

  Serial.begin(115200);
  octoliner.begin();
  octoliner.setSensitivity(200);

  delay(1500);
  octoliner_calibrate();
  delay(500);
}

void printEncSignals() {
  Serial.print(" L_A: "); Serial.print(digitalRead(2));
  Serial.print(" L_B: "); Serial.print(digitalRead(9));
  Serial.print("| R_A: "); Serial.print(digitalRead(3));
  Serial.print(" R_B: "); Serial.println(digitalRead(10));
}

void printLineVals() {
  Serial.print("OCTO: "); Serial.print(octoliner.analogRead(7));
  Serial.print(" "); Serial.print(octoliner.analogRead(6));
  Serial.print(" "); Serial.print(octoliner.analogRead(5));
  Serial.print(" "); Serial.print(octoliner.analogRead(4));
  Serial.print(" "); Serial.print(octoliner.analogRead(3));
  Serial.print(" "); Serial.print(octoliner.analogRead(2));
  Serial.print(" "); Serial.print(octoliner.analogRead(1));
  Serial.print(" "); Serial.println(octoliner.analogRead(0));
}

void printLineNorm() {
  Serial.print("NORM: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(octoNorm(i), 2); Serial.print(" ");
  }
  Serial.print(" | POS: "); Serial.println(line_position(), 2);
}

void loop() {

  delay(3000);

  cross(1);
  move_cm(70, 12);
  stop_();
  turn_enc(-95);
  stop_();
  
  line_cm(80, 50);
  cross(1);
  move_cm(70, 12);
  cross(1);
  move_cm(70, 12);
  
  cross(1);
  move_cm(70, 12);
  stop_();
  turn_enc(-94);
  stop_();

  cross(1);
  move_cm(70, 12);
  
  cross(1);
  move_cm(70, 12);
  stop_();
  turn_enc(-94);
  stop_();

  cross(1);
  move_cm(70, 12);
  cross(1);
  move_cm(70, 12);

  cross(1);
  move_cm(70, 12);
  stop_();
  turn_enc(-94);
  stop_();

  cross(1);
  move_cm(70, 12);
  stop_();

  delay(50000);

  // Для теста раскомментируй:
  // printLineNorm(); delay(100);

  delay(10);
}