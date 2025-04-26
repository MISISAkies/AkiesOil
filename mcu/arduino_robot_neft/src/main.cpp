#include <Arduino.h>
#include "motor_regulator.h"
#include "ros2_communication.hpp"
#include "sensors.hpp"
#include "Octoliner.h"
#include "communication_module.h"
#include "logger.h"
#include <SoftwareSerial.h>

// Create a SoftwareSerial instance
SoftwareSerial softLogger(11, 12); // RX, TX pins

// Create a Logger instance
Logger logger(&softLogger);

#define BASE_SPEED 240


//ЛЕВЫЙ МОТОР
void __left_motor_enc(); //Заголовок функции
//Создание экземпляра левого регулятора
Regulator left_regulator(
  Motor(7, 6, true),
  Encoder(3, 10, __left_motor_enc, false), // 12 -> 10
  PID(1.1, 0.01, 0.005, 100) 
);

void __left_motor_enc() {
  left_regulator.encoder.encoder_int();
}

//ПРАВЫЙ МОТОР
void __right_motor_enc(); //Заголовок функции
//Создание экземпляра правого регулятора
Regulator right_regulator(
  Motor(4, 5),
  Encoder(2, 9, __right_motor_enc, false),  // 11 -> 9
  PID(1.1, 0.01, 0.005, 100)
);

void __right_motor_enc() {
  right_regulator.encoder.encoder_int();
}

Octoliner octoliner;
LineSensor lline(A3);
LineSensor rline(A2);
LineSensor mline(A1);
PID linePid(0.5, 0.05, 0.01, 100); // 18 плавно 20 остро / 
DriveBase drive_base(left_regulator, right_regulator);

void setup() {
  logger.begin();
  logger.setLogLevel(LOG_DEBUG);
  logger.debug("Init pins");
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(10, INPUT);
  pinMode(9, INPUT);
  octoliner.begin();
  octoliner.setSensitivity(240);
}

#define PT(x) Serial.print(x); Serial.print('\t')


void stop() {
  for (int i=0, s=255; i < 16; i++, s=-s){
    left_regulator.motor.set_pwmdir(s);
    right_regulator.motor.set_pwmdir(s);
    delay(5);
  }
  left_regulator.motor.set_pwmdir(0);
  right_regulator.motor.set_pwmdir(0);
}

#define LINE_TRASHOLD 700
bool is_cross() {
  int16_t brightness_value[8];
  octoliner.analogReadAll(brightness_value);
  int cnt = 0;
  for (int i_br_val = 0; i_br_val < 8; i_br_val++) {
    if (brightness_value[i_br_val] > LINE_TRASHOLD) {cnt++;}
  }
  //logger.debug(str_out);
  return (cnt > 5);
}

void readPIDCoefficients() {
  if (Serial.available() > 0) {
    stop();
    Serial.setTimeout(30000);
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    // Parse the data using comma as separator
    int firstCommaIndex = data.indexOf(',');
    int secondCommaIndex = data.indexOf(',', firstCommaIndex + 1);
    
    if (firstCommaIndex != -1 && secondCommaIndex != -1) {
      float kp = data.substring(0, firstCommaIndex).toFloat();
      float ki = data.substring(firstCommaIndex + 1, secondCommaIndex).toFloat();
      float kd = data.substring(secondCommaIndex + 1).toFloat();
      
      // Update PID coefficients
      linePid.kp = kp;
      linePid.ki = ki;
      linePid.kd = kd;
      
      // Reset integral to avoid sudden changes
      linePid.integral = 0;
      
      // Log the new coefficients
      logger.debug("Updated PID coefficients: Kp=" + String(kp) + ", Ki=" + String(ki) + ", Kd=" + String(kd));
      Serial.println("Updated PID coefficients: Kp=" + String(kp) + ", Ki=" + String(ki) + ", Kd=" + String(kd));
    } else {
      Serial.println("Invalid format. Use: kp,ki,kd");
    }
  }
}

void linefollow() {
  while (!is_cross()) {
    int ans = linePid.calc((octoliner.analogRead(1) - octoliner.analogRead(5))/5);
    left_regulator.motor.set_pwmdir(BASE_SPEED+ans);
    right_regulator.motor.set_pwmdir(BASE_SPEED-ans);
    readPIDCoefficients();
  }
}

void loop() {

  // linefollow();
  // stop();
  // left_regulator.motor.set_pwmdir(200); +- ЕЗДА ПРЯМО НА ВСЯКИЙ СЛУЧАЙ
  // right_regulator.motor.set_pwmdir(235);
  // delay(500);
  stop();
  while (true)
  {
    /* code */
  }
  
}
