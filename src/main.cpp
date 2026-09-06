#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>

Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

MPU6050 mpu;
uint8_t fifoBuffer[64];
bool dmpReady = false;

#include <BMP390.h>

#define I2C_SDA 21
#define I2C_SCL 22

BMP390 bmp(I2C_SDA, I2C_SCL);
uint32_t tmrADXL = 0;   
uint32_t tmrMPU  = 0;   
uint32_t tmrBMP  = 0;  
uint32_t lastBaroTime = 0;

float accelData[3];
float angleData[3];
float pressure;

float pStart = 1013.25; 
float altitude = 0;     
float lastAlitude = 0;
float vs = 0;

void get_base_data(){
  sensors_event_t event;
  accel.getEvent(&event);
  accelData [0] = event.acceleration.x;
  accelData [1] = event.acceleration.y;
  accelData [2] = event.acceleration.z;

  if (dmpReady) {
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
      Quaternion q;
      VectorFloat gravity;
      float ypr[3];
      VectorInt16 mpuAccel;

      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      
      angleData[0] = degrees (ypr[0]);
      angleData[1] = degrees (ypr[1]);
      angleData[2] = degrees (ypr[2]);
    }
  }

  bmp3_data data = bmp.get_bmp_values();
  if (data.success) {
    uint32_t currentTime = millis();
    float dt = (currentTime - lastBaroTime);

    pressure = data.pressure/100.0;

    if (pStart > 0 && pressure > 0) {
      altitude = 44330.0 * (1.0 - pow(pressure / pStart, 0.1903));
      if (altitude < 0) altitude = 0; // Отсекаем отрицательный шум у земли
    }

    if (dt > 10) {
      vs = (altitude - lastAlitude) / (dt / 1000);

      lastBaroTime = currentTime;
      lastAlitude = altitude;
    }

  } else {
    Serial.println("[BMP390]  Ошибка чтения данных!");
  }
}


void setup() {
  Serial.begin(115200);
  delay(1500); 

  Serial.println("============================================");
  Serial.println("  Инициализация датчиков...");
  Serial.println("============================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.print("[1/3] ADXL375 (I2C)... ");
  if (!accel.begin()) {
    Serial.println("НЕ НАЙДЕН!");
  } else {
    accel.setTrimOffsets(0, 0, 0);
    delay(200);
    int16_t rx = accel.getX();
    int16_t ry = accel.getY();
    int16_t rz = accel.getZ();
    accel.setTrimOffsets(-(rx+2)/4, -(ry+2)/4, -(rz-20+2)/4);
    Serial.println("ok");
  }

  Serial.print("[1/2] MPU6050 (I2C)... ");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("НЕ НАЙДЕН! Проверь SDA=21, SCL=22.");
  } else {
    uint8_t devStatus = mpu.dmpInitialize();
    if (devStatus == 0) {

      mpu.setXGyroOffset(0);
      mpu.setYGyroOffset(0);
      mpu.setZGyroOffset(0);
      mpu.setXAccelOffset(0);
      mpu.setYAccelOffset(0);
      mpu.setZAccelOffset(0);

      mpu.setDMPEnabled(true);
      dmpReady = true;
      Serial.println("ok (DMP включён)");
    } else {
      Serial.print("ОШИБКА DMP: код ");
      Serial.println(devStatus);
    }
  }

 
  Serial.print("[2/2] BMP390 (I2C)... ");
  bmp3_data testData = bmp.get_bmp_values();
  if (testData.success) {
    Serial.println("ok");
  } else {
    Serial.println("НЕ НАЙДЕН! Проверь SDA=21, SCL=22.");
  }

  
  Serial.print("Запись стартового давления pStart... ");
  float pSum = 0;
  int samples = 0;
  for (int i = 0; i < 20; i++) {
    bmp3_data d = bmp.get_bmp_values();
    if (d.success) {
      pSum += d.pressure / 100.0f;
      samples++;
    }
    delay(20);
  }
  if (samples > 0) pStart = pSum / samples;

  Serial.print("pStart = ");
  Serial.print(pStart);
  Serial.println(" hPa");
}

void loop() {
  uint32_t now = millis();

  get_base_data();

  //Serial.print(now); Serial.print(", ");

  Serial.print( accelData  [0] ); Serial.print(", ");
  Serial.print( accelData  [1] ); Serial.print(", ");
  Serial.print( accelData  [2] ); Serial.print(", ");

  Serial.print(angleData  [0] ); Serial.print(", ");
  Serial.print(angleData  [1] ); Serial.print(", ");
  Serial.print(angleData  [2] ); Serial.print(", ");

  Serial.print(pressure); Serial.print(", ");
  Serial.print(altitude); Serial.print(", ");
  Serial.println(vs);
}