#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

#include <TinyGPSPlus.h>

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

#define SD_CS 4 // пин СД карты

BMP390 bmp(I2C_SDA, I2C_SCL);

uint32_t lastBaroTime = 0; // таймер вариометра

float accelData[3]; // ускорения
float angleData[3]; // углы
float pressure;     // давление
float altitude = 0; // высота    
float vs = 0;       // вертикальная скорость

int gpsSats; // сюда все данные по спутникам и координатам
float gpsLat;
float gpsLng;

float pStart = 1013.25; // давление на земле
float lastAlitude = 0; // последняя высота для вариометра

String smsPhone = "";
String smsText  = "";
bool isBusy     = false; // Флаг: занят ли GSM-модуль отправкой

enum GSMState { IDLE, SET_FORMAT, SET_NUMBER, SEND_TEXT };
GSMState gsmState = IDLE;
unsigned long gsmTimer = 0;

void get_base_data(){ // тут мы получаем всю основную информацию
  sensors_event_t event; // что то для работы акселерометра
  accel.getEvent(&event);
  accelData [0] = event.acceleration.x; // всего в три строки получаем все ускорения, я было чуть не пропустил этот блок
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
      
      angleData[0] = degrees (ypr[0]); // углы
      angleData[1] = degrees (ypr[1]);
      angleData[2] = degrees (ypr[2]);
    }
  }

  bmp3_data data = bmp.get_bmp_values(); // давление, высота, вертикальная скорость
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
    Serial.println("[BMP390]  Ошибка чтения данных!"); // ловим ошибки чтения данных
    write_sys_log("ERR: bmp390 get data error");
  }
}

void get_GPS_data() {
  // тут нужно получать данные от ЖПС, а именно:
  // координаты, количество спутников
  // всё это тоже в глобальные переменные и потом будем обрабатывать
}

void write_sys_log (String dataString) { // функция для записи логов
  File dataFile = SD.open("/syslog.txt", FILE_APPEND);
  dataFile.print(millis());
  dataFile.print(" - ");
  dataFile.println(dataString);
  dataFile.close();
}

void write_data (String dataString) { // функция для записи данных
  File dataFile = SD.open("/data.txt", FILE_APPEND);
  dataFile.println(dataString);
  dataFile.close();
}

void processGSM() {
  if (!isBusy) return;

  switch (gsmState) {
    case IDLE:
      Serial2.print("AT+CMGF=1\r");
      gsmState = SET_FORMAT;
      gsmTimer = millis();
      break;

    case SET_FORMAT:
      if (Serial2.find("OK")) {
        Serial2.print("AT+CMGS=\"" + smsPhone + "\"\r");
        gsmState = SET_NUMBER;
        gsmTimer = millis();
      } else if (millis() - gsmTimer > 5000) { isBusy = false; } // Тайм-аут 5 сек
      break;

    case SET_NUMBER:
      if (Serial2.find(">")) {
        Serial2.print(smsText);
        Serial2.write((char)26); // CTRL+Z
        gsmState = SEND_TEXT;
        gsmTimer = millis();
      } else if (millis() - gsmTimer > 5000) { isBusy = false; }
      break;

    case SEND_TEXT:
      if (Serial2.find("OK")) {
        isBusy = false;
      } else if (millis() - gsmTimer > 10000) { isBusy = false; } // На отправку 10 сек
      break;
  }
}

bool sendSMS(String phone, String text) {
  if (isBusy) return false; // Если модуль уже шлет SMS, игнорируем новое

  smsPhone = phone;
  smsText = text;
  isBusy = true;
  gsmState = IDLE; // Запускаем автомат
  return true;
}

void send_GSM_data () {

}


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  while (!Serial);
  while (!Serial2);

  Serial.print("Initializing SD card..."); // Подключаем карту
  if(!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    while (true);
  }
  Serial.println("initialization done.");

  File dataFile = SD.open("/syslog.txt", FILE_WRITE); // тут мы используя FILE_WRITE очищаем старые данные
  dataFile.print(millis());
  dataFile.print(" - ");
  dataFile.print("System init start");

  File dataFile = SD.open("/data.txt", FILE_WRITE);
  dataFile.print(millis());
  dataFile.print(" - ");
  dataFile.print("System init start");

  Serial.println("============================================"); // просто отлладка
  Serial.println("  Инициализация датчиков...");
  Serial.println("============================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Serial.print("[1/3] ADXL375 (I2C)... ");
  if (!accel.begin()) {
    Serial.println("НЕ НАЙДЕН!");
    write_sys_log("FATAL: adxl375 init fail"); // лог
    while (true); // и полная остановка
  } else {
    accel.setTrimOffsets(0, 0, 0);
    delay(200);
    int16_t rx = accel.getX();
    int16_t ry = accel.getY();
    int16_t rz = accel.getZ();
    accel.setTrimOffsets(-(rx+2)/4, -(ry+2)/4, -(rz-20+2)/4);
    Serial.println("ok");
    write_sys_log("INFO: adxl375 pass"); // лог
  }

  Serial.print("[2/3] MPU6050 (I2C)... ");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("НЕ НАЙДЕН! Проверь SDA=21, SCL=22.");
    write_sys_log("FATAL: mpu6050 init fail"); // лог
    while (true); // и полная остановка
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
      write_sys_log("INFO: mpu6050 pass DMP ok"); // лог
    } else {
      Serial.print("ОШИБКА DMP: код ");
      Serial.println(devStatus);
      write_sys_log("ERR: dmp fail, err code:"); // тоже логи
      write_sys_log(String(devStatus));
    }
  }

 
  Serial.print("[3/3] BMP390 (I2C)... ");
  bmp3_data testData = bmp.get_bmp_values();
  if (testData.success) {
    Serial.println("ok");
    write_sys_log("INFO: bmp390 pass"); //лог
  } else {
    Serial.println("НЕ НАЙДЕН! Проверь SDA=21, SCL=22.");
    write_sys_log("FATAL: bmp390 init fail"); //лог
    while (true); // и полная остановка
  }

  write_sys_log("INFO: subsystems init comlete"); //лог

  
  Serial.print("Запись стартового давления pStart... "); // начинаем работать
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

  write_sys_log("INFO: ground pressure: "); //лог
  write_sys_log(String(pStart)); //лог
}

void loop() { // тут уже всё серьезно и пытаеся по минимуму что либо делать
  uint32_t now = millis(); //! записываем время на всякий, потом УБРАТЬ

  get_base_data();

  //Serial.print(now); Serial.print(", ");

  Serial.print( accelData  [0] ); Serial.print(", "); //! тут выводим все данные, потом убрать, это чисто отладка
  Serial.print( accelData  [1] ); Serial.print(", ");
  Serial.print( accelData  [2] ); Serial.print(", ");

  Serial.print(angleData  [0] ); Serial.print(", ");
  Serial.print(angleData  [1] ); Serial.print(", ");
  Serial.print(angleData  [2] ); Serial.print(", ");

  Serial.print(pressure); Serial.print(", ");
  Serial.print(altitude); Serial.print(", ");
  Serial.println(vs);
}