#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <math.h>

#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <BMP390.h>

// ===== НАСТРОЙКИ ПИНОВ И ИНТЕРФЕЙСОВ =====
#define I2C_SDA 21
#define I2C_SCL 22
#define SD_CS   4   // Пин CS для SD-карты

// Назначение пинов UART (укажите свои пины!)
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

#define GSM_RX_PIN 26
#define GSM_TX_PIN 27

// ===== ОБЪЕКТЫ ДАТЧИКОВ И МОДУЛЕЙ =====
Adafruit_ADXL375 accel = Adafruit_ADXL375(12345);
MPU6050 mpu;
BMP390 bmp(I2C_SDA, I2C_SCL);
TinyGPSPlus gps;

// ===== ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ =====
uint8_t fifoBuffer[64];
bool dmpReady = false;

uint32_t lastBaroTime = 0;   // Таймер вариометра
uint32_t lastSDWriteTime = 0; // Таймер записи на SD-карту

float accelData[3]; // Ускорения ADXL375
float angleData[3]; // Угли MPU6050 (Yaw, Pitch, Roll)
float pressure;     // Давление hPa
float altitude = 0; // Отфильтрованная высота
float vs = 0;       // Вертикальная скорость

int gpsSats = 0;    // Спутники
float gpsLat = 0.0; // Широта
float gpsLng = 0.0; // Долгота

float pStart = 1013.25; // Давление на уровне земли
float lastAlitude = 0;  // Предыдущая высота

// ===== ФИЛЬТР КАЛМАНА ДЛЯ БАРОМЕТРА =====
float kalman_altitude = 0.0;
float kalman_pc = 0.0;
float kalman_q = 0.125; // Шум процесса
float kalman_r = 1.0;   // Шум измерений BMP390
float kalman_k = 0.0;
bool kalman_initialized = false;

float kalman_filter(float val) {
  if (!kalman_initialized) {
    kalman_altitude = val;
    kalman_initialized = true;
    return val;
  }
  float pc = kalman_pc + kalman_q;
  kalman_k = pc / (pc + kalman_r);
  kalman_pc = (1.0 - kalman_k) * pc;
  kalman_altitude = kalman_k * val + (1.0 - kalman_k) * kalman_altitude;
  return kalman_altitude;
}

// ===== ФУНКЦИИ ЛОГИРОВАНИЯ =====
void write_sys_log(String dataString) {
  File sysFile = SD.open("/syslog.txt", FILE_APPEND);
  if (sysFile) {
    sysFile.print(millis());
    sysFile.print(" - ");
    sysFile.println(dataString);
    sysFile.close();
  }
}

void write_data(String dataString) {
  File datFile = SD.open("/data.txt", FILE_APPEND);
  if (datFile) {
    datFile.println(dataString);
    datFile.close();
  }
}

// ===== ОПРОС БАЗОВЫХ ДАТЧИКОВ =====
void get_base_data() {
  // 1. Чтение ADXL375
  sensors_event_t event;
  accel.getEvent(&event);
  accelData[0] = event.acceleration.x;
  accelData[1] = event.acceleration.y;
  accelData[2] = event.acceleration.z;

  // 2. Чтение MPU6050 (DMP) без блокировки
  if (dmpReady) {
    uint16_t fifoCount = mpu.getFIFOCount();
    if (fifoCount >= 42) {
      if (fifoCount > 200) {
        mpu.resetFIFO(); // Сброс при переполнении
      } else {
        while (fifoCount >= 42) {
          mpu.getFIFOBytes(fifoBuffer, 42);
          fifoCount -= 42;
        }
        Quaternion q;
        VectorFloat gravity;
        float ypr[3];

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        angleData[0] = degrees(ypr[0]);
        angleData[1] = degrees(ypr[1]);
        angleData[2] = degrees(ypr[2]);
      }
    }
  }

  // 3. Чтение и фильтрация BMP390
  bmp3_data data = bmp.get_bmp_values();
  if (data.success) {
    uint32_t currentTime = millis();
    float dt = (currentTime - lastBaroTime);

    pressure = data.pressure / 100.0f;

    if (pStart > 0 && pressure > 0) {
      float rawAltitude = 44330.0f * (1.0f - pow(pressure / pStart, 0.1903f));
      if (rawAltitude < 0) rawAltitude = 0;

      altitude = kalman_filter(rawAltitude);
    }

    if (dt > 10) {
      vs = (altitude - lastAlitude) / (dt / 1000.0f);
      lastBaroTime = currentTime;
      lastAlitude = altitude;
    }
  } else {
    Serial.println("[BMP390] Ошибка чтения данных!");
    write_sys_log("ERR: bmp390 get data error");
  }
}

// ===== ОПРОС GPS (Serial1) =====
void get_GPS_data() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    gps.encode(c);
  }

  if (gps.location.isValid()) {
    gpsLat = gps.location.lat();
    gpsLng = gps.location.lng();
  }

  if (gps.satellites.isValid()) {
    gpsSats = gps.satellites.value();
  }
}

// ===== ЗАГОТОВКА ПОД GSM/SMS (Serial2) =====
void send_SMS(String phoneNumber, String messageText) {
  Serial2.println("AT+CMGF=1"); // Текстовый режим
  delay(100);
  Serial2.print("AT+CMGS=\"");
  Serial2.print(phoneNumber);
  Serial2.println("\"");
  delay(100);
  Serial2.print(messageText);
  delay(100);
  Serial2.write(26); // Ctrl+Z для отправки
  write_sys_log("INFO: SMS sent request");
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.print("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("initialization failed!");
    while (true);
  }
  Serial.println("initialization done.");

  // Подготовка файлов логов (перезапись при старте)
  File sysFile = SD.open("/syslog.txt", FILE_WRITE);
  if (sysFile) {
    sysFile.print(millis());
    sysFile.print(" - ");
    sysFile.println("System init start");
    sysFile.close();
  }

  File datFile = SD.open("/data.txt", FILE_WRITE);
  if (datFile) {
    datFile.println("time,ax,ay,az,yaw,pitch,roll,press,alt,vs,sats,lat,lng");
    datFile.close();
  }

  Serial.println("============================================");
  Serial.println("  Инициализация датчиков...");
  Serial.println("============================================");

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // 1. ADXL375
  Serial.print("[1/4] ADXL375 (I2C)... ");
  if (!accel.begin()) {
    Serial.println("НЕ НАЙДЕН!");
    write_sys_log("FATAL: adxl375 init fail");
    while (true);
  } else {
    accel.setTrimOffsets(0, 0, 0);
    delay(200);
    int16_t rx = accel.getX();
    int16_t ry = accel.getY();
    int16_t rz = accel.getZ();
    accel.setTrimOffsets(-(rx + 2) / 4, -(ry + 2) / 4, -(rz - 20 + 2) / 4);
    Serial.println("ok");
    write_sys_log("INFO: adxl375 pass");
  }

  // 2. MPU6050
  Serial.print("[2/4] MPU6050 (I2C)... ");
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("НЕ НАЙДЕН!");
    write_sys_log("FATAL: mpu6050 init fail");
    while (true);
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
      write_sys_log("INFO: mpu6050 pass DMP ok");
    } else {
      Serial.print("ОШИБКА DMP: код ");
      Serial.println(devStatus);
      write_sys_log("ERR: dmp fail code: " + String(devStatus));
    }
  }

  // 3. BMP390
  Serial.print("[3/4] BMP390 (I2C)... ");
  bmp3_data testData = bmp.get_bmp_values();
  if (testData.success) {
    Serial.println("ok");
    write_sys_log("INFO: bmp390 pass");
  } else {
    Serial.println("НЕ НАЙДЕН!");
    write_sys_log("FATAL: bmp390 init fail");
    while (true);
  }

  // 4. GPS (Serial1)
  Serial1.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("[4/4] GPS (Serial1) initialized");
  write_sys_log("INFO: gps serial1 init");

  // 5. GSM (Serial2)
  Serial2.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);
  write_sys_log("INFO: gsm serial2 init");

  write_sys_log("INFO: subsystems init complete");

  // Расчет стартового давления pStart
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

  write_sys_log("INFO: ground pressure: " + String(pStart));
}

// ===== MAIN LOOP =====
void loop() {
  uint32_t now = millis();

  // Непрерывный опрос базовых датчиков и GPS
  get_base_data();
  get_GPS_data();

  // Вывод в Serial-монитор (отладка)
  Serial.print(accelData[0]); Serial.print(", ");
  Serial.print(accelData[1]); Serial.print(", ");
  Serial.print(accelData[2]); Serial.print(", ");

  Serial.print(angleData[0]); Serial.print(", ");
  Serial.print(angleData[1]); Serial.print(", ");
  Serial.print(angleData[2]); Serial.print(", ");

  Serial.print(pressure); Serial.print(", ");
  Serial.print(altitude); Serial.print(", ");
  Serial.print(vs); Serial.print(", ");

  Serial.print(gpsSats); Serial.print(", ");
  Serial.print(gpsLat, 6); Serial.print(", ");
  Serial.println(gpsLng, 6);

  // Запись на SD-карту раз в 100 мс (10 Гц) без задержки loop()
  if (now - lastSDWriteTime >= 100) {
    lastSDWriteTime = now;

    String csvRow = String(now) + "," +
                    String(accelData[0], 2) + "," + String(accelData[1], 2) + "," + String(accelData[2], 2) + "," +
                    String(angleData[0], 2) + "," + String(angleData[1], 2) + "," + String(angleData[2], 2) + "," +
                    String(pressure, 2) + "," + String(altitude, 2) + "," + String(vs, 2) + "," +
                    String(gpsSats) + "," + String(gpsLat, 6) + "," + String(gpsLng, 6);

    write_data(csvRow);
  }
}