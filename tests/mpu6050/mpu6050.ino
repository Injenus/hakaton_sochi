#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <EEPROM.h>

MPU6050 mpu;
// переменные для расчёта (ypr можно вынести в глобал)
Quaternion q;
VectorFloat gravity;
float ypr[3];

uint8_t fifoBuffer[45];         // буфер

#define BUFFER_SIZE 100
#define START_BYTE 1010
int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  //Wire.setClock(1000000UL);   // разгоняем шину на максимум

  // инициализация DMP
  mpu.initialize();
  //----------------- ВСПОМИНАЕМ ОФФСЕТЫ ------------------------
  int offsets[6];
  EEPROM.get(START_BYTE, offsets);

  // ставим оффсеты из памяти
  mpu.setXAccelOffset(offsets[0]);
  mpu.setYAccelOffset(offsets[1]);
  mpu.setZAccelOffset(offsets[2]);
  mpu.setXGyroOffset(offsets[3]);
  mpu.setYGyroOffset(offsets[4]);
  mpu.setZGyroOffset(offsets[5]);
  //----------------- ВСПОМИНАЕМ ОФФСЕТЫ ------------------------
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
  Serial.println("Current offsets:");
  Serial.println("accX accY accZ gyrX gyrY gyrZ");
  Serial.print(mpu.getXAccelOffset()); Serial.print(", ");
  Serial.print(mpu.getYAccelOffset()); Serial.print(", ");
  Serial.print(mpu.getZAccelOffset()); Serial.print(", ");
  Serial.print(mpu.getXGyroOffset()); Serial.print(", ");
  Serial.print(mpu.getYGyroOffset()); Serial.print(", ");
  Serial.print(mpu.getZGyroOffset()); Serial.println(" ");
  Serial.println(" ");
}

void loop() {
  static uint32_t tmr;
  if (millis() - tmr >= 11) {  // таймер на 11 мс (на всякий случай)
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {

      // расчёты
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

      // выводим результат в радианах (-3.14, 3.14)
      Serial.print(degrees(ypr[0])); // вокруг оси Z
      Serial.print(',');
      Serial.print(degrees(ypr[1])); // вокруг оси Y
      Serial.print(',');
      Serial.print(degrees(ypr[2])); // вокруг оси X
      Serial.println();
      // для градусов можно использовать degrees()

      tmr = millis();  // сброс таймера
    }
  }
}
