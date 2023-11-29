#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_VL53L0X.h>
#include <ServoDriverSmooth.h>
#include <Servo.h>
#include <EEPROM.h>

#define CTRL_MLTX 3
#define NUM_IR 2
#define NUM_END 4
#define DATA_NRF 6
#define EEP_ADDR 0
#define ERROR_ADR 100
#define PLAT_STAT_ADR 150

struct Receive
{
  char init_sb = '#';
  uint8_t hsum = 9;
  int8_t move_type = 0;
  int8_t val_move = 0;
  int16_t arm_q1 = 110;
  int16_t arm_q2 = 90;
  int16_t arm_q3 = 45;
  int8_t arm_mode = 60;
  int8_t auido_mode = -1;
};
Receive rx;

int16_t plat_status = 10;



void setup()
{
  /* ЗАКОММЕНТИ!! ПИШЕМ ЭТО ОДИН РАЗ ДЛЯ ОТЛДАКИ!!!*/
   rx.arm_q1 = 110;
   rx.arm_q2 = 100;
   rx.arm_q3 = 60;
   rx.arm_mode = 60;
   EEPROM.put(EEP_ADDR, rx);
   EEPROM.put(PLAT_STAT_ADR, plat_status);
}
void loop(){
  
}
