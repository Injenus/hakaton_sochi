/*
   Приводной уровень для двухколёсного мобильного робота на Atmega328p (Arduino Nano).
   Оснащён:
   1) Два привода колёс (servo 360) (direct PWM)
   2) IMU MPU6050 (I2C)
   3) Поворотный лазерный дальномер (VL53L0X + servo 180) (I2C + direct PWM)
   4) Приёмо-передатчик NRF24L01 (SPI)
   5) Два ИК-датчика цифр. (direct Analog)
   6) Два магнитных одометра (direct Analog)
   7) 4 механичеcких концевика (через мультиплексор, direct 2 pin)
   8) Манипулятор (4 servo, через PCA9875, I2C)

   Режим работы выбирается высоким уровнем по UART:
   0 - по умолчанию - ручой режим (с пульта NRF24L01)
   1 - режим задания 1
   2 - режим задания 2
   ...

   В ручном режиме по UART ничего не отправляем, только слушаем периодически.
   Один стик для робота, другой для манипулятора (ось Z вместо Y при нажатой другой кнопке)

   Не в ручном - шлём и принимаем.

   В любой режиме шлём пакет вида:
   %<hash><mode_left_wh>,<mode_right_wh>,<mode_move>,<x>,<y>,<z>,<grip>,<...9 mpu data>,<odo_l>,<odo_r>,<IR_left>,<IR_right>,<IR_3>,<sw1>,<sw2>,<sw3>,<sw4>,<lidar_angle>,<lidar_dist>,<sonar_1>,<sonar_2>;/n = 62 bytes

  Сначала шлёт МК, потом (по принятию) шлёт ПК
*/

#define IS_TEST_UART 0

#if (!IS_TEST_UART)
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_VL53L0X.h>
#include <ServoDriverSmooth.h>
#include <Servo.h>
#include <EEPROM.h>
#include "GyverWDT.h"
#endif
// #include <stdint.h>
#include <Arduino.h>
const uint8_t MODE = 1;
#define CTRL_MLTX 3
#define NUM_IR 2
#define NUM_END 4
#define DATA_NRF 6
#define EEP_ADDR 0
#define ERROR_ADR 100
#define PLAT_STAT_ADR 150

struct Timer
{
  uint32_t main = 0;
  uint32_t tx = 0;
  uint32_t rx = 0;
  uint32_t nrf_t = 0;
  uint32_t nrf_r = 0;
  uint32_t set_wheel = 0;
  uint32_t set_arm = 0;
  uint32_t set_periph = 0;
  uint32_t check_mltx = 0;
  uint32_t check_lidar = 0;
  uint32_t check_imu = 0;
  uint32_t check_odo = 0;
  uint32_t check_nrf = 0;
};
Timer tmr;

struct Period
{
  const uint32_t main = 0;
  const uint32_t tx = 200;
  const uint32_t rx = 50;
  const uint32_t nrf_t = 5;
  const uint32_t nrf_r = 5;
  const uint32_t set_wheel = 11;
  const uint32_t set_arm = 42;
  const uint32_t set_periph = 15;
  const uint32_t check_mltx = 15;
  const uint32_t check_lidar = 5;
  const uint32_t check_imu = 14;
  const uint32_t check_odo = 5;
  const uint32_t check_nrf = 100;
};
Period PRD;

struct Num
{
  const uint8_t arm = 4;
  const uint8_t ir = 2;
  const int end_sens = 4;
  const int mltx_ctrl = 3;
};
Num num;

struct Arm
{
  const uint8_t base = 0;
  const uint8_t first = 2;
  const uint8_t second = 4;
  const uint8_t gripper = 6;

  uint8_t servo[4] = {0, 2, 4, 6};
};

struct Multiplexor
{
  const uint8_t s_ctrl[CTRL_MLTX] = {15, 16, 17}; // A1 A2 A3  //num.mltx_ctrl
  const uint8_t sig = 14;                         // A0
  uint8_t ir[NUM_IR][CTRL_MLTX] = {               // num.ir  //num.mltx_ctrl
      {0, 0, 0},
      {1, 0, 0}};
  uint8_t end_sens[NUM_END][CTRL_MLTX] = { // num.end_sens  //num.mltx_ctrl
      {0, 1, 0},
      {1, 1, 0},
      {0, 0, 1},
      {1, 0, 1}};
  uint8_t pin_mode = 0;
};

struct Pin
{ // reserved A4-A5(18-19)(I2C), 11-13(SPI), 0-1(UART), 2-3(extrn. interr.)
  const uint8_t left_wh = 9;
  const uint8_t right_wh = 10;
  const uint8_t CE = 7;
  const uint8_t CSN = 8;
  const uint8_t lidar_servo = 5;
  const uint8_t left_odo = 20;  // A6
  const uint8_t right_odo = 21; // A7
  const uint8_t buzz = 4;
  Arm arm;
  Multiplexor mltx;
};
Pin pin;

struct Rec_nrf
{
  int16_t data[DATA_NRF];
  uint8_t pipeNo;
  int16_t x1 = 512;
  int16_t y1 = 512;
  int16_t x2 = 512;
  int16_t y2 = 512;
  uint8_t btn1 = 0;
  uint8_t btn2 = 0;
};
Rec_nrf rec_nrf;

struct Transmit
{
  char start_sb = '%';
  // char end_sb = ';';
  // char terminator = ',';
  uint8_t hsum = 0x09;
  int16_t left_wh = 0;
  int16_t right_wh = 0;
  int16_t mode_move = 3; // сейчас считаем это за индикатор состяния последней команды 1 - выполнено, 0 - выполняется
  int16_t x_arm = 90;
  int16_t y_arm = 90;
  int16_t z_arm = 90;
  int16_t mode_arm = -1;
  int16_t ax = -123;
  int16_t ay = -1234;
  int16_t az = -12345;
  int16_t gx = -123;
  int16_t gy = -1234;
  int16_t gz = -12345;
  int16_t ang_x = -123;
  int16_t ang_y = -1234;
  int16_t ang_z = -12345;
  int16_t odo_l = 1;
  int16_t odo_r = 2;
  int16_t lidar_angle = 3;
  int16_t lidar_dist = 4;
  int16_t sonar_1 = 5;
  int16_t sonar_2 = 6;
  int8_t ir = 0b00000011;       // 0b00000011
  int8_t end_sens = 0b00001111; // 0b00001111
};
Transmit tx;

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

struct Buff
{
  uint8_t rx[11] = {0, 0, 0, 110, 0, 90, 0, 45, 0, 60, 0}; // hsum + 2*1 + 3*2 + 2*1
  uint8_t two_bytes[2];
  uint8_t tx[48]; // hsum + 22*2+2
  uint8_t tx_add[16];
  uint8_t nrf_rec[12];
};
Buff buff;
bool rx_flag = false;

struct Pid
{
  int32_t kp = 0;
  int32_t ki = 0;
  int32_t kd = 0;
  int32_t dt = 20;
  int16_t minOut = -1000;
  int16_t maxOut = 1000;

  // int32_t prev_er = 0;
  // int32_t integral = 0;
  int32_t constr[2] = {100, 1000};
  // (вход, установка, п, и, д, период в секундах, мин.выход, макс. выход)
  int16_t pid(int16_t input, int16_t setpoint, bool is_reset)
  {
    int32_t err = setpoint - input;
    static int32_t integral = 0, prevErr = 0;
    integral = constrain(integral + (int32_t)err * dt * ki / 1000, minOut, maxOut);
    int32_t D = (err - prevErr) / dt / 1000;
    prevErr = err;

    if (is_reset)
    {
      integral = 0;
      prevErr = 0;
      D = 0;
      err = 0;
      return 0;
    }

    return constrain(err * kp + integral + D * kd, minOut, maxOut);
  }
};
Pid pid_left;
Pid pid_right;
Pid pid_gen;

struct MG_996_R_360
{
  int16_t const dead_zone = 21; // +- relative to 90
  int16_t const min_v = 0;      // >=0 <90    55 is good
  int16_t const stop_v = 90;    //
  int16_t const max_v = 180;    // >90 <=180   125 is good
  int16_t const min_prd = 600;
  int16_t const stop_prd = 1500;
  int16_t const max_prd = 2400;
};
MG_996_R_360 mg996;

const uint8_t WHEEL_NUM = 2;
struct Wheel
{
  Servo servo[WHEEL_NUM];
  bool const is_direct[WHEEL_NUM] = {true, false};
  int16_t const min_spd = -1000;
  int16_t const max_spd = 1000;
  int16_t abstr_spd[WHEEL_NUM] = {0, 0};
};
Wheel wheel;

struct Platform
{
  int16_t loc_init_ang[3] = {0, 0, 0}; // x y z
  int16_t target_type = 0;
  int16_t target_val = 0;
  // bool is_done_move = false;
  uint32_t tmr[5] = {0, 0, 0, 0, 0};
  uint32_t prd[5] = {100, 100, 750, 3600000, 3600000};
  int16_t stop[WHEEL_NUM] = {0, 0};
  int16_t forw[WHEEL_NUM] = {int16_t(float(wheel.max_spd) * 0.005), int16_t(float(-wheel.max_spd) * 0.005)};
  int16_t backw[WHEEL_NUM] = {int16_t(float(wheel.min_spd) * 0.005), int16_t(float(-wheel.min_spd) * 0.005)};
  int8_t direc = -1;
  int16_t cur_speed_pid = 0;
  int16_t ang_err = 0;
  int16_t add_speed = 0;
  int16_t status = -1;
};
Platform plat;

#if (!IS_TEST_UART)
RF24 radio(pin.CE, pin.CSN);                                                // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; // возможные номера труб
byte pipeNo = 1;

ServoDriverSmooth arm_servo[4](0x40);

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float ypr[3];
uint8_t fifoBuffer[45];

uint32_t check_time_wh = 0;
uint32_t prev_check_time_wh = 0;

// ###############3

void nrf_set();
void mpu_set();

void get_imu();
void get_mltx();
void get_lidar();
void get_odo();
void tr_nrf();
void rc_nrf();
#endif
void tx_uart();
void rx_uart();

float middle_of_3(float *a, float *b, float *c);
void set_buzz(uint8_t mode);

int8_t to_int8(uint8_t val);
int16_t to_int16(uint8_t val_1, uint8_t val_2, uint8_t *val_i);
uint8_t from_int8(int8_t val);
void from_int16(int16_t val, uint8_t *int_buff);
uint8_t hash(uint8_t *data, uint32_t start_i, uint32_t end_i);
bool check_data(uint8_t *data_rec, uint32_t start_i, uint32_t end_i);

void update_control_data();
void send_buff(uint8_t *buff, uint8_t size);
void fill_tx_arr();
void buff_to_tx_buff(uint8_t *ind, uint8_t *int_buff);

void set_PWM_wheel(int16_t left_sp, int16_t right_sp);
void set_directly_wheel(int16_t left_val, int16_t right_val);
int16_t convertSpeedToVal(int16_t speed_);
int16_t add_speed(int16_t err);
void setup_wh();
/*
 *  не регаируем на поток байтов, пока не увидим #
 *  после этого всё приходящее кладём в rx_buffer
 *  как заполнится - чекаем хэш-сумму
 *  если всё ок - принимаем, парсим
 *
 *  если во время заполнения буфера снова увидели #, то начинаем всё заново
 */

void setup()
{
  /* ЗАКОММЕНТИ!! ПИШЕМ ЭТО ОДИН РАЗ ДЛЯ ОТЛДАКИ!!!*/
  // rx.arm_q1 = 110;
  // rx.arm_q2 = 100;
  // rx.arm_q3 = 60;
  // rx.arm_mode = 60;
  // EEPROM.put(EEP_ADDR, rx);
  // №№№№№№№№№№№№№№№№№№№№№№

  EEPROM.get(EEP_ADDR, rx); // прочитать из адреса - явно и верно задаем наш первый "прочитыннй" пакет
  EEPROM.get(PLAT_STAT_ADR, plat.status);

  if (MODE < 2)
  {
    Serial.begin(1000000);
  }
  else
  {
    Serial.begin(115200);
  }
  Serial.setTimeout(10);
  Serial.print("Freq clock is ");
  Serial.print(F_CPU);
  Serial.println(" Hz");
#if (!IS_TEST_UART)
  nrf_set();
  mpu_set();
#endif
  for (uint8_t i = 0; i < CTRL_MLTX; i++)
  {
    pinMode(pin.mltx.s_ctrl[i], OUTPUT);
  }

  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(pin.buzz, OUTPUT);

  wheel.servo[0].attach(pin.left_wh, mg996.min_prd, mg996.max_prd);
  wheel.servo[1].attach(pin.right_wh, mg996.min_prd, mg996.max_prd);
  for (int i = 0; i < WHEEL_NUM; i++)
  {
    wheel.servo[i].write(90);
  }
  pin.arm.servo[0] = pin.arm.base;
  pin.arm.servo[1] = pin.arm.first;
  pin.arm.servo[2] = pin.arm.second;
  pin.arm.servo[3] = pin.arm.gripper;

  for (uint8_t i = 0; i < num.arm; i++)
  {
    // arm_servo[i].attach(pin.arm.servo[i], 600, 2400); нельзя, виснет!!!!!!!!!!!!! (хер знает почему блтб)
    arm_servo[i].attach(pin.arm.servo[i]);
    // arm_servo[i].write(buff.rx[i * 2 + 3]);
    //  arm_servo[i].smoothStart(); нельзя!!!! ему пизда!!
    arm_servo[i].setAutoDetach(false);
    arm_servo[i].setSpeed(120);
    arm_servo[i].setAccel(0.3);
  }
  arm_servo[0].write(rx.arm_q1);
  arm_servo[1].write(rx.arm_q2);
  arm_servo[2].write(rx.arm_q3);
  arm_servo[3].write(rx.arm_mode);

  buff.tx[0] = uint8_t(tx.start_sb);

  pid_left.kp = 0;
  pid_left.ki = 0;
  pid_left.kd = 0;

  pid_right.kp = 0;
  pid_right.ki = 0;
  pid_right.kd = 0;

  pid_gen.kp = 20;
  pid_gen.ki = 100;
  pid_gen.kd = 1000;
  pid_gen.minOut = 100;
  pid_gen.maxOut = 1000;
  pid_gen.dt = 20;

  set_buzz(1);
  delay(100);
  set_buzz(0);
  for (uint8_t i = 0; i < 17; i++)
  {
    digitalWrite(2, 1);
    delay(800);
    digitalWrite(2, 0);
    delay(200);
  }
  set_buzz(2);
  Watchdog.enable(INTERRUPT_RESET_MODE, WDT_PRESCALER_128); // Комбинированный режим , таймаут ~1c
}

// ISR (USART_RX_vect){
//   Serial.println("smth");
// }

void loop()
{
  for (uint8_t i = 0; i < num.arm; i++)
  {
    arm_servo[i].tick();
  }
  Watchdog.reset();

  // if (millis() - tmr.main > PRD.main)
  // {
  //   tmr.main = millis();

  if (MODE == 0)
  {
#if (!IS_TEST_UART)
    if (millis() - tmr.nrf_r > PRD.nrf_r)
    {
      tmr.nrf_r = millis();
      rc_nrf();
    }
#endif
    // ctrl by nrf
  }
  else if (MODE > 0)
  {
#if (!IS_TEST_UART)
    /// опрос всего

    // if (millis() - tmr.check_imu > PRD.check_imu)
    // {
    //   tmr.check_imu = millis();
    //   get_imu();
    // }

    if (millis() - tmr.check_mltx > PRD.check_mltx)
    {
      tmr.check_mltx = millis();
      get_mltx();
    }

    // if (millis() - tmr.check_lidar > PRD.check_lidar)
    // {
    //   tmr.check_lidar = millis();
    // }

    // if (millis() - tmr.check_odo > PRD.check_odo)
    // {
    //   tmr.check_odo = millis();
    // }
#endif
    // отправка сборанной инфы
    if (millis() - tmr.tx > PRD.tx)
    {
      tmr.tx = millis();
      // Serial.println("TX");
      fill_tx_arr(); // заполнение массива на отправку собранными данными
      tx_uart();     // отправка
    }
    // приём, чек, парсинг и устанвока упарвляющей инфы
    if (millis() - tmr.rx > PRD.rx && MODE != 2)
    {
      tmr.rx = millis();
      rx_uart(); // так вышло, что тут всё, - приняли и уставки сразу актуальные, если прошло проверку
    }

    // устанвока колёс
    if (millis() - tmr.set_wheel > PRD.set_wheel)
    {
      tmr.set_wheel = millis();
      get_imu();
      if (tx.mode_move != 0)
      {
        digitalWrite(3, 0);
        // если зaвершили предыдущее движение, то делаем иниты для движения
        plat.target_type = constrain(rx.move_type, 0, 6);
        plat.loc_init_ang[2] = tx.ang_z;
        rx.move_type = 0;
        if (rx.val_move > 0)
        {
          if (plat.target_type != 3)
          {
            plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17 - 100;
          }
          else
          {
            plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17 - 210;
          }
          plat.direc = 1;
        }
        else
        {
          if (plat.target_type != 3)
          {
            plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17 + 100;
          }
          else
          {
            plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17 + 210;
          }
          plat.direc = 0;
        }
        // *****************************************************
        // if (rx.val_move > 0)
        // {
        //   plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17 - 100;
        //   plat.direc = 1;
        // }
        // else
        // {
        //   plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17 + 100;
        //   plat.direc = 0;
        // }
        // plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17; // 17.453  Ded to Mrad
        if (plat.target_val < 0)
        {
          plat.target_val = 6283 + plat.target_val;
        }
        else if (plat.target_val > 6283)
        {
          plat.target_val = plat.target_val - 6283;
        }
        // plat.direc = rx.val_move > 0 ? 1 : 0;
        // rx.val_move = 0;

        for (uint8_t i = 0; i < 5; i++)
        {
          plat.tmr[i] = millis();
        }
        // plat.is_done_move = false;
        tx.mode_move = 0;
      }
      if (tx.mode_move == 0) // а если не завершили, то делаем движение, че ждём-то  // else
      {
        digitalWrite(3, 1);
        digitalWrite(2, !plat.target_type);
        switch (plat.target_type)
        {
        case 0: // stop
          //set_PWM_wheel(plat.stop[0], plat.stop[1]);
          tx.mode_move = 1;
          // if (millis() - plat.tmr[plat.target_type] > plat.prd[plat.target_type])
          // {
          //   // plat.is_done_move = true;
          //   tx.mode_move = 1;
          //   set_PWM_wheel(plat.stop[0], plat.stop[1]);
          // }
          break;
        case 1: // прямо по углу z
          // set_PWM_wheel(plat.forw[0], plat.forw[1]);
          // set_directly_wheel(mg996.stop_v + mg996.dead_zone, mg996.stop_v - mg996.dead_zone);

          /*calc PID, then Set*/
          // pid_left.minOut = 50;
          // pid_left.maxOut = wheel.max_spd;
          // pid_right.minOut = -50;
          // pid_right.maxOut = wheel.min_spd;

          // pid_left.pid(tx.ang_z, plat.loc_init_ang[2], false);
          // pid_right.pid(tx.ang_z, plat.loc_init_ang[2], false);

          // pid_gen.dt = 20;
          // plat.cur_speed_pid = pid_gen.pid(tx.ang_z, plat.loc_init_ang[2], false);
          // set_PWM_wheel(plat.cur_speed_pid, 0 - plat.cur_speed_pid);

          plat.ang_err = plat.loc_init_ang[2] - tx.ang_z;
          plat.add_speed = add_speed(plat.ang_err);
          if (plat.ang_err < 0) // ускоряем правое!!
          {
            set_PWM_wheel(plat.forw[0], plat.forw[1] - plat.add_speed);
          }
          else // усореем левое!!
          {
            set_PWM_wheel(plat.forw[0] + plat.add_speed, plat.forw[1]);
          }

          if (millis() - plat.tmr[plat.target_type] > plat.prd[plat.target_type] * rx.val_move)
          {
            // plat.is_done_move = true;
            tx.mode_move = 1;
            set_PWM_wheel(plat.stop[0], plat.stop[1]);
            // pid_gen.pid(tx.ang_z, plat.loc_init_ang[2], true);
            plat.status = plat.target_type + 10;
          }
          break;
        case 2: // назад по углу z
                // set_PWM_wheel(plat.backw[0], plat.backw[1]);
                // set_directly_wheel(mg996.stop_v - mg996.dead_zone, mg996.stop_v + mg996.dead_zone);

          /*calc PID, then Set*/
          // plat.cur_speed_pid = pid_gen.pid(tx.ang_z, plat.loc_init_ang[2], false);
          // set_PWM_wheel(0 - plat.cur_speed_pid, plat.cur_speed_pid);

          plat.ang_err = plat.loc_init_ang[2] - tx.ang_z;
          plat.add_speed = add_speed(plat.ang_err);
          if (plat.ang_err < 0) // ускоряем левое!!!
          {
            set_PWM_wheel(plat.backw[0] - plat.add_speed, plat.backw[1]);
          }
          else // усореем правое!!!!
          {
            set_PWM_wheel(plat.backw[0], plat.backw[1] + plat.add_speed);
          }

          if (millis() - plat.tmr[plat.target_type] > plat.prd[plat.target_type] * rx.val_move)
          {
            // plat.is_done_move = true;
            tx.mode_move = 1;
            set_PWM_wheel(plat.stop[0], plat.stop[1]);
            // pid_gen.pid(tx.ang_z, plat.loc_init_ang[2], true);
            plat.status = plat.target_type + 10;
          }
          break;
        case 6: // вперёд по дефолтам
          set_directly_wheel(mg996.stop_v + mg996.dead_zone, mg996.stop_v - mg996.dead_zone);
          if (millis() - plat.tmr[plat.target_type] > plat.prd[plat.target_type] * rx.val_move)
          {
            // plat.is_done_move = true;
            tx.mode_move = 1;
            set_PWM_wheel(plat.stop[0], plat.stop[1]);
            // pid_gen.pid(tx.ang_z, plat.loc_init_ang[2], true);
            plat.status = plat.target_type + 10;
          }
          break;
        case 3: // вращение вокруг центра оси  ang>=0 - gj  час.  ang<0 - ghjn час.
          if (plat.direc == 0)
          {
            // set_PWM_wheel(plat.backw[0], plat.forw[1]);
            set_directly_wheel(mg996.stop_v - mg996.dead_zone, mg996.stop_v - mg996.dead_zone);
            if (tx.ang_z <= plat.target_val)
            {
              // plat.is_done_move = true;
              tx.mode_move = 1;
              set_PWM_wheel(plat.stop[0], plat.stop[1]);
              plat.status = plat.target_type + 10;
            }
          }
          else if (plat.direc == 1)
          {
            // set_PWM_wheel(plat.forw[0], plat.backw[1]);
            set_directly_wheel(mg996.stop_v + mg996.dead_zone, mg996.stop_v + mg996.dead_zone);
            if (tx.ang_z >= plat.target_val)
            {
              // plat.is_done_move = true;
              tx.mode_move = 1;
              set_PWM_wheel(plat.stop[0], plat.stop[1]);
              plat.status = plat.target_type + 10;
            }
          }
          break;
        case 4: // вращение вокруг ЛЕВ колеса ang>=0 - против час.  ang<0 - по час.
          if (plat.direc == 0)
          {
            // set_PWM_wheel(plat.stop[0], plat.forw[1]);
            set_directly_wheel(mg996.stop_v, mg996.stop_v - mg996.dead_zone);
            if (tx.ang_z <= plat.target_val)
            {
              // plat.is_done_move = true;
              tx.mode_move = 1;
              set_PWM_wheel(plat.stop[0], plat.stop[1]);
              plat.status = plat.target_type + 10;
            }
          }
          else if (plat.direc == 1)
          {
            // set_PWM_wheel(plat.forw[0], plat.stop[1]);
            set_directly_wheel(mg996.stop_v, mg996.stop_v + mg996.dead_zone);
            if (tx.ang_z >= plat.target_val)
            {
              // plat.is_done_move = true;
              tx.mode_move = 1;
              set_PWM_wheel(plat.stop[0], plat.stop[1]);
              plat.status = plat.target_type + 10;
            }
          }
          break;
        case 5: // вращение вокруг ПРАВ колеса ang>=0 - против час.  ang<0 - по час.
          if (plat.direc == 0)
          {
            // set_PWM_wheel(plat.stop[0], plat.forw[1]);
            set_directly_wheel(mg996.stop_v - mg996.dead_zone, mg996.stop_v);
            if (tx.ang_z <= plat.target_val)
            {
              // plat.is_done_move = true;
              tx.mode_move = 1;
              set_PWM_wheel(plat.stop[0], plat.stop[1]);
              plat.status = plat.target_type + 10;
            }
          }
          else if (plat.direc == 1)
          {
            // set_PWM_wheel(plat.forw[0], plat.stop[1]);
            set_directly_wheel(mg996.stop_v + mg996.dead_zone, mg996.stop_v);
            if (tx.ang_z >= plat.target_val)
            {
              // plat.is_done_move = true;
              tx.mode_move = 1;
              set_PWM_wheel(plat.stop[0], plat.stop[1]);
              plat.status = plat.target_type + 10;
            }
          }
          break;
        default:
          // КАКАЯ_ТО ОШИБКА!!!!!!!!!!
          tx.mode_move = 9;
          break;
        }
        /*точка выхода в аждом кейсе*/
        // if (/*ang*/ (abs(tx.ang_z - plat.loc_init_ang[2]) >= abs((lat.target_val))) || /*time*/) // точка выхода из движения
        // {
        //   plat.is_done_move = true;
        //   tx.mode_move = 1;
        // }
      }
      check_time_wh = millis() - prev_check_time_wh;
      prev_check_time_wh = check_time_wh;
    }

    // уставнока манипуоятора
    if (millis() - tmr.set_arm > PRD.set_arm)
    {
      tmr.set_arm = millis();

      // arm_servo[0].write(rx.arm_q1);
      // arm_servo[1].write(rx.arm_q2);
      // arm_servo[2].write(rx.arm_q3);
      // arm_servo[3].write(rx.arm_mode);

      arm_servo[0].setTargetDeg(rx.arm_q1);
      arm_servo[1].setTargetDeg(rx.arm_q2);
      arm_servo[2].setTargetDeg(rx.arm_q3);
      arm_servo[3].setTargetDeg(rx.arm_mode);

      tx.x_arm = rx.arm_q1;
      tx.y_arm = rx.arm_q2;
      tx.z_arm = rx.arm_q3;
      tx.mode_arm = rx.arm_mode;
    }
    // уставнока периферии (магнит, аудио, ещё какая-нибудь хрень)
    if (millis() - tmr.set_periph > PRD.set_periph)
    {
      tmr.set_periph = millis();
      /**/
    }
  }
}
//}
#if (!IS_TEST_UART)
void nrf_set()
{
  if (MODE == 2)
  {
    radio.begin();                        // активировать модуль
    radio.setAutoAck(1);                  // режим подтверждения приёма, 1 вкл 0 выкл
    radio.setRetries(0, 15);              // (время между попыткой достучаться, число попыток)
    radio.enableAckPayload();             // разрешить отсылку данных в ответ на входящий сигнал
    radio.setPayloadSize(32);             // размер пакета, в байтах
    radio.openWritingPipe(address[0]);    // мы - труба 0, открываем канал для передачи данных
    radio.openReadingPipe(1, address[0]); // хотим слушать трубу 0
    radio.setChannel(0x6a);               // выбираем канал (в котором нет шумов!)
    radio.setPALevel(RF24_PA_MAX);        // уровень мощности передатчика
    radio.setDataRate(RF24_2MBPS);        // скорость обмена
    // должна быть одинакова на приёмнике и передатчике!
    // при самой низкой скорости имеем самую высокую чувствительность и дальность!!

    radio.powerUp();       // начать работу
    radio.stopListening(); // не слушаем радиоэфир, мы передатчик
  }
  else
  {
    radio.begin();            // активировать модуль
    radio.setAutoAck(1);      // режим подтверждения приёма, 1 вкл 0 выкл
    radio.setRetries(0, 15);  // (время между попыткой достучаться, число попыток)
    radio.enableAckPayload(); // разрешить отсылку данных в ответ на входящий сигнал
    radio.setPayloadSize(32); // размер пакета, в байтах

    radio.openReadingPipe(1, address[0]); // хотим слушать трубу 0
    radio.setChannel(0x6a);               // выбираем канал (в котором нет шумов!)

    radio.setPALevel(RF24_PA_MAX); // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio.setDataRate(RF24_2MBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
    radio.powerUp();               // начать работу
    radio.startListening();        // начинаем слушать эфир, мы приёмный модуль
  }
}

void mpu_set()
{
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}
#endif

void tx_uart()
{
  // Serial.println("TX");
  if (MODE == 1)
  {
    // Serial.write(tx.start_sb);
    // send_buff(buff.tx, 48);
    for (uint8_t i = 0; i < 48; i++)
    {
      Serial.write(buff.tx[i]);
    }
  }
  else if (MODE == 2)
  {
#if (!IS_TEST_UART)
    for (uint8_t i = 0; i < 16; i++)
    {
      buff.tx_add[i] = buff.tx[32 + i];
    }
    radio.write(&buff.tx, 32);
    radio.write(&buff.tx_add, 16);
    if (!radio.available(&pipeNo))
    { // если получаем пустой ответ
    }
    else
    {
      while (radio.available(&pipeNo))
      {                                // если в ответе что-то есть
        radio.read(&buff.nrf_rec, 12); // читаем
        // получили забитый данными массив telemetry ответа от приёмника
        for (uint8_t i = 0; i < 11; i++)
        {
          buff.rx[i] = buff.nrf_rec[i + 1];
        }
      }
      update_control_data();
    }
#endif
  }
}

void rx_uart()
{
  // Serial.println("RX");
  if (MODE == 1)
  {
    static uint8_t i;
    if (Serial.available())
    {
      // Serial.println();
      if (rx_flag)
      {
        buff.rx[i++] = Serial.read();
        // Serial.println(int8_t(rx_buff[i-1]));
        if (i > 10)
        {
          rx_flag = false;
          i = 0;
          // send_buff();
          update_control_data();
        }
      }
      else
      {
        if (char(Serial.read()) == rx.init_sb)
        {
          // Serial.println(rx.init_sb);
          rx_flag = true;
          i = 0;
        }
      }
    }
    else
    {
      Serial.flush();
    }
  }
  else if (MODE == 2)
  {
  }
}

void send_buff(uint8_t *buff, uint8_t size)
{
  for (uint8_t i = 0; i < size; i++)
  {
    Serial.write(buff[i]);
  }
}
void update_control_data()
{
  /*int8_t move_type = -1;
  int8_t val_move = -1;
  int16_t arm_q1 = 90;
  int16_t arm_q2 = 90;
  int16_t arm_q3 = 90;
  int8_t arm_mode = -1;
  int8_t auido_mode = -1;*/

  rx.hsum = hash(buff.rx, 1, 11);
  if (rx.hsum == buff.rx[0])
  {
    uint8_t i = 0;
    rx.move_type = to_int8(buff.rx[1]);
    rx.val_move = to_int8(buff.rx[2]);
    rx.arm_q1 = to_int16(buff.rx[3], buff.rx[4], &i);
    rx.arm_q2 = to_int16(buff.rx[5], buff.rx[6], &i);
    rx.arm_q3 = to_int16(buff.rx[7], buff.rx[8], &i);
    i = 8;
    rx.arm_mode = to_int8(buff.rx[9]);
    rx.auido_mode = to_int8(buff.rx[10]);
  }
  else
  {
    uint8_t i = 0;
    rx.move_type = to_int8(buff.rx[1]);
    rx.val_move = to_int8(buff.rx[2]);
    rx.arm_q1 = to_int16(buff.rx[3], buff.rx[4], &i);
    rx.arm_q2 = to_int16(buff.rx[5], buff.rx[6], &i);
    rx.arm_q3 = to_int16(buff.rx[7], buff.rx[8], &i);
    rx.arm_mode = to_int8(buff.rx[9]);
    rx.auido_mode = to_int8(buff.rx[10]);
  }
}

#if (!IS_TEST_UART)
void rc_nrf()
{
  if (millis() - tmr.check_nrf > PRD.check_nrf)
  {
    rec_nrf.x1 = 512;
    rec_nrf.y1 = 512;
    rec_nrf.x2 = 512;
    rec_nrf.y2 = 512;
    rec_nrf.btn1 = 0;
    rec_nrf.btn2 = 0;
    // потеряли пульт, стоим
  }
  else if (radio.available(&rec_nrf.pipeNo))
  { // слушаем эфир со всех труб
    tmr.check_nrf = millis();
    radio.read(&rec_nrf.data, sizeof(rec_nrf.data)); // чиатем входящий сигнал
    rec_nrf.x1 = rec_nrf.data[0];
    rec_nrf.y1 = rec_nrf.data[1];
    rec_nrf.x2 = rec_nrf.data[2];
    rec_nrf.y2 = rec_nrf.data[3];
    rec_nrf.btn1 = rec_nrf.data[4];
    rec_nrf.btn2 = rec_nrf.data[5];
  }
}

void tr_nrf()
{
}

void get_imu()
{
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer))
  {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // tx.ang_x = degrees(ypr[2]) * 1000;
    // tx.ang_y = degrees(ypr[1]) * 1000;
    // tx.ang_z = degrees(ypr[0]) * 1000;
    tx.ang_x = (ypr[2] + M_PI) * 1000;
    tx.ang_y = (ypr[1] + M_PI) * 1000;
    tx.ang_z = (ypr[0] + M_PI) * 1000;
    // tx.ang_x = int16_t(ypr[2] + M_PI) << 10;
    // tx.ang_y = int16_t(ypr[1] + M_PI) << 10;
    // tx.ang_z = int16_t(ypr[0] + M_PI) << 10;
  }
}
void set_mltx(uint8_t *mode, uint8_t *val_map)
{
  switch (*mode)
  {
  case 0:
    pinMode(pin.mltx.sig, INPUT_PULLUP);
    break;
  case 1:
    pinMode(pin.mltx.sig, INPUT);
    break;
  case 2:
    pinMode(pin.mltx.sig, OUTPUT);
    break;
  }
  for (uint8_t i = 0; i < CTRL_MLTX; i++)
  {
    digitalWrite(pin.mltx.s_ctrl[i], val_map[i]);
  }
}

void get_mltx()
{
  tx.ir = 0b00000000;
  for (int8_t i = 0; i < NUM_IR; i++)
  {
    set_mltx(&pin.mltx.pin_mode, pin.mltx.ir[i]);
    tx.ir = tx.ir | (digitalRead(pin.mltx.sig) << i);
  }
  tx.end_sens = 0b00000000;
  for (int8_t i = 0; i < NUM_END; i++)
  {
    set_mltx(&pin.mltx.pin_mode, pin.mltx.end_sens[i]);
    tx.end_sens = tx.end_sens | (digitalRead(pin.mltx.sig) << i);
  }
}
#endif
float middle_of_3(float *a, float *b, float *c)
{
  if ((*a <= *b) && (*a <= *c))
  {
    return (*b <= *c) ? *b : *c;
  }
  else
  {
    if ((*b <= *a) && (*b <= *c))
    {
      return (*a <= *c) ? *a : *c;
    }
    else
    {
      return (*a <= *b) ? *a : *b;
    }
  }
}

int8_t to_int8(uint8_t val)
{
  return int8_t(val);
}

int16_t to_int16(uint8_t val_1, uint8_t val_2, uint8_t *val_i)
{
  // inc(&(*val_i));
  return int16_t((val_2 << 8) | val_1);
}

uint8_t from_int8(int8_t val)
{
  return uint8_t(val);
}

void from_int16(int16_t val, uint8_t *int_buff)
{
  int_buff[0] = uint8_t(val);
  int_buff[1] = uint8_t(val >> 8);
}
void buff_to_tx_buff(uint8_t *ind, uint8_t *int_buff)
{
  buff.tx[(*ind)++] = int_buff[0];
  buff.tx[(*ind)++] = int_buff[1];
}

uint8_t inc(uint8_t *val_i)
{
  (*val_i)++;
  return *val_i;
}

uint8_t hash(uint8_t *data, uint32_t start_i, uint32_t end_i)
{
  uint8_t ch_sum = 0;

  for (uint8_t i = start_i; i < end_i; i++)
  {
    ch_sum = (ch_sum << 3) | data[i];
    ch_sum = (ch_sum << 4) | data[i]; // ~ ch_sum*128 + 17*byte
                                      // ch_sum = (ch_sum >> 2) | arr[i]; // ~ ch_sum*64 +9.5*byte
                                      // printf("%d\r\n", ch_sum);
  }
  return ch_sum;
}

bool check_data(uint8_t *data_rec, uint32_t start_i, uint32_t end_i)
{
  uint8_t rec_hash = hash(data_rec, start_i, end_i);
  return (!(data_rec[0] == rec_hash)); // 0 - хэши совпали
}

void fill_tx_arr()
{
  uint8_t i = 2;
  //  int16_t left_wh = 5;
  //  int16_t right_wh = -1;
  //  int16_t mode_move = -1;
  //  int16_t x_arm = 256;
  //  int16_t y_arm = -1;
  //  int16_t z_arm = -1;
  //  int16_t mode_arm = -1;
  //  int16_t ax = -12345;
  //  int16_t ay = -12345;
  //  int16_t az = -12345;
  //  int16_t gx = -12345;
  //  int16_t gy = -12345;
  //  int16_t gz = -12345;
  //  int16_t ang_x = -12345;
  //  int16_t ang_y = -12345;
  //  int16_t ang_z = -12345;
  //  int16_t odo_l = -1;
  //  int16_t odo_r = -1;
  //  int16_t lidar_angle = -1;
  //  int16_t lidar_dist = -1;
  //  int16_t sonar_1 = -1;
  //  int16_t sonar_2 = -1;
  //  int8_t ir = 0b00000000;       // 0b00000011
  //  int8_t end_sens = 0b00000000; // 0b00001111

  // move
  from_int16(tx.left_wh, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.right_wh, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.mode_move, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  // arm
  from_int16(tx.x_arm, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.y_arm, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.z_arm, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.mode_arm, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  // from_int16(rx.arm_q1, buff.two_bytes);
  // buff_to_tx_buff(&i, buff.two_bytes);
  // from_int16(rx.arm_q2, buff.two_bytes);
  // buff_to_tx_buff(&i, buff.two_bytes);
  // from_int16(rx.arm_q2, buff.two_bytes);
  // buff_to_tx_buff(&i, buff.two_bytes);
  // from_int16(rx.arm_mode, buff.two_bytes);
  // buff_to_tx_buff(&i, buff.two_bytes);
  // accel
  from_int16(tx.ax, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.ay, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.az, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  // gyro
  from_int16(tx.gx, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.gy, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.gz, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  // ang
  from_int16(tx.ang_x, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.ang_y, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.ang_z, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  // odo
  from_int16(tx.odo_l, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.odo_r, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  // lidar
  from_int16(tx.lidar_angle, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  from_int16(tx.lidar_dist, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  // sonar
  from_int16(plat.status, buff.two_bytes);
  buff_to_tx_buff(&i, buff.two_bytes);
  // from_int16(tx.sonar_2, buff.two_bytes);   //check_time_wh
  from_int16(int16_t(check_time_wh), buff.two_bytes); // check_time_wh
  buff_to_tx_buff(&i, buff.two_bytes);
  // ик и концевики
  buff.tx[i++] = from_int8(tx.ir);
  buff.tx[i++] = from_int8(tx.end_sens);
  // hash sum
  // buff.tx[0] = hash(buff.tx, 1, 47);
  buff.tx[1] = tx.hsum;
}
// ####################### for robot #######
void set_buzz(uint8_t mode)
{
  switch (mode)
  {
  case 0:
    digitalWrite(pin.buzz, 1);
    break;
  case 1:
    digitalWrite(pin.buzz, 0);
    break;
  case 2:
    digitalWrite(pin.buzz, 0);
    delay(50);
    digitalWrite(pin.buzz, 1);
    delay(100);
    digitalWrite(pin.buzz, 0);
    delay(50);
    digitalWrite(pin.buzz, 1);
    break;
  }
}
void setup_wh()
{
  get_imu();
  if (tx.mode_move != 0)
  {
    digitalWrite(3, 0);
    // если зaвершили предыдущее движение, то делаем иниты для движения
    // plat.target_type = constrain(rx.move_type, 0, 5);
    plat.loc_init_ang[2] = tx.ang_z;
    rx.move_type = 0;
    if (rx.val_move > 0)
    {
      plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17 - 190;
      plat.direc = 1;
    }
    else
    {
      plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17 + 190;
      plat.direc = 0;
    }
    // plat.target_val = plat.loc_init_ang[2] + rx.val_move * 17; // 17.453  Ded to Mrad
    if (plat.target_val < 0)
    {
      plat.target_val = 6283 + plat.target_val;
    }
    else if (plat.target_val > 6283)
    {
      plat.target_val = plat.target_val - 6283;
    }
    // plat.direc = rx.val_move > 0 ? 1 : 0;
    // rx.val_move = 0;

    for (uint8_t i = 0; i < 5; i++)
    {
      plat.tmr[i] = millis();
    }
    // plat.is_done_move = false;
    tx.mode_move = 0;

    digitalWrite(3, 1);
    digitalWrite(2, !plat.target_type);
    switch (plat.target_type)
    {
    case 0: // stop
      set_PWM_wheel(plat.stop[0], plat.stop[1]);
      break;
    case 1: // прямо по углу z
      // set_PWM_wheel(plat.forw[0], plat.forw[1]);
      set_directly_wheel(mg996.stop_v + mg996.dead_zone, mg996.stop_v - mg996.dead_zone);
      break;
    case 2: // назад по углу z
      // set_PWM_wheel(plat.backw[0], plat.backw[1]);
      set_directly_wheel(mg996.stop_v - mg996.dead_zone, mg996.stop_v + mg996.dead_zone);
      break;
    case 3: // вращение вокруг центра оси  ang>=0 - gj  час.  ang<0 - ghjn час.
      if (plat.direc == 0)
      {
        // set_PWM_wheel(plat.backw[0], plat.forw[1]);
        set_directly_wheel(mg996.stop_v - mg996.dead_zone, mg996.stop_v - mg996.dead_zone);
      }
      else if (plat.direc == 1)
      {
        // set_PWM_wheel(plat.forw[0], plat.backw[1]);
        set_directly_wheel(mg996.stop_v + mg996.dead_zone, mg996.stop_v + mg996.dead_zone);
      }
      break;
    case 4: // вращение вокруг ЛЕВ колеса ang>=0 - против час.  ang<0 - по час.
      if (plat.direc == 0)
      {
        // set_PWM_wheel(plat.stop[0], plat.forw[1]);
        set_directly_wheel(mg996.stop_v, mg996.stop_v - mg996.dead_zone);
      }
      else if (plat.direc == 1)
      {
        // set_PWM_wheel(plat.forw[0], plat.stop[1]);
        set_directly_wheel(mg996.stop_v, mg996.stop_v + mg996.dead_zone);
      }
      break;
    case 5: // вращение вокруг ПРАВ колеса ang>=0 - против час.  ang<0 - по час.
      if (plat.direc == 0)
      {
        // set_PWM_wheel(plat.stop[0], plat.forw[1]);
        set_directly_wheel(mg996.stop_v - mg996.dead_zone, mg996.stop_v);
      }
      else if (plat.direc == 1)
      {
        // set_PWM_wheel(plat.forw[0], plat.stop[1]);
        set_directly_wheel(mg996.stop_v + mg996.dead_zone, mg996.stop_v);
      }
      break;
    default:
      // КАКАЯ_ТО ОШИБКА!!!!!!!!!!
      tx.mode_move = 9;
      break;
    }
    /*точка выхода в аждом кейсе*/
    // if (/*ang*/ (abs(tx.ang_z - plat.loc_init_ang[2]) >= abs((lat.target_val))) || /*time*/) // точка выхода из движения
    // {
    //   plat.is_done_move = true;
    //   tx.mode_move = 1;
    // }
  }
  else if (tx.mode_move == 0) // а если не завершили, то ничего не меняем!! движение, че ждём-то
  {
    switch (plat.target_type)
    {
    case 0: // stop
      if (millis() - plat.tmr[plat.target_type] > plat.prd[plat.target_type])
      {
        // plat.is_done_move = true;
        tx.mode_move = 1;
        set_PWM_wheel(plat.stop[0], plat.stop[1]);
      }
      break;
    case 1: // прямо по углу z
      if (millis() - plat.tmr[plat.target_type] > plat.prd[plat.target_type])
      {
        // plat.is_done_move = true;
        tx.mode_move = 1;
        set_PWM_wheel(plat.stop[0], plat.stop[1]);
      }
      break;
    case 2: // назад по углу z
      if (millis() - plat.tmr[plat.target_type] > plat.prd[plat.target_type])
      {
        // plat.is_done_move = true;
        tx.mode_move = 1;
        set_PWM_wheel(plat.stop[0], plat.stop[1]);
      }
      break;
    case 3: // вращение вокруг центра оси  ang>=0 - gj  час.  ang<0 - ghjn час.
      if (plat.direc == 0)
      {
        if (tx.ang_z <= plat.target_val)
        {
          // plat.is_done_move = true;
          tx.mode_move = 1;
          set_PWM_wheel(plat.stop[0], plat.stop[1]);
        }
      }
      else if (plat.direc == 1)
      {
        if (tx.ang_z >= plat.target_val)
        {
          // plat.is_done_move = true;
          tx.mode_move = 1;
          set_PWM_wheel(plat.stop[0], plat.stop[1]);
        }
      }
      break;
    case 4: // вращение вокруг ЛЕВ колеса ang>=0 - против час.  ang<0 - по час.
      if (plat.direc == 0)
      {
        if (tx.ang_z <= plat.target_val)
        {
          // plat.is_done_move = true;
          tx.mode_move = 1;
          set_PWM_wheel(plat.stop[0], plat.stop[1]);
        }
      }
      else if (plat.direc == 1)
      {
        if (tx.ang_z >= plat.target_val)
        {
          // plat.is_done_move = true;
          tx.mode_move = 1;
          set_PWM_wheel(plat.stop[0], plat.stop[1]);
        }
      }
      break;
    case 5: // вращение вокруг ПРАВ колеса ang>=0 - против час.  ang<0 - по час.
      if (plat.direc == 0)
      {
        if (tx.ang_z <= plat.target_val)
        {
          // plat.is_done_move = true;
          tx.mode_move = 1;
          set_PWM_wheel(plat.stop[0], plat.stop[1]);
        }
      }
      else if (plat.direc == 1)
      {
        if (tx.ang_z >= plat.target_val)
        {
          // plat.is_done_move = true;
          tx.mode_move = 1;
          set_PWM_wheel(plat.stop[0], plat.stop[1]);
        }
      }
      break;
    default:
      // КАКАЯ_ТО ОШИБКА!!!!!!!!!!
      tx.mode_move = 9;
      break;
    }
  }
}

void set_PWM_wheel(int16_t left_sp, int16_t right_sp) // принимает абстрактную уставку от -1000 до 1000
{
  tx.left_wh = convertSpeedToVal(left_sp);
  tx.right_wh = convertSpeedToVal(right_sp);
  wheel.servo[0].write(tx.left_wh);
  wheel.servo[1].write(tx.right_wh);
}
void set_directly_wheel(int16_t left_val, int16_t right_val)
{
  tx.left_wh = left_val;
  tx.right_wh = right_val;
  wheel.servo[0].write(tx.left_wh);
  wheel.servo[1].write(tx.right_wh);
}

int16_t convertSpeedToVal(int16_t speed_)
{
  speed_ = constrain(speed_, wheel.min_spd, wheel.max_spd);
  int16_t val = 90;
  if (speed_ < 0)
  {
    val = map(speed_, wheel.min_spd, 1, mg996.min_v, mg996.stop_v - mg996.dead_zone);
  }
  else if (speed_ > 0)
  {
    val = map(speed_, 1, wheel.max_spd, mg996.stop_v + mg996.dead_zone, mg996.max_v);
  }
  return val;
}

int16_t add_speed(int16_t err) // этой функцией только генерим ускорилку для колеса!!!!!!!!!!!
{                              // ошибк - разность  из цели и курента (цель - куррент)
  int16_t add_value = 0;
  if (err < 0) // значит ускоряем првое!!!!, но здесь это не важно!!!
  {
    err = 0 - err;
  }
  add_value = 3 * err;
  if (add_value > 1000)
  {
    add_value = 1000;
  }
  return add_value;
}

// /* Первый тайм-аут вызывает прерывание */
ISR(WATCHDOG)
{
  set_buzz(1);
  EEPROM.put(EEP_ADDR, rx);
  EEPROM.put(PLAT_STAT_ADR, plat.status);
  // Serial.println("warning!");
  // Если исправить причину не вышло - следующий таймаут вызывает сброс
  // Watchdog.enable(INTERRUPT_RESET_MODE, WDT_PRESCALER_128); // Если перенастроить watchdog здесь - сброса не будет
}
