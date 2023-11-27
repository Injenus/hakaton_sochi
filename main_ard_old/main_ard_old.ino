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
   %<mode_left_wh>,<mode_right_wh>,<mode_move>,<x>,<y>,<z>,<grip>,<...9 mpu data>,<odo_l>,<odo_r>,<IR_left>,<IR_right>,<IR_3>,<sw1>,<sw2>,<sw3>,<sw4>,<lidar_angle>,<lidar_dist>,<sonar_1>,<sonar_2>;/n = 62 bytes

  Сначала шлёт МК, потом (по принятию) шлёт ПК
*/

#define IS_TEST_UART 0

#if (!IS_TEST_UART)
//#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Adafruit_VL53L0X.h>
#include <ServoDriverSmooth.h>
#endif
//#include <avr/io.h>
//#include <avr/interrupt.h>

#define CTRL_MLTX 3
#define NUM_IR 2
#define NUM_END 4
#define DATA_NRF 6

struct Timer {
  uint32_t main = 0;
  uint32_t tx = 0;
  uint32_t rx = 0;
  uint32_t nrf_t = 0;
  uint32_t nrf_r = 0;
  uint32_t set_wheel = 0;
  uint32_t set_arm = 0;
  uint32_t check_mltx = 0;
  uint32_t check_lidar = 0;
  uint32_t check_imu = 0;
  uint32_t check_odo = 0;
  uint32_t check_nrf = 0;
}; Timer tmr;

struct Period {
  const uint32_t main = 0;
  const uint32_t tx = 500;
  const uint32_t rx = 0;
  const uint32_t nrf_t = 5;
  const uint32_t nrf_r = 5;
  const uint32_t set_wheel = 5;
  const uint32_t set_arm = 5;
  const uint32_t check_mltx = 5;
  const uint32_t check_lidar = 5;
  const uint32_t check_imu = 14;
  const uint32_t check_odo = 5;
  const uint32_t check_nrf = 200;
}; Period PRD;


struct Num {
  const uint8_t arm = 4;
  const uint8_t ir = 2;
  const int end_sens = 4;
  const int mltx_ctrl = 3;
}; Num num;

struct Arm {
  const uint8_t base = 0;
  const uint8_t first = 1;
  const uint8_t second = 2;
  const uint8_t gripper = 3;
};

struct Multiplexor {
  const uint8_t s_ctrl[CTRL_MLTX] = {15, 16, 17}; //A1 A2 A3  //num.mltx_ctrl
  const uint8_t sig = 14; // A0
  const uint8_t ir[NUM_IR][CTRL_MLTX] = {  //num.ir  //num.mltx_ctrl
    {0, 0, 0},
    {1, 0, 0}
  };
  const uint8_t end_sens[NUM_END][CTRL_MLTX] = { //num.end_sens  //num.mltx_ctrl
    {0, 1, 0},
    {1, 1, 0},
    {0, 0, 1},
    {1, 0, 1}
  };
};

struct Pin { // reserved A4-A5(18-19)(I2C), 11-13(SPI), 0-1(UART), 2-3(extrn. interr.)
  const uint8_t left_wh = 9;
  const uint8_t right_wh = 10;
  const uint8_t CE = 7;
  const uint8_t CSN = 8;
  const uint8_t lidar_servo = 5;
  const uint8_t left_odo = 20; //A6
  const uint8_t right_odo = 21; //A7
  Arm arm;
  Multiplexor mltx;
}; Pin pin;

struct Rec_nrf {
  int16_t data[DATA_NRF];
  byte pipeNo;
  int16_t x1 = 512;
  int16_t y1 = 512;
  int16_t x2 = 512;
  int16_t y2 = 512;
  uint8_t btn1 = 0;
  uint8_t btn2 = 0;
}; Rec_nrf rec_nrf;

struct Transmit {
  char start_sb = '%';
  //char end_sb = ';';
  //char terminator = ',';
  int16_t left_wh = 5;
  int16_t right_wh = -1;
  int16_t mode_move = -1;
  int16_t x_arm = 256;
  int16_t y_arm = -1;
  int16_t z_arm = -1;
  int16_t mode_arm = -1;
  int16_t ax = -12345;
  int16_t ay = -12345;
  int16_t az = -12345;
  int16_t gx = -12345;
  int16_t gy = -12345;
  int16_t gz = -12345;
  int16_t ang_x = -12345;
  int16_t ang_y = -12345;
  int16_t ang_z = -12345;
  int16_t odo_l = -1;
  int16_t odo_r = -1;
  int16_t lidar_angle = -1;
  int16_t lidar_dist = -1;
  int16_t sonar_1 = -1;
  int16_t sonar_2 = -1;
  int8_t ir[NUM_IR] = { -1, -1};
  int8_t end_sens[NUM_END] = { -1, -1, -1, -1};
}; Transmit tx;

struct Receive {
  char init_sb = '#';
  uint8_t hsum = 9;
  int8_t move_type = -1;
  int8_t val_move = -1;
  int16_t arm_q1 = 90;
  int16_t arm_q2 = 90;
  int16_t arm_q3 = 90;
  int8_t arm_mode = -1;
  int8_t auido_mode = -1;
}; Receive rx;

const uint8_t MODE = 1;

#if (!IS_TEST_UART)
RF24 radio(pin.CE, pin.CSN);  // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб

ServoDriverSmooth arm_servo[4](0x40);

MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
float ypr[3];
uint8_t fifoBuffer[45];


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

int8_t to_int8(uint8_t val);
int16_t to_int16(uint8_t val_1, uint8_t val_2);
uint8_t from_int8(int8_t val);
void from_int16(int16_t val, uint8_t *int_buff);
uint8_t hash(uint8_t *data, uint32_t start_i, uint32_t end_i);
bool check_data(uint8_t *data_rec, uint32_t start_i, uint32_t end_i);


/*
 *  не регаируем на поток байтов, пока не увидим #
 *  после этого всё приходящее кладём в rx_buffer
 *  как заполнится - чекаем хэш-сумму
 *  если всё ок - принимаем, парсим
 *  
 *  если во время заполнения буфера снова увидели #, то начинаем всё заново
*/

struct Buff{
  volatile uint8_t rx[11];
  volatile uint8_t two_bytes[2];
  volatile uint8_t tx[46]; // 22*2+2 (чисто данные)
};
Buff buff;
volatile bool rx_flag = false;


void setup() {
  Serial.begin(1000000);
  Serial.setTimeout(10);
  Serial.print("Freq clock is ");
  Serial.print(F_CPU);
  Serial.println(" Hz");
  #if (!IS_TEST_UART)
  nrf_set();
  mpu_set();
  #endif
  for (uint8_t i = 0; i < CTRL_MLTX; i++) {
    pinMode(pin.mltx.s_ctrl[i], OUTPUT);
  }

//  UCSROB = (1<<RXEND) | (1<<RXCIEO); // разрешение перрваания по получению
  

}

//ISR (USART_RX_vect){
//  Serial.println("smth");
//}

void loop() {
  if (millis() - tmr.main > PRD.main) {
    tmr.main = millis();

    if (MODE == 0) {
#if (!IS_TEST_UART)
      if (millis() -  tmr.nrf_r > PRD.nrf_r) {
        tmr.nrf_r = millis();
        rc_nrf();
      }
      #endif

      // ctrl by nrf
    } else {
#if (!IS_TEST_UART)
      if (millis() - tmr.check_imu > PRD.check_imu) {
        tmr.check_imu = millis();
        get_imu();
      }

      if (millis() - tmr.check_mltx > PRD.check_mltx) {
        tmr.check_mltx = millis();
        get_mltx();
      }

      if (millis() - tmr.check_lidar > PRD.check_lidar) {
        tmr.check_lidar = millis();
      }

      if (millis() - tmr.check_odo > PRD.check_odo) {
        tmr.check_odo = millis();
      }
#endif
      if (millis() - tmr.tx > PRD.tx) {
        tmr.tx = millis();
        tx_uart();
      }

      if (millis() - tmr.rx > PRD.rx) {
        tmr.rx = millis();
        rx_uart();
      }

      if (millis() - tmr.set_wheel > PRD.set_wheel) {
        tmr.set_wheel = millis();
      }

      if (millis() - tmr.set_arm > PRD.set_arm) {
        tmr.set_arm = millis();
      }

    }
  }
}
#if (!IS_TEST_UART)
void nrf_set() {
  radio.begin();              // активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);   // размер пакета, в байтах

  radio.openReadingPipe(1, address[0]);   // хотим слушать трубу 0
  radio.setChannel(0x6a);     // выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  radio.powerUp();        // начать работу
  radio.startListening(); // начинаем слушать эфир, мы приёмный модуль
}

void mpu_set() {
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setDMPEnabled(true);
}
#endif

void tx_uart() {
  //  Serial.write('%');
  //  Serial.write(',');
  //
  //  //  // Отправляем int16_t как 2 байта
  //  //  Serial.write((uint8_t*)&tx.ay, 2);
  //
  //  // Отправляем uint8_t как 1 байт
  //  Serial.write(tx.irs);
  //  Serial.write(',');
  // Отправляем int16_t как 2 байта
  //Serial.write((uint8_t*)&tx.ay, 2);

  //  // Отправляем символ конца пакета
  //  Serial.write(tx.ed);
  //Serial.println();
  
  //Serial.write(tx.start_sb);
  //Serial.write(tx.x_arm);
  //Serial.println("qwe");
  print_rx_buff();
  if (rx.arm_q1 == 42){ 
//    from_int16(rx.arm_q1, int_buff);
//    Serial.write(int_buff[0]);
//    Serial.write(int_buff[1]);
Serial.write(4);
  }
  

}

void rx_uart() {
  static uint8_t i;
  if (Serial.available()){
    //Serial.println();
    if (rx_flag){
      buff.rx[i++] = Serial.read();
      //Serial.println(int8_t(rx_buff[i-1]));
      if (i > 10){
        rx_flag = false;
        i = 0;
        //print_rx_buff();
        update_control_data();
      }
    }
    else{
      if (char(Serial.read()) == rx.init_sb){
        //Serial.println(rx.init_sb);
        rx_flag = true;
        i = 0;
      }
    }
        
  }

}
void print_rx_buff(){
  for (uint8_t i =0;i<11;i++){
    Serial.write(buff.rx[i]);
  }
}
void update_control_data(){
  /*int8_t move_type = -1;
  int8_t val_move = -1;
  int16_t arm_q1 = 90;
  int16_t arm_q2 = 90;
  int16_t arm_q3 = 90;
  int8_t arm_mode = -1;
  int8_t auido_mode = -1;*/

  rx.hsum = hash(buff.rx, 1, 11);
  if (rx.hsum == buff.rx[0]){
    uint8_t i = 0;
    rx.move_type = to_int8(buff.rx[1]);
    rx.val_move = to_int8(buff.rx[2]);
    rx.arm_q1 = to_int16(buff.rx[3], buff.rx[4], &i);
    rx.arm_q2 = to_int16(buff.rx[5], buff.rx[6], &i);
    rx.arm_q3 = to_int16(buff.rx[7], buff.rx[8], &i);
    i=8;
    rx.arm_mode = to_int8(buff.rx[9]);
    rx.auido_mode = to_int8(buff.rx[10]);
  }
  else{
    rx.move_type = -1;
    rx.val_move = -1;
//    rx.arm_q1 = ;
//    rx.arm_q2 = to_int16(rx_buff[5], rx_buff[6], &i);
//    rx.arm_q3 = to_int16(rx_buff[7], rx_buff[8], &i);
//    rx.arm_mode = to_int8(rx_buff[9]);
    rx.auido_mode = -1;
    
  }
}

#if (!IS_TEST_UART)
void rc_nrf() {
  if (millis() - tmr.check_nrf > PRD.check_nrf) {
    rec_nrf.x1 = 512;
    rec_nrf.y1 = 512;
    rec_nrf.x2 = 512;
    rec_nrf.y2 = 512;
    rec_nrf.btn1 = 0;
    rec_nrf.btn2 = 0;
    // потеряли пульт, стоим
  }
  else  if (radio.available(&rec_nrf.pipeNo)) {        // слушаем эфир со всех труб
    tmr.check_nrf = millis();
    radio.read(&rec_nrf.data, sizeof(rec_nrf.data));  // чиатем входящий сигнал
    rec_nrf.x1 = rec_nrf.data[0];
    rec_nrf.y1 = rec_nrf.data[1];
    rec_nrf.x2 = rec_nrf.data[2];
    rec_nrf.y2 = rec_nrf.data[3];
    rec_nrf.btn1 = rec_nrf.data[4];
    rec_nrf.btn2 = rec_nrf.data[5];
  }
}

void tr_nrf() {
  

}

void get_imu() {
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    tx.ang_x = degrees(ypr[2]) * 1000;
    tx.ang_y = degrees(ypr[1]) * 1000;
    tx.ang_z = degrees(ypr[0]) * 1000;
  }
}
void set_mltx(uint8_t *mode, uint8_t *val_map) {
  switch (*mode) {
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
  for (uint8_t i = 0; i < CTRL_MLTX; i++) {
    digitalWrite(pin.mltx.s_ctrl[i], val_map[i]);
  }
}

void get_mltx() {
  for (uint8_t i =  0; i < NUM_IR; i++) {
    set_mltx((uint8_t*)0, pin.mltx.ir[i]);
    tx.ir[i] = digitalRead(pin.mltx.sig);
  }
  for (uint8_t i =  0; i < NUM_END; i++) {
    set_mltx((uint8_t*)0, pin.mltx.end_sens[i]);
    tx.end_sens[i] = digitalRead(pin.mltx.sig);
  }
}
#endif
float middle_of_3(float * a, float * b, float * c) {
  if ((*a <= *b) && (*a <= *c)) {
    return (*b <= *c) ? *b : *c;
  } else {
    if ((*b <= *a) && (*b <= *c)) {
      return (*a <= *c) ? *a : *c;
    } else {
      return (*a <= *b) ? *a : *b;
    }
  }
}

int8_t to_int8(uint8_t val){
  return int8_t(val);
}

int16_t to_int16(uint8_t val_1, uint8_t val_2, uint8_t *val_i){
  //inc(&(*val_i));
  return int16_t((val_2 << 8) | val_1);
}

uint8_t from_int8(int8_t val){
  return uint8_t(val);
}

void from_int16(int16_t val, uint8_t *int_buff){
    int_buff[0] = uint8_t(val);
    int_buff[1] = uint8_t(val >> 8);
}

uint8_t inc(uint8_t *val_i){
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
        //printf("%d\r\n", ch_sum);
    }
    return ch_sum;
}

bool check_data(uint8_t *data_rec, uint32_t start_i, uint32_t end_i)
{
     uint8_t rec_hash = hash(data_rec, start_i, end_i);
    return (!(data_rec[0] == rec_hash)); // 0 - хэши совпали
}

void fill_tx_arr(){
  from_int16(tx.left_wh, buff.two_bytes);
  buff.tx[0]=buff.two_bytes[0];
  buff.tx[1]=buff.two_bytes[1];


}
