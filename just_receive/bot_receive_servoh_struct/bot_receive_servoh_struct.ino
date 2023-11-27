//Скорость колеса от -1000 до 1000 в абстрактных единцах

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#include <Servo.h>

const byte WHEEL_NUM = 2;
int wheel_pin[WHEEL_NUM] = {9, 10};

struct Mcs {
  int const min_prd = 600;
  int const stop_prd = 1500;
  int const max_prd = 2400;
};

struct Servo_val {
  int const dead_zone = 19; // +- relative to 90
  int const min_v = 55; // >=0 <90
  int const stop_v = 90; //
  int const max_v = 125; // >90 <=180
};

struct Wheel {
  Servo servo[WHEEL_NUM];
  bool const is_direct[WHEEL_NUM] = {false, true};
  int const min_spd = -1000;
  int const max_spd = 1000;
  int abstr_spd[WHEEL_NUM] = {0, 0};
};

Mcs mcs;
Servo_val servo_val;
Wheel wheel;


RF24 radio(7, 8);  // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
const byte LEN_DATA = 6;
int data[LEN_DATA]; //x1 y1 x2 y2 b1 b2
byte pipeNo;

#define MAIN_PERIOD 10
uint32_t main_timer = 0;

void setup() {
  for (int i = 2; i < 22; i++) {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);         // открываем порт для связи с ПК
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

  for (int i = 0; i < WHEEL_NUM; i++) {
    wheel.servo[i].attach(wheel_pin[i], mcs.min_prd, mcs.max_prd);
    wheel.servo[i].write(90);
  }
}

void loop() {
  if (radio.available(&pipeNo)) {        // слушаем эфир со всех труб
    radio.read(&data, sizeof(data));  // чиатем входящий сигнал

    //    Serial.print("Recieved: ");
    //    Serial.print(data[0]);
    //    for (byte i = 1; i < LEN_DATA; i++) {
    //      Serial.print(',');
    //      Serial.print(data[i]);
    //    }
    //    Serial.println(';');
  }// если не принимаем данные, ставим иниты
  //  for (byte i = 0; i < LEN_DATA; i++) {
  //    if (i < LEN_DATA - 2) {
  //      data[i] = 512;
  //    }
  //    else {
  //      data[i] = 1;
  //    }
  //  }

  if (millis() - main_timer > MAIN_PERIOD) {
    main_timer = millis();

    for (byte i = 0; i < WHEEL_NUM; i++) {
      byte ind = byte(pow(2, i)) + i;
      int val = convertSpeedToVal(dataToSpeed(data[ind], wheel.is_direct[i]));
      wheel.servo[i].write(val);
    }
  }

}

void setWheelSpeed() {

}

int dataToSpeed(int analog, bool is_dir) {
  if (!is_dir) {
    analog = map(analog, 0, 1023, 1023, 0);
  }

  int speed_ = 0;
  if (analog > 534) {
    speed_ = map(analog, 535, 1023, 1, wheel.max_spd);
  }
  else if (analog < 480) {
    speed_ = map(analog, 0, 479, wheel.min_spd, 1);
  }

  return speed_;
}

int convertSpeedToVal(int speed_) {
  speed_ = constrain(speed_, wheel.min_spd, wheel.max_spd);
  int val = 90;
  if (speed_ < 0) {
    val = map(speed_, wheel.min_spd, 1, servo_val.min_v, servo_val.stop_v - servo_val.dead_zone);
  } else if (speed_ > 0) {
    val = map(speed_, 1, wheel.max_spd, servo_val.stop_v + servo_val.dead_zone, servo_val.max_v);
  }
  return val;
}
