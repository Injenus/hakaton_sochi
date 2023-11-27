
#include <Servo.h>
#define WHEEL_NUM 2
int servo_pin[WHEEL_NUM] = {9, 10};
int min_mcs[WHEEL_NUM] = {600, 600}; //
int stop_mcs[WHEEL_NUM] = {1500, 1500}; //mcs
int max_mcs[WHEEL_NUM] = {2400, 2400}; //

int dead_servo[WHEEL_NUM] = {10, 10}; // +- around 90
byte dir_wheel[WHEEL_NUM] = {1, 1}; // 1 or -1
int servo_speed[WHEEL_NUM] = {stop_mcs[0], stop_mcs[1]};
Servo wheel[WHEEL_NUM];

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

RF24 radio(7, 8);  // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
#define LEN_DATA 6
int data[LEN_DATA]; //x1 y1 x2 y2 b1 b2
byte pipeNo;

#define MAIN_PERIOD 1
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
    wheel[i].attach(servo_pin[i], min_mcs[i], max_mcs[i]);
    //wheel[i].attach(servo_pin[i]);
    wheel[i].write(90);
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
    setSpeedWheel();
  }

}


void calcSpeed() {
  // тупой алгоритм как у танка
  //  servo_speed[0] = map(data[1], 0, 1023, 0, 180);
  //  servo_speed[1] = map(data[3], 0, 1023, 0, 180);

  if (data[1] > 511 + 15) {
    servo_speed[0] = map(data[1], 527, 1023, 90 - dead_servo[0], 60);
  }
  else if (data[1] < 511 - 15) {
    servo_speed[0] = map(data[1], 0, 495, 120, 90 + dead_servo[0]);
  }
  else {
    servo_speed[0] = 90;
  }

  if (data[3] > 511 + 15) {
    servo_speed[1] = map(data[3], 527, 1023, 90 + dead_servo[1], 120);
  }
  else if (data[3] < 511 - 15) {
    servo_speed[1] = map(data[3], 0, 495, 60, 90 - dead_servo[1]);
  }
  else {
    servo_speed[1] = 90;
  }





}

void setSpeedWheel() {
  calcSpeed();
  for (byte i = 0; i < WHEEL_NUM; i++) {
    wheel[i].write(servo_speed[i]);
    Serial.print(servo_speed[i]);
    Serial.print(',');
  }
  Serial.println();

}
