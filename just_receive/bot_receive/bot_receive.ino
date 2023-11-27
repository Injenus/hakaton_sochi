#define WHEEL_NUM 2
byte servo_pin[WHEEL_NUM] = {9, 10};
int min_mcs[WHEEL_NUM] = {600, 600}; //
int stop_mcs[WHEEL_NUM] = {1595, 1595}; //mcs
int max_mcs[WHEEL_NUM] = {2500, 2500}; //
int min_10bit[WHEEL_NUM] = {200, 200}; //
int stop_10bit[WHEEL_NUM] = {400, 400}; //10bit
int max_10bit[WHEEL_NUM] = {600, 600}; //
byte dir_wheel[WHEEL_NUM] = {1, 1}; // 1 or -1
int servo_speed[WHEEL_NUM] = {stop_10bit[0], stop_10bit[1]};

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
  // Пины D9 и D10 - 244 Гц 10bit
  TCCR1A = 0b00000011;  // 10bit
  TCCR1B = 0b00001011;  // x64 fast pwm
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
    calcSpeed();
    setSpeedWheel();
  }




}
void calcSpeed() {
  // тупой алгоритм как у танка

  //  if (data[1] > 512) {
  //    servo_speed[0] = stop_speed[0] + dir_wheel[0] * exp(dir_wheel[0] * (double(data[1]) - 511) / 75);
  //  }
  //  else {
  //    servo_speed[0] = stop_speed[0] - dir_wheel[0] * exp(dir_wheel[0] * (-double(data[1]) + 511) / 75);
  //  }
  //
  //  if (data[3] > 512) {
  //    servo_speed[1] = stop_speed[1] + dir_wheel[1] * exp(dir_wheel[1] * (double(data[3]) - 511) / 75);
  //  }
  //  else {
  //    servo_speed[1] = stop_speed[1] - dir_wheel[1] * exp(dir_wheel[1] * (-double(data[3]) + 511) / 75);
  //  }


  servo_speed[0] = stop_mcs[0] + 1.6 * (data[1] - 511);
  servo_speed[1] = stop_mcs[1] - 1.6 * (data[3] - 511);

  servo_speed[0] = map(servo_speed[0], min_mcs[0], max_mcs[0], min_10bit[0], max_10bit[0]);
  servo_speed[1] = map(servo_speed[1], min_mcs[1], max_mcs[1], min_10bit[1], max_10bit[1]);


}

void setSpeedWheel() {
  for (byte i = 0; i < WHEEL_NUM; i++) {
    analogWrite(servo_pin[i], servo_speed[i]);
    Serial.print(servo_speed[i]);
    Serial.print(',');
  }
  Serial.println();
}
