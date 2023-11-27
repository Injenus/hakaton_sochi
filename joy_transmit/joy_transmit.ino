#include <SPI.h>          // библиотека для работы с шиной SPI
#include "nRF24L01.h"     // библиотека радиомодуля
#include "RF24.h"         // ещё библиотека радиомодуля

#define MAIN_PERIOD 5
uint32_t main_timer = 0;

RF24  radio(9, 10);
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
byte counter;
#define LEN_DATA 6
byte data_pin[LEN_DATA] = {14, 15, 17, 16, 2, 4}; //x1 y1 x2 y2 btn1 btn2
int joy_data[LEN_DATA];
#define LED 3

void setup() {
  for (int i = 2; i < 20; i++) {
    pinMode(i, OUTPUT);
  }
  Serial.begin(9600);         // открываем порт для связи с ПК

  radio.begin();              // активировать модуль
  radio.setAutoAck(1);        // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);    // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();   // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);   // размер пакета, в байтах

  radio.openWritingPipe(address[0]);  // мы - труба 0, открываем канал для передачи данных
  radio.setChannel(0x6a);             // выбираем канал (в котором нет шумов!)

  radio.setPALevel (RF24_PA_MAX);   // уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate (RF24_250KBPS); // скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике! при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();        // начать работу
  radio.stopListening();  // не слушаем радиоэфир, мы передатчик

  for (byte i = 0; i < LEN_DATA; i++) {
    if (i < 4) {
      pinMode(data_pin[i], INPUT);
    }
    else {
      pinMode(data_pin[i], INPUT_PULLUP);
    }
  }
  digitalWrite(LED, HIGH);
  Serial.println("Hell");
}

void loop() {
  if (millis() - main_timer > MAIN_PERIOD) {
    main_timer = millis();
    digitalWrite(LED, !digitalRead(LED));
    for (byte i = 0; i < LEN_DATA; i++) {
      if (i < 4) {
        joy_data[i] = analogRead(data_pin[i]);
      }
      else {
        joy_data[i] = digitalRead(data_pin[i]);
      }
    }
    radio.write(&joy_data, sizeof(joy_data));
  }
}
