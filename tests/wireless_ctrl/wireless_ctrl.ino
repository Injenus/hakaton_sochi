/*Принимает пакет от компа и передает по nrf
  затем принимает от Робота и передаёт компу
  Это у робота MODE = 2

  этот модуль сначала ПРНИМАЕТ от МК, потом переключается в отправку для МК
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(9, 10);  // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
byte pipeNo;

#define FROM_BOT 48
#define FROM_PC 12
uint8_t mod_status = 0; // 0 - ждём посылку от робота, 1 - получили, пересыламем ПК, 2 - ждём от ПК, 3 - получили, пересыламем роботу
uint8_t led_arr[3] = {3, 6, 7};

uint8_t from_bot[FROM_BOT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
uint8_t from_pc[FROM_PC] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};


void init_nrf_set();
void set_transmit();
void set_receive();
void send_buff(uint8_t *buff, uint8_t size_b);
void rx_uart();
void set_led(uint8_t mode);

void setup()
{
  Serial.begin(1000000);
  Serial.setTimeout(10);
  Serial.println("INIT");
  for (byte i = 0; i < 3; i++) {
    pinMode(led_arr[i], OUTPUT);
  }
  initset_nrf();
  set_receive();
  mod_status = 0;
  set_led(mod_status);
}

void loop() {

  //  while (mod_status == 0) {
  //    if (radio.available()) {        // слушаем эфир со всех труб
  //      radio.read(&from_bot, FROM_BOT);  // чиатем входящий сигнал
  //      mod_status = 1;
  //      set_led(mod_status);
  //    }
  //    set_transmit();
  //  }
  //
  //  while (mod_status == 1) {
  //    send_buff(from_bot, FROM_BOT);
  //    mod_status = 2;
  //    set_led(mod_status);
  //  }
  //
  //  while (mod_status == 2) {
  //    rx_uart();
  //    mod_status = 3;
  //    set_led(mod_status);
  //  }
  //
  //  while (mod_status == 3) {
  //    radio.write(&from_pc, FROM_PC);
  //    mod_status = 0;
  //    set_led(mod_status);
  //  }

  while (radio.available(&pipeNo)) {                    // слушаем эфир
    radio.read(&from_bot, 48);  // чиатем входящий сигнал
    send_buff(from_bot, FROM_BOT);

    // формируем пакет данных телеметрии
    rx_uart();

    // отправляем пакет телеметрии
    radio.writeAckPayload(pipeNo, &from_pc, FROM_PC);
  }

}

void initset_nrf() { ///
  radio.begin();                // активировать модуль
  radio.setAutoAck(1);          // режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 15);      // (время между попыткой достучаться, число попыток)
  radio.enableAckPayload();     // разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);     // размер пакета, байт
  radio.openReadingPipe(1, address[0]); // хотим слушать трубу 0
  radio.setChannel(0x6a);     // выбираем канал (в котором нет шумов!)
  radio.setPALevel(RF24_PA_MAX);  // уровень мощности передатчика
  radio.setDataRate(RF24_2MBPS); // скорость обмена
  // должна быть одинакова на приёмнике и передатчике!
  // при самой низкой скорости имеем самую высокую чувствительность и дальность!!

  radio.powerUp();         // начать работу
  radio.startListening();  // начинаем слушать эфир, мы приёмный модуль
}

void set_transmit() {
  radio.stopListening();
  radio.openWritingPipe(address[0]);  // мы - труба 0, открываем канал для передачи данных
}

void set_receive() {
  radio.openReadingPipe(1, address[0]);   // хотим слушать трубу 0
  radio.startListening();
}

void send_buff(uint8_t *buff, uint8_t size_b)
{
  for (uint8_t i = 0; i < size_b; i++)
  {
    Serial.write(buff[i]);
  }
}

void rx_uart() {
  uint8_t i = 0;
  while (i < FROM_PC) {
    if (Serial.available())
    {
      from_pc[i++] = Serial.read();
    }
  }
}

void set_led(uint8_t mode) {
  for (byte i = 0; i < 3; i++) {
    digitalWrite(led_arr[i], 0);
  }
  switch (mode) {
    case 0:
      digitalWrite(led_arr[0], 1);
      digitalWrite(led_arr[1], 1);
      break;
    case 1:
      digitalWrite(led_arr[0], 1);
      digitalWrite(led_arr[2], 1);
      break;
    case 2:
      digitalWrite(led_arr[0], 0);
      digitalWrite(led_arr[1], 1);
      break;
    case 3:
      digitalWrite(led_arr[0], 0);
      digitalWrite(led_arr[2], 1);
      break;
  }

}
