/*Принимает пакет от компа и передает по nrf
  затем принимает от Робота и передаёт компу
  Это у робота MODE = 2

  этот модуль сначала ПРНИМАЕТ от МК, потом переключается в отправку для МК
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <SoftwareSerial.h>
//SoftwareSerial mySerial (rxPin, txPin);
SoftwareSerial mySerial (5, 7);

uint32_t p_timeout = 50;
uint32_t t_timer = 0;

uint32_t rep_period = 500;
uint32_t rep_timer = 0;

RF24 radio(9, 10);  // "создать" модуль на пинах 9 и 10 Для Уно
byte address[][6] = {"1Node", "2Node", "3Node", "4Node", "5Node", "6Node"}; //возможные номера труб
byte pipeNo = 1;

#define FROM_BOT 48
#define FROM_PC 12
uint8_t mod_status = 0; // 0 - ждём посылку от робота, 1 - получили, пересыламем ПК, 2 - ждём от ПК, 3 - получили, пересыламем роботу
uint8_t led_arr[3] = {3, 6, 4};
uint8_t read_1[32];
uint8_t read_2[16];
uint8_t from_bot[FROM_BOT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,};
//uint8_t from_pc[FROM_PC] = { -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2, -2};
uint8_t from_pc[FROM_PC] = {0x23, 0, 0, 0, 90, 0, 90, 0, 90, 0, 0, 0};
uint8_t temp_byte = 0b00000000;

void initset_nrf();
void send_buff(uint8_t *buff, uint8_t size_b);
int8_t rx_uart();
void set_led(uint8_t mode);
void rec_init();

void setup()
{
  Serial.begin(1000000);
  Serial.setTimeout(10);
  Serial.println("INIT");
  mySerial.begin(115200);
  mySerial.println("INIT mySerial");
  for (byte i = 0; i < 3; i++) {
    pinMode(led_arr[i], OUTPUT);
  }
  initset_nrf();
  mod_status = 0;
  //set_led(mod_status);
}

void loop() {
  while (radio.available(&pipeNo)) {                    // слушаем эфир
    radio.read(&read_1, 32);  // чиатем входящий сигнал
    radio.read(&read_2, 16);  // чиатем входящий сигнал

    if (read_1[0] == 0b00100101 && read_2[10] == 5) {
      for (uint8_t i = 0; i < 32; i++) {
        from_bot[i] = read_1[i];
      }
      for (uint8_t i = 0; i < 16; i++) {
        from_bot[32 + i] = read_2[i];
      }
    }
    // отправляем пакет телеметрии
    radio.writeAckPayload(pipeNo, &from_pc, FROM_PC);
    send_buff(from_bot, FROM_BOT);
  }


  // формируем пакет данных телеметрии
  if (millis() - rep_timer > rep_period) {
    rep_timer = millis();
    if (rx_uart() == -1) {
      rec_init();
    }




    //    for (uint8_t i = 0; i < 12; i++) {
    //      mySerial.print(from_pc[i]);
    //      mySerial.print(',');
    //    }
    //    mySerial.println();
    //  }

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

void send_buff(uint8_t *buff, uint8_t size_b)
{
  for (uint8_t i = 0; i < size_b; i++)
  {
    Serial.write(buff[i]);
  }
}

int8_t rx_uart() {

  uint8_t i = 0;
  t_timer = millis();
  while (i < FROM_PC) {
    if (Serial.available())
    {
      set_led(0);
      temp_byte = Serial.read();
      //      if (i == 0 && temp_byte == 0b01100011) {
      //
      //        from_pc[i++] = temp_byte;
      //      }
      //      else if (i != 0) {
      //        //set_led(3);
      //        from_pc[i++] = temp_byte;
      //      }


      from_pc[i++] = temp_byte;

    }
    set_led(3);
    //    else {
    //      //return -2;
    //      i=99;
    //    }
    if (millis() - t_timer > p_timeout) {

      return -1;
    }
  }
  return 0;
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

void rec_init() {
  from_pc[0] = 0x23;
  from_pc[1] = 0;
  from_pc[2] = 0;
}
