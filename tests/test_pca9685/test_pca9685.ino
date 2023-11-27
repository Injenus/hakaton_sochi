#include <ServoDriverSmooth.h>
//ServoDriverSmooth servo;
//ServoDriverSmooth servo(0x40);      // с указанием адреса драйвера
//ServoDriverSmooth servo(0x40, 270); // с указанием адреса и макс. угла
ServoDriverSmooth servo[4](0x40);

uint32_t tmr;
boolean flag;
const uint8_t ini = 0;
const uint8_t coun  = 1;

void setup() {
  Serial.begin(9600);
  //servo.begin();
  Serial.println("setup");
  //  servo.attach(7);     // подключить
  //  servo.write(90);
  //  servo.setSpeed(130); // ограничить скорость
  //  servo.setAccel(0.3);   // установить ускорение (разгон и торможение)
  for (uint8_t i = ini; i < coun; i++) {
    servo[i].attach(i);
    servo[i].setSpeed(120);
    servo[i].setAccel(0.3);
    servo[i].write(180);
  }
  delay(1000);
  Serial.println("qwert");
  servo[0].write(119);
  Serial.println("hf");
  delay(1000);
  Serial.println("end setup");
}

void loop() {
  for (uint8_t i = 0; i < 4; i++) {
    servo[i].tick();
  }


  if (millis() - tmr >= 3000) {   // каждые 2 сек
    tmr = millis();
    flag = !flag;
        for (uint8_t i = ini; i < coun; i++) {
          //servo[i].setTargetDeg(flag ? 90 : 90);
          if (flag){
            Serial.println("qwqw 90");
            servo[i].write(90);
          }
          else{
            Serial.println("QEQ 180");
            servo[i].write(0);
          }
        }
    //    servo.setTargetDeg(flag ? 70 : 110);  // двигаем на углы 50 и 120   //manip 22 100
  }
}
