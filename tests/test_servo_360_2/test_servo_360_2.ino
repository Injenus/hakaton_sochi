#include <Servo.h>
ServoSmooth servo(360);
int val = 0;

void setup() {
  Serial.begin(9600);
  servo.attach(2, 600, 2400);
  //  servo.setSpeed(50);   // ограничить скорость
  //  servo.setAccel(0.3);    // установить ускорение (разгон и торможение)

  servo.setAutoDetach(false); // отключить автоотключение (detach) при достижении целевого угла (по умолчанию включено)
}

void loop() {
  servo.tick();
  if (Serial.available() > 0) {
    val  = Serial.parseInt();
  }
  Serial.println(val);
  servo.setTarget(val);

}
