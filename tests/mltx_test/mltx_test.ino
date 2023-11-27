#define CTRL_MLTX 3
#define NUM_IR 2
#define NUM_END 4
#define DATA_NRF 6
struct Multiplexor
{
  const uint8_t s_ctrl[CTRL_MLTX] = {15, 16, 17}; // A1 A2 A3  //num.mltx_ctrl
  const uint8_t sig = 14;                         // A0
  const uint8_t ir[NUM_IR][CTRL_MLTX] = {         // num.ir  //num.mltx_ctrl
      {0, 0, 0},
      {1, 0, 0}};
  const uint8_t end_sens[NUM_END][CTRL_MLTX] = { // num.end_sens  //num.mltx_ctrl
      {0, 1, 0},
      {1, 1, 0},
      {0, 0, 1},
      {1, 0, 1}};
  int8_t pin_mode = 0;
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
  Multiplexor mltx;
};
Pin pin;
struct Transmit
{
  char start_sb = '%';
  // char end_sb = ';';
  // char terminator = ',';
  uint8_t hsum = 0x09;
  int16_t left_wh = 5;
  int16_t right_wh = 256;
  int16_t mode_move = 9; // сейчас считаем это за индикатор состяния последней команды 1 - выполнено, 0 - выполняется
  int16_t x_arm = 256;
  int16_t y_arm = -1;
  int16_t z_arm = -1;
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

void setup(){
  Serial.begin(9600);
}
void loop(){

  get_mltx();
  Serial.print("Res: ");
  Serial.println(tx.end_sens);
}


void set_mltx(uint8_t *mode, uint8_t *val_map)
{
  Serial.println(*mode);
  Serial.print(val_map[0]);
  Serial.print(val_map[1]);
  Serial.println(val_map[2]);
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
//  tx.ir = 0b10000000;
//  for (uint8_t i = 0; i < NUM_IR; i++)
//  {
//    set_mltx((uint8_t *)0, pin.mltx.ir[i]);
//    tx.ir = tx.ir | (digitalRead(pin.mltx.sig) << i);
//  }
  tx.end_sens = 0b10000000;
  for (uint8_t i = 0; i < NUM_END; i++)
  {
    set_mltx(&pin.mltx.pin_mode, pin.mltx.end_sens[i]);
    tx.end_sens = tx.end_sens | (digitalRead(pin.mltx.sig) << i);
  }
}
