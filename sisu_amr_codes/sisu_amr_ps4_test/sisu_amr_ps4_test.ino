#include <PS4BT.h>
#include <usbhub.h>
#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <INA226_WE.h>
#include <Servo.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <TimerFour.h>
#include <TimerFive.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>

#define right_hall_A_pin A10 //SARI
#define right_hall_B_pin A11 //YESIL
#define right_hall_C_pin A12 //MAVI
#define right_speed_pin 4
#define right_direction_pin 22
#define right_slow_speed_pin 23
#define right_fast_speed_pin 24
#define right_low_stop_pin 25

#define left_hall_A_pin A13 //SARI
#define left_hall_B_pin A14 //YESIL
#define left_hall_C_pin A15 //MAVI
#define left_speed_pin 5
#define left_direction_pin 26
#define left_slow_speed_pin 27
#define left_fast_speed_pin 28
#define left_low_stop_pin 29

#define right_lineer_speed_pin 6
#define right_lineer_upper_pin 30
#define right_lineer_lower_pin 31

#define left_lineer_speed_pin 8
#define left_lineer_upper_pin 32
#define left_lineer_lower_pin 33

#define right_lantern_pin 37
#define left_lantern_pin 38

#define buzzer_pin 39

#define lineer_speed 255

#define INA226_I2C_ADDRESS 0x40

#define I2C_COMM_ADDRESS 0x08

USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd, PAIR);
//PS4BT PS4(&Btd);

INA226_WE ina226(INA226_I2C_ADDRESS);

Servo right_motor, left_motor;

float right_speed = 0.0;
long right_counter = 0;
char right_hall_before = 0;
char right_hall_begin = 0;
unsigned long right_time_begin = 0;

float left_speed = 0.0;
long left_counter = 0;
char left_hall_before = 0;
char left_hall_begin = 0;
unsigned long left_time_begin = 0;

bool mz80_val = 0;
float mass_g = 0;
int lever_chrg_val;

unsigned char strip_led_color = 1;

struct amrVehicleReceiving {
  bool mz80_val;
  float mass_g;
  int chrg_val;
} amr_vehicle_data;

bool right_speed_counter_started = false;

void updateRightHallA() {

  if (right_counter == 0) {
    right_time_begin = millis();
  }

  if (right_hall_before == 'B') {
    right_counter--;
  }

  else if (right_hall_before == 'C') {
    right_counter++;
  }

  if (right_counter == 3) {
    right_speed = 1000.0 / (millis() - right_time_begin);
    right_counter = 0;
  }

  else if (right_counter == -3) {
    right_speed = -1000.0 / (millis() - right_time_begin);
    right_counter = 0;
  }

  right_hall_before = 'A';
}

void updateRightHallB() {

  if (right_counter == 0) {
    right_time_begin = millis();
  }

  if (right_hall_before == 'A') {
    right_counter--;
  }

  else if (right_hall_before == 'C') {
    right_counter++;
  }

  if (right_counter == 3) {
    right_speed = 1000.0 / (millis() - right_time_begin);
    right_counter = 0;
  }

  else if (right_counter == -3) {
    right_speed = -1000.0 / (millis() - right_time_begin);
    right_counter = 0;
  }

  right_hall_before = 'B';
}

void updateRightHallC() {

  if (right_counter == 0) {
    right_time_begin = millis();
  }

  if (right_hall_before == 'A') {
    right_counter--;
  }

  else if (right_hall_before == 'B') {
    right_counter++;
  }

  if (right_counter == 3) {
    right_speed = 1000.0 / (millis() - right_time_begin);
    right_counter = 0;
  }

  else if (right_counter == -3) {
    right_speed = -1000.0 / (millis() - right_time_begin);
    right_counter = 0;
  }

  right_hall_before = 'C';
}

void updateLeftHallA() {

  if (left_counter == 0) {
    left_time_begin = millis();
  }

  if (left_hall_before == 'B') {
    left_counter--;
  }

  else if (left_hall_before == 'C') {
    left_counter++;
  }

  if (left_counter == 3) {
    left_speed = 1000.0 / (millis() - left_time_begin);
    left_counter = 0;
  }

  else if (left_counter == -3) {
    left_speed = -1000.0 / (millis() - left_time_begin);
    left_counter = 0;
  }

  left_hall_before = 'A';
}

void updateLeftHallB() {

  if (left_counter == 0) {
    left_time_begin = millis();
  }

  if (left_hall_before == 'A') {
    left_counter--;
  }

  else if (left_hall_before == 'C') {
    left_counter++;
  }

  if (left_counter == 3) {
    left_speed = 1000.0 / (millis() - left_time_begin);
    left_counter = 0;
  }

  else if (left_counter == -3) {
    left_speed = -1000.0 / (millis() - left_time_begin);
    left_counter = 0;
  }

  left_hall_before = 'B';
}

void updateLeftHallC() {

  if (left_counter == 0) {
    left_time_begin = millis();
  }

  if (left_hall_before == 'A') {
    left_counter--;
  }

  else if (left_hall_before == 'B') {
    left_counter++;
  }

  if (left_counter == 3) {
    left_speed = 1000.0 / (millis() - left_time_begin);
    left_counter = 0;
  }

  else if (left_counter == -3) {
    left_speed = -1000.0 / (millis() - left_time_begin);
    left_counter = 0;
  }

  left_hall_before = 'C';
}

void motorSpeedControl() {
  if (millis() - right_time_begin >= 1000) {
    right_counter = 0;
    right_speed = 0;
  }
  if (millis() - left_time_begin >= 1000) {
    left_counter = 0;
    left_speed = 0;
  }
}

void readMass() {

  static bool sent = false;

  if (not sent) {
    Serial3.write('+');
    sent = true;
  }

  if (Serial3.available() > 0) {
    Serial3.readBytes((uint8_t*)&amr_vehicle_data, sizeof(amr_vehicle_data));
    mz80_val = amr_vehicle_data.mz80_val;
    mass_g = amr_vehicle_data.mass_g;
    lever_chrg_val = amr_vehicle_data.chrg_val;
    sent = false;
  }
  else {
    static unsigned long begin_time = millis();
    if (millis() - begin_time >= 500) {
      sent = false;
      begin_time = millis();
    }
  }
}

char direction = 'S';

char gear = 0;
bool gear_opened = false;

char speed_level = 7;
bool speed_opened = false;
byte speed;

char lineer_direction = 0;
char lineer_direction_before = 0;
bool lineer_opened = false;

void setup() {

  Wire.begin();

  Serial.begin(115200);

  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }

  Serial.print(F("\r\nPS4 Bluetooth Library Started"));

  pinMode(right_hall_A_pin, INPUT);
  pinMode(right_hall_B_pin, INPUT);
  pinMode(right_hall_C_pin, INPUT);
  right_motor.attach(right_speed_pin);
  pinMode(right_direction_pin, OUTPUT);
  pinMode(right_slow_speed_pin, OUTPUT);
  pinMode(right_fast_speed_pin, OUTPUT);
  pinMode(right_low_stop_pin, OUTPUT);

  pinMode(left_hall_A_pin, INPUT);
  pinMode(left_hall_B_pin, INPUT);
  pinMode(left_hall_C_pin, INPUT);
  left_motor.attach(left_speed_pin);
  pinMode(left_direction_pin, OUTPUT);
  pinMode(left_slow_speed_pin, OUTPUT);
  pinMode(left_fast_speed_pin, OUTPUT);
  pinMode(left_low_stop_pin, OUTPUT);

  pinMode(right_lineer_upper_pin, OUTPUT);
  pinMode(right_lineer_lower_pin, OUTPUT);
  pinMode(right_lineer_speed_pin, OUTPUT);

  pinMode(left_lineer_upper_pin, OUTPUT);
  pinMode(left_lineer_lower_pin, OUTPUT);
  pinMode(left_lineer_speed_pin, OUTPUT);

  pinMode(right_lantern_pin, OUTPUT);
  pinMode(left_lantern_pin, OUTPUT);

  pinMode(buzzer_pin, OUTPUT);

  attachPinChangeInterrupt(digitalPinToPCINT(right_hall_A_pin), updateRightHallA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(right_hall_B_pin), updateRightHallB, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(right_hall_C_pin), updateRightHallC, CHANGE);

  attachPinChangeInterrupt(digitalPinToPCINT(left_hall_A_pin), updateLeftHallA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(left_hall_B_pin), updateLeftHallB, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(left_hall_C_pin), updateLeftHallC, CHANGE);

  right_motor.writeMicroseconds(500);
  left_motor.writeMicroseconds(500);

  digitalWrite(right_direction_pin, HIGH);
  digitalWrite(right_slow_speed_pin, HIGH);
  digitalWrite(right_fast_speed_pin, HIGH);
  digitalWrite(right_low_stop_pin, HIGH);

  digitalWrite(left_direction_pin, HIGH);
  digitalWrite(left_slow_speed_pin, HIGH);
  digitalWrite(left_fast_speed_pin, HIGH);
  digitalWrite(left_low_stop_pin, HIGH);

  digitalWrite(right_lineer_upper_pin, LOW);
  digitalWrite(right_lineer_lower_pin, LOW);

  digitalWrite(left_lineer_upper_pin, LOW);
  digitalWrite(left_lineer_lower_pin, LOW);

  digitalWrite(right_lantern_pin, HIGH);
  digitalWrite(left_lantern_pin, HIGH);

  digitalWrite(buzzer_pin, LOW);

  Serial2.begin(115200);

  ina226.init();
  //ina226.setResistorRange(0.1, 1.3);
  //ina226.setCorrectionFactor(0.93);
  ina226.waitUntilConversionCompleted();

  Serial3.begin(9600);

  Timer1.initialize(100000);
  Timer1.attachInterrupt(motorSpeedControl);

  /*Timer1.initialize(2000);
    Timer4.initialize(2000);*/

  Serial.println("System started.");
}

void loop() {

  //if (millis() - readMassmacounter >= 100) {
  readMass();
  //}

  //Serial.println(mass_g);

  /*static unsigned long speedKontrolcounter = millis();

    if (millis() - speedKontrolcounter >= 100) {
    motorSpeedControl();
    speedKontrolcounter = millis();
    }*/

  /*digitalWrite(right_lineer_upper_pin, HIGH);
    delay(1000);
    digitalWrite(right_lineer_upper_pin, LOW);
    delay(1000);*/

  /*digitalWrite(right_lineer_lower_pin, HIGH);
    delay(1000);
    digitalWrite(right_lineer_lower_pin, LOW);
    delay(1000);*/

  /*digitalWrite(right_lineer_upper_pin, LOW);
    digitalWrite(right_lineer_lower_pin, HIGH);
    delay(1000);

    digitalWrite(right_lineer_upper_pin, HIGH);
    digitalWrite(right_lineer_lower_pin, LOW);
    delay(1000);

    digitalWrite(right_lineer_upper_pin, HIGH);
    digitalWrite(right_lineer_lower_pin, HIGH);
    delay(1000);*/

  ina226.readAndClearFlags();

  float voltage = ina226.getBusVoltage_V();
  float current = ina226.getCurrent_mA();

  //readMass();

  /*const uint8_t bufSize = sizeof(amr_vehicle_data);
    byte buf[bufSize];
    uint8_t numLostMsgs = 0;
    uint8_t numRcvdBytes = 0;

    uint8_t err = getReceivedData(buf, bufSize, numRcvdBytes, numLostMsgs);

    if (err == TRF_ERR_SUCCESS) {
    memcpy(&amr_vehicle_data, buf, sizeof(amr_vehicle_data));
    mz80_val = amr_vehicle_data.mz80_val;
    mass_g = amr_vehicle_data.mass_g;
    Serial.println(mass_g);
    }*/

  /*if (mySwitch.available()) {
    //memcpy(&amr_vehicle_data, mySwitch.getReceivedRawdata(), sizeof(amr_vehicle_data));
    byte buf[sizeof(amr_vehicle_data)];
    getReceivedData(buf, sizeof(amr_vehicle_data));
    memcpy(&amr_vehicle_data, buf, sizeof(amr_vehicle_data));
    mz80_val = amr_vehicle_data.mz80_val;
    mass_g = amr_vehicle_data.mass_g;
    //output(mySwitch.getReceivedValue(), mySwitch.getReceivedBitlength(), mySwitch.getReceivedDelay(), mySwitch.getReceivedRawdata(),mySwitch.getReceivedProtocol());
    //Serial.println(mass_g);
    //Serial.println(mySwitch.getReceivedValue());
    mySwitch.resetAvailable();
    }*/

  Serial2.print("voltage_val.val=");
  Serial2.print(int(voltage * 100));
  Serial2.print("\xFF\xFF\xFF");

  Serial2.print("current_val.val=");
  Serial2.print(int(current * 100));
  Serial2.print("\xFF\xFF\xFF");

  Serial2.print("right_spd_val.val=");
  Serial2.print(int(right_speed * 100));
  Serial2.print("\xFF\xFF\xFF");

  Serial2.print("left_spd_val.val=");
  Serial2.print(int(left_speed * 100));
  Serial2.print("\xFF\xFF\xFF");

  Serial2.print("mass_val.val=");
  Serial2.print(int(mass_g));
  Serial2.print("\xFF\xFF\xFF");

  Serial2.print("rem_chrg_val.val=");
  Serial2.print(lever_chrg_val);
  Serial2.print("\xFF\xFF\xFF");

  Usb.Task();

  if (PS4.connected()) {

    int ps4_chrg = PS4.getBatteryLevel();

    Serial2.print("ps4_chrg_val.val=");
    Serial2.print(ps4_chrg * 100);
    Serial2.print("\xFF\xFF\xFF");

    //Serial.println(PS4.getBatteryLevel());

    if (PS4.getButtonPress(RIGHT) and not PS4.getButtonPress(LEFT)) {
      if (not gear_opened) {
        gear++;
        if (gear > 1) gear = 1;
      }
      gear_opened = true;
    }

    else if (not PS4.getButtonPress(RIGHT) and PS4.getButtonPress(LEFT)) {
      if (not gear_opened) {
        gear--;
        if (gear < -1) gear = -1;
      }
      gear_opened = true;
    }

    else {
      gear_opened = false;
    }

    switch (gear) {

      case -1:
        digitalWrite(right_slow_speed_pin, LOW);
        digitalWrite(right_fast_speed_pin, HIGH);
        digitalWrite(left_slow_speed_pin, LOW);
        digitalWrite(left_fast_speed_pin, HIGH);
        break;

      case 0:
        digitalWrite(right_slow_speed_pin, HIGH);
        digitalWrite(right_fast_speed_pin, HIGH);
        digitalWrite(left_slow_speed_pin, HIGH);
        digitalWrite(left_fast_speed_pin, HIGH);
        break;

      case 1:
        digitalWrite(right_slow_speed_pin, HIGH);
        digitalWrite(right_fast_speed_pin, LOW);
        digitalWrite(left_slow_speed_pin, HIGH);
        digitalWrite(left_fast_speed_pin, LOW);
        break;
    }

    /*if (PS4.getButtonPress(UP) and not PS4.getButtonPress(DOWN)) {
      if (not speed_opened) {
        speed_level++;
        if (speed_level > 7) speed_level = 7;
      }
      speed_opened = true;
      }

      else if (not PS4.getButtonPress(UP) and PS4.getButtonPress(DOWN)) {
      if (not speed_opened) {
        speed_level--;
        if (speed_level < 0) speed_level = 0;
      }
      speed_opened = true;
      }

      else {
      speed_opened = false;
      }

      speed = map(speed_level, 0, 8, 63, 256);*/

    static bool strip_led_color_set = false;

    static int speed = 2500;

    if (PS4.getButtonPress(UP) and not PS4.getButtonPress(DOWN)) {
      if (not strip_led_color_set) {
        //strip_led_color++;
        //strip_led_color %= 8;
        speed += 5;
        speed = constrain(speed, 500, 2500);
        /*digitalWrite(buzzer_pin, HIGH);
          delay(100);
          digitalWrite(buzzer_pin, LOW);*/
        //Serial.println(speed);
        /*Serial2.print("voltage_deger.val=");
          Serial2.print(speed);
          Serial2.print("\xFF\xFF\xFF");*/
        /*Wire.beginTransmission(I2C_COMM_ADDRESS);
          Wire.write('0' + strip_led_color);
          Wire.endTransmission();*/
        strip_led_color_set = true;
      }
    }

    else if (not PS4.getButtonPress(UP) and PS4.getButtonPress(DOWN)) {
      if (not strip_led_color_set) {
        //strip_led_color--;
        //strip_led_color %= 8;
        speed -= 5;
        speed = constrain(speed, 500, 2500);
        /*digitalWrite(buzzer_pin, HIGH);
          delay(100);
          digitalWrite(buzzer_pin, LOW);*/
        //Serial.println(speed);
        /*Serial2.print("voltage_deger.val=");
          Serial2.print(speed);
          Serial2.print("\xFF\xFF\xFF");*/
        /*Wire.beginTransmission(I2C_COMM_ADDRESS);
          Wire.write('0' + strip_led_color);
          Wire.endTransmission();*/
        strip_led_color_set = true;
      }
    }

    else {
      strip_led_color_set = false;
    }

    static int right_pwm = 0;
    static int left_pwm = 0;

    if (PS4.getAnalogButton(R2) > 127 and
        PS4.getAnalogButton(L2) <= 127 and
        not PS4.getButtonPress(R1) and
        not PS4.getButtonPress(L1)) {
      digitalWrite(right_direction_pin, HIGH);
      digitalWrite(left_direction_pin, HIGH);
      if (direction != 'F') {
        delay(100);
      }
      direction = 'F';
      Serial.print(right_pwm);
      Serial.print(" ");
      Serial.println(left_pwm);
      //analogWrite(right_speed_pin, right_pwm);
      //analogWrite(left_speed_pin, speed);
      right_motor.writeMicroseconds(speed);
      left_motor.writeMicroseconds(speed);
      //Timer1.pwm(right_speed_pin, map(speed, 0, 255, 0, 1023));
      //Timer4.pwm(left_speed_pin, map(speed, 0, 255, 0, 1023));
    }

    else if (PS4.getAnalogButton(R2) <= 127 and
             PS4.getAnalogButton(L2) > 127 and
             not PS4.getButtonPress(R1) and
             not PS4.getButtonPress(L1)) {
      digitalWrite(right_direction_pin, LOW);
      digitalWrite(left_direction_pin, LOW);
      if (direction != 'B') {
        delay(100);
      }
      direction = 'B';
      //analogWrite(right_speed_pin, speed);
      //analogWrite(left_speed_pin, speed);
      right_motor.writeMicroseconds(speed);
      left_motor.writeMicroseconds(speed);
      //Timer1.pwm(right_speed_pin, map(speed, 0, 255, 0, 1023));
      //Timer4.pwm(left_speed_pin, map(speed, 0, 255, 0, 1023));
    }

    else if (PS4.getAnalogButton(R2) <= 127 and
             PS4.getAnalogButton(L2) <= 127 and
             PS4.getButtonPress(R1) and
             not PS4.getButtonPress(L1)) {
      digitalWrite(right_direction_pin, LOW);
      digitalWrite(left_direction_pin, HIGH);
      if (direction != 'R') {
        delay(100);
      }
      direction = 'R';
      //analogWrite(right_speed_pin, speed);
      //analogWrite(left_speed_pin, speed);
      right_motor.writeMicroseconds(speed);
      left_motor.writeMicroseconds(speed);
      //Timer1.pwm(right_speed_pin, map(speed, 0, 255, 0, 1023));
      //Timer4.pwm(left_speed_pin, map(speed, 0, 255, 0, 1023));
    }

    else if (PS4.getAnalogButton(R2) <= 127 and
             PS4.getAnalogButton(L2) <= 127 and
             not PS4.getButtonPress(R1) and
             PS4.getButtonPress(L1)) {
      digitalWrite(right_direction_pin, HIGH);
      digitalWrite(left_direction_pin, LOW);
      if (direction != 'L') {
        delay(100);
      }
      direction = 'L';
      //analogWrite(right_speed_pin, speed);
      //analogWrite(left_speed_pin, speed);
      right_motor.writeMicroseconds(speed);
      left_motor.writeMicroseconds(speed);
      //Timer1.pwm(right_speed_pin, map(speed, 0, 255, 0, 1023));
      //Timer4.pwm(left_speed_pin, map(speed, 0, 255, 0, 1023));
    }

    else {
      //analogWrite(right_speed_pin, 0);
      //analogWrite(left_speed_pin, 0);
      right_motor.writeMicroseconds(500);
      left_motor.writeMicroseconds(500);
    }

    if (PS4.getButtonPress(CIRCLE)) {
      digitalWrite(right_low_stop_pin, LOW);
      digitalWrite(left_low_stop_pin, LOW);
    }

    else {
      digitalWrite(right_low_stop_pin, HIGH);
      digitalWrite(left_low_stop_pin, HIGH);
    }

    if (PS4.getButtonPress(CROSS) and not PS4.getButtonPress(SQUARE)) {
      lineer_direction = 1;
    }

    else if (not PS4.getButtonPress(CROSS) and PS4.getButtonPress(SQUARE)) {
      lineer_direction = -1;
    }

    else {
      lineer_direction = 0;
    }

    switch (lineer_direction) {

      case -1:
        digitalWrite(right_lineer_upper_pin, LOW);
        digitalWrite(right_lineer_lower_pin, HIGH);
        analogWrite(right_lineer_speed_pin, lineer_speed);
        digitalWrite(left_lineer_upper_pin, LOW);
        digitalWrite(left_lineer_lower_pin, HIGH);
        analogWrite(left_lineer_speed_pin, lineer_speed);
        break;

      case 0:
        digitalWrite(right_lineer_upper_pin, HIGH);
        digitalWrite(right_lineer_lower_pin, HIGH);
        analogWrite(right_lineer_speed_pin, 0);
        digitalWrite(left_lineer_upper_pin, HIGH);
        digitalWrite(left_lineer_lower_pin, HIGH);
        analogWrite(left_lineer_speed_pin, 0);
        break;

      case 1:
        digitalWrite(right_lineer_upper_pin, HIGH);
        digitalWrite(right_lineer_lower_pin, LOW);
        analogWrite(right_lineer_speed_pin, lineer_speed);
        digitalWrite(left_lineer_upper_pin, HIGH);
        digitalWrite(left_lineer_lower_pin, LOW);
        analogWrite(left_lineer_speed_pin, lineer_speed);
        break;
    }

    lineer_direction_before = lineer_direction;
  }

  static bool lantern_set = false;
  static bool lantern_status = true;

  if (PS4.getButtonPress(TRIANGLE)) {
    if (not lantern_set) {
      lantern_status = not lantern_status;
      digitalWrite(right_lantern_pin, lantern_status);
      digitalWrite(left_lantern_pin, lantern_status);
      lantern_set = true;
    }
  }

  else {
    lantern_set = false;
  }

  if (PS4.getButtonPress(CIRCLE)) {
    digitalWrite(right_low_stop_pin, LOW);
    digitalWrite(left_low_stop_pin, LOW);
  }

  else {
    digitalWrite(right_low_stop_pin, HIGH);
    digitalWrite(left_low_stop_pin, HIGH);
  }
}
