#include <PinChangeInterrupt.h>
#include <Wire.h>
#include <INA226_WE.h>
#include <MPU6050_tockn.h>
#include <MadgwickAHRS.h>
#include <Servo.h>
#include <TimerOne.h>
#include <TimerThree.h>
#include <TimerFour.h>
#include <TimerFive.h>

#define right_hall_A_pin A10 //SARI
#define right_hall_B_pin A11 //YESIL
#define right_hall_C_pin A12 //MAVI
#define right_motor_speed_pin 4
#define right_motor_directifront_pin 22
#define right_motor_low_speed_pin 23
#define right_motor_high_speed_pin 24
#define right_motor_low_stop_pin 25

#define left_hall_A_pin A13 //SARI
#define left_hall_B_pin A14 //YESIL
#define left_hall_C_pin A15 //MAVI
#define left_motor_speed_pin 5
#define left_motor_directifront_pin 26
#define left_motor_low_speed_pin 27
#define left_motor_high_speed_pin 28
#define left_motor_low_stop_pin 29

#define right_lineer_speed_pin 6
#define right_lineer_upper_pin 30
#define right_lineer_lower_pin 31

#define left_lineer_speed_pin 8
#define left_lineer_upper_pin 32
#define left_lineer_lower_pin 33

#define front_left_line_pin A0
#define front_middle_line_pin A1
#define front_right_line_pin A2

#define middle_left_line_pin A3
#define middle_middle_line_pin A4
#define middle_right_line_pin A5

#define middle_distance_trig_pin 34
#define middle_distance_echo_pin 18

#define right_distance_trig_pin 35
#define right_distance_echo_pin 3

#define left_distance_trig_pin 36
#define left_distance_echo_pin 2

#define right_lantern_pin 37
#define left_lantern_pin 38

#define buzzer_pin 39

#define ldr_pin A8

#define leverPort Serial3

#define INA226_I2C_ADDRESS 0x40

#define lineer_speed 255

#define SERVO_MIN 500
#define SERVO_MAX 2500
#define SERVO_STOP 500

#define RECEIVING_RIGHT_MOTOR_VAL_ROW 0
#define RECEIVING_LEFT_MOTOR_VAL_ROW 1
#define RECEIVING_RIGHT_MOTOR_DIRECTION_ROW 2
#define RECEIVING_LEFT_MOTOR_DIRECTION_ROW 3
#define RECEIVING_RIGHT_MOTOR_SPEED_MODE_ROW 4
#define RECEIVING_LEFT_MOTOR_SPEED_MODE_ROW 5
#define RECEIVING_RIGHT_MOTOR_STOP_ROW 6
#define RECEIVING_LEFT_MOTOR_STOP_ROW 7
#define RECEIVING_RIGHT_LINEER_SPEED_ROW 8
#define RECEIVING_LEFT_LINEER_SPEED_ROW 9
#define RECEIVING_RIGHT_LINEER_DIRECTION_ROW 10
#define RECEIVING_LEFT_LINEER_DIRECTION_ROW 11
#define RECEIVING_RIGHT_LANTERN_VAL_ROW 12
#define RECEIVING_LEFT_LANTERN_VAL_ROW 13
#define RECEIVING_STRIP_LED_VAL_ROW 14
#define RECEIVING_BUZZER_VAL_ROW 15
#define RECEIVING_DIRECTION_RESET_ROW 16

#define SENDING_RIGHT_MOTOR_SPEED_ROW 128
#define SENDING_LEFT_MOTOR_SPEED_ROW 129
#define SENDING_FRONT_LEFT_LINE_VAL_ROW 130
#define SENDING_FRONT_MIDDLE_LINE_VAL_ROW 131
#define SENDING_FRONT_RIGHT_LINE_VAL_ROW 132
#define SENDING_CENTER_LEFT_LINE_VAL_ROW 133
#define SENDING_CENTER_MIDDLE_LINE_VAL_ROW 134
#define SENDING_CENTER_RIGHT_LINE_VAL_ROW 135
#define SENDING_LEFT_DISTANCE_VAL_ROW 136
#define SENDING_MIDDLE_DISTANCE_VAL_ROW 137
#define SENDING_RIGHT_DISTANCE_VAL_ROW 138
#define SENDING_VOLTAGE_VAL_ROW 139
#define SENDING_CURRENT_VAL_ROW 140
#define SENDING_MASS_VAL_ROW 141
#define SENDING_UPPER_DISTANCE_VAL_ROW 142
#define SENDING_UPPER_BOX_REM_CHRG_ROW 143
#define SENDING_ROLL_VAL_ROW 144
#define SENDING_PITCH_VAL_ROW 145
#define SENDING_YAW_VAL_ROW 146
#define SENDING_LDR_VAL_ROW 147

#define RIGHT_MOTOR_SPEED_MODE 0
#define LEFT_MOTOR_SPEED_MODE 1

#define MOTOR_SPEED_MODE_NORMAL 0
#define MOTOR_SPEED_MODE_SLOW 1
#define MOTOR_SPEED_MODE_FAST 2

#define RIGHT_LINEER_HEIGHT 0
#define LEFT_LINEER_HEIGHT 1

#define LINEER_HEIGHT_NORMAL 0
#define LINEER_HEIGHT_LOW 1
#define LINEER_HEIGHT_HIGH 2

INA226_WE ina226(INA226_I2C_ADDRESS);
MPU6050 mpu6050(Wire);
Madgwick filter;
Servo right_motor, left_motor;

float right_speed = 0.0;
long right_counter = 0;
char right_hall_before = 0;
char right_hall_begin = 0;
unsigned long right_time_start = 0;

float left_speed = 0.0;
long left_counter = 0;
char left_hall_before = 0;
char left_hall_begin = 0;
unsigned long left_time_start = 0;

float left_distance, middle_distance, right_distance;
unsigned long left_dis_time_start, middle_dis_time_start, right_dis_time_start;
bool left_dis_int_control = false, middle_dis_int_control = false, right_dis_int_control = false;

int neopixel_row = 0;

struct leverData {
  bool mz80_val;
  float mass_g;
  int charge_val;
} lever_data;

struct amrVehicleReceiving {
  uint16_t right_motor_val;
  uint16_t left_motor_val;
  uint8_t right_motor_direction;
  uint8_t left_motor_direction;
  uint8_t right_motor_speed_mode;
  uint8_t left_motor_speed_mode;
  uint8_t right_motor_stop;
  uint8_t left_motor_stop;
  uint8_t right_lineer_val;
  uint8_t left_lineer_val;
  uint8_t right_lineer_direction;
  uint8_t left_lineer_direction;
  uint8_t right_lantern_val;
  uint8_t left_lantern_val;
  uint8_t strip_led_val;
  uint8_t buzzer_val;
  uint8_t direction_reset_val;
} amr_vehicle_receiving;

struct amrVehicleSending {
  float right_motor_speed;
  float left_motor_speed;
  uint16_t front_left_line_val;
  uint16_t front_middle_line_val;
  uint16_t front_right_line_val;
  uint16_t middle_left_line_val;
  uint16_t middle_middle_line_val;
  uint16_t middle_right_line_val;
  float left_distance_val;
  float middle_distance_val;
  float right_distance_val;
  float voltage;
  float current;
  float mass;
  uint8_t upper_distance_val;
  uint16_t rem_chrg_upper_box_val;
  float roll;
  float pitch;
  float yaw;
  uint16_t ldr_val;
} amr_vehicle_sending;

void updateRightHallA() {

  if (right_counter == 0) {
    right_time_start = millis();
  }

  if (right_hall_before == 'B') {
    right_counter--;
  }

  else if (right_hall_before == 'C') {
    right_counter++;
  }

  if (right_counter == 3) {
    right_speed = 1000.0 / (millis() - right_time_start);
    right_counter = 0;
  }

  else if (right_counter == -3) {
    right_speed = -1000.0 / (millis() - right_time_start);
    right_counter = 0;
  }

  right_hall_before = 'A';
}

void updateRightHallB() {

  if (right_counter == 0) {
    right_time_start = millis();
  }

  if (right_hall_before == 'A') {
    right_counter--;
  }

  else if (right_hall_before == 'C') {
    right_counter++;
  }

  if (right_counter == 3) {
    right_speed = 1000.0 / (millis() - right_time_start);
    right_counter = 0;
  }

  else if (right_counter == -3) {
    right_speed = -1000.0 / (millis() - right_time_start);
    right_counter = 0;
  }

  right_hall_before = 'B';
}

void updateRightHallC() {

  if (right_counter == 0) {
    right_time_start = millis();
  }

  if (right_hall_before == 'A') {
    right_counter--;
  }

  else if (right_hall_before == 'B') {
    right_counter++;
  }

  if (right_counter == 3) {
    right_speed = 1000.0 / (millis() - right_time_start);
    right_counter = 0;
  }

  else if (right_counter == -3) {
    right_speed = -1000.0 / (millis() - right_time_start);
    right_counter = 0;
  }

  right_hall_before = 'C';
}

void updateLeftHallA() {

  if (left_counter == 0) {
    left_time_start = millis();
  }

  if (left_hall_before == 'B') {
    left_counter--;
  }

  else if (left_hall_before == 'C') {
    left_counter++;
  }

  if (left_counter == 3) {
    left_speed = 1000.0 / (millis() - left_time_start);
    left_counter = 0;
  }

  else if (left_counter == -3) {
    left_speed = -1000.0 / (millis() - left_time_start);
    left_counter = 0;
  }

  left_hall_before = 'A';
}

void updateLeftHallB() {

  if (left_counter == 0) {
    left_time_start = millis();
  }

  if (left_hall_before == 'A') {
    left_counter--;
  }

  else if (left_hall_before == 'C') {
    left_counter++;
  }

  if (left_counter == 3) {
    left_speed = 1000.0 / (millis() - left_time_start);
    left_counter = 0;
  }

  else if (left_counter == -3) {
    left_speed = -1000.0 / (millis() - left_time_start);
    left_counter = 0;
  }

  left_hall_before = 'B';
}

void updateLeftHallC() {

  if (left_counter == 0) {
    left_time_start = millis();
  }

  if (left_hall_before == 'A') {
    left_counter--;
  }

  else if (left_hall_before == 'B') {
    left_counter++;
  }

  if (left_counter == 3) {
    left_speed = 1000.0 / (millis() - left_time_start);
    left_counter = 0;
  }

  else if (left_counter == -3) {
    left_speed = -1000.0 / (millis() - left_time_start);
    left_counter = 0;
  }

  left_hall_before = 'C';
}

void motorSpeedControl() {
  if (millis() - right_time_start >= 1000) {
    right_counter = 0;
    right_speed = 0;
  }
  if (millis() - left_time_start >= 1000) {
    left_counter = 0;
    left_speed = 0;
  }
}

float getDistance(int trig_pin, int echo_pin) {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  int time = pulseIn(echo_pin, HIGH);
  float distance = time / 2 / 29.1;
  return distance;
}

void setLeftDistance() {
  if (left_dis_int_control) {
    unsigned long time = micros() - left_dis_time_start;
    left_distance = time / 2 / 29.1 - 8;
    left_dis_int_control = false;
  }
}

void setMiddleDistance() {
  if (middle_dis_int_control) {
    unsigned long time = micros() - middle_dis_time_start;
    middle_distance = time / 2 / 29.1 - 38;
    middle_dis_int_control = false;
  }
}

void setRightDistance() {
  if (right_dis_int_control) {
    unsigned long time = micros() - right_dis_time_start;
    right_distance = time / 2 / 29.1 - 38;
    right_dis_int_control = false;
  }
}

void distanceControl() {
  digitalWrite(left_distance_trig_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(left_distance_trig_pin, LOW);
  if (not left_dis_int_control) {
    left_dis_time_start = micros();
    left_dis_int_control = true;
  }
  digitalWrite(middle_distance_trig_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(middle_distance_trig_pin, LOW);
  if (not middle_dis_int_control) {
    middle_dis_time_start = micros();
    middle_dis_int_control = true;
  }
  digitalWrite(right_distance_trig_pin, HIGH);
  delayMicroseconds(5);
  digitalWrite(right_distance_trig_pin, LOW);
  if (not right_dis_int_control) {
    right_dis_time_start = micros();
    right_dis_int_control = true;
  }
}

void readMass() {

  static bool sent = false;

  if (not sent) {
    leverPort.write('+');
    sent = true;
  }

  if (leverPort.available() > 0) {
    leverPort.readBytes((uint8_t*)&lever_data, sizeof(lever_data));
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

void setMotorSpeedMode(int direction, int mode) {

  if (direction == RIGHT_MOTOR_SPEED_MODE) {

    switch (mode) {

      case MOTOR_SPEED_MODE_NORMAL:
        digitalWrite(right_motor_low_speed_pin, HIGH);
        digitalWrite(right_motor_high_speed_pin, HIGH);
        break;

      case MOTOR_SPEED_MODE_SLOW:
        digitalWrite(right_motor_low_speed_pin, LOW);
        digitalWrite(right_motor_high_speed_pin, HIGH);
        break;

      case MOTOR_SPEED_MODE_FAST:
        digitalWrite(right_motor_low_speed_pin, HIGH);
        digitalWrite(right_motor_high_speed_pin, LOW);
        break;
    }
  }

  else if (direction == LEFT_MOTOR_SPEED_MODE) {

    switch (mode) {

      case MOTOR_SPEED_MODE_NORMAL:
        digitalWrite(left_motor_low_speed_pin, HIGH);
        digitalWrite(left_motor_high_speed_pin, HIGH);
        break;

      case MOTOR_SPEED_MODE_SLOW:
        digitalWrite(left_motor_low_speed_pin, LOW);
        digitalWrite(left_motor_high_speed_pin, HIGH);
        break;

      case MOTOR_SPEED_MODE_FAST:
        digitalWrite(left_motor_low_speed_pin, HIGH);
        digitalWrite(left_motor_high_speed_pin, LOW);
        break;
    }
  }
}

void setLineerHeight(int direction, int height) {

  if (direction == RIGHT_LINEER_HEIGHT) {

    switch (height) {

      case LINEER_HEIGHT_NORMAL:
        digitalWrite(right_lineer_upper_pin, LOW);
        digitalWrite(right_lineer_lower_pin, LOW);
        break;

      case LINEER_HEIGHT_LOW:
        digitalWrite(right_lineer_upper_pin, LOW);
        digitalWrite(right_lineer_lower_pin, HIGH);
        break;

      case LINEER_HEIGHT_HIGH:
        digitalWrite(right_lineer_upper_pin, HIGH);
        digitalWrite(right_lineer_lower_pin, LOW);
        break;
    }
  }

  else if (direction == LEFT_LINEER_HEIGHT) {

    switch (height) {

      case LINEER_HEIGHT_NORMAL:
        digitalWrite(left_lineer_upper_pin, LOW);
        digitalWrite(left_lineer_lower_pin, LOW);
        break;

      case LINEER_HEIGHT_LOW:
        digitalWrite(left_lineer_upper_pin, LOW);
        digitalWrite(left_lineer_lower_pin, HIGH);
        break;

      case LINEER_HEIGHT_HIGH:
        digitalWrite(left_lineer_upper_pin, HIGH);
        digitalWrite(left_lineer_lower_pin, LOW);
        break;
    }
  }
}

void setStripLed(int row) {
  Wire.beginTransmission(0x08);
  Wire.write('0' + row);
  Wire.endTransmission();
}

float getVoltage() {
  ina226.readAndClearFlags();
  return ina226.getBusVoltage_V();
}

float getCurrent() {
  ina226.readAndClearFlags();
  return ina226.getCurrent_mA();
}

float getMass() {
  readMass();
  return lever_data.mass_g;
}

bool getUpperDistance() {
  readMass();
  return lever_data.mz80_val;
}

int getRemChrgUpperBox() {
  readMass();
  return lever_data.charge_val;
}

float getRoll() {

  mpu6050.update();

  float ax = mpu6050.getAccX();
  float ay = mpu6050.getAccY();
  float az = mpu6050.getAccZ();

  float gx = mpu6050.getGyroX();
  float gy = mpu6050.getGyroY();
  float gz = mpu6050.getGyroZ();

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  return filter.getRoll();
}

float getPitch() {

  mpu6050.update();

  float ax = mpu6050.getAccX();
  float ay = mpu6050.getAccY();
  float az = mpu6050.getAccZ();

  float gx = mpu6050.getGyroX();
  float gy = mpu6050.getGyroY();
  float gz = mpu6050.getGyroZ();

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  return filter.getPitch();
}

float getYaw() {

  mpu6050.update();

  float ax = mpu6050.getAccX();
  float ay = mpu6050.getAccY();
  float az = mpu6050.getAccZ();

  float gx = mpu6050.getGyroX();
  float gy = mpu6050.getGyroY();
  float gz = mpu6050.getGyroZ();

  filter.updateIMU(gx, gy, gz, ax, ay, az);

  return filter.getYaw();
}

void setup() {

  Wire.begin();

  Serial.begin(115200);

  pinMode(right_hall_A_pin, INPUT);
  pinMode(right_hall_B_pin, INPUT);
  pinMode(right_hall_C_pin, INPUT);
  right_motor.attach(right_motor_speed_pin);
  pinMode(right_motor_directifront_pin, OUTPUT);
  pinMode(right_motor_low_speed_pin, OUTPUT);
  pinMode(right_motor_high_speed_pin, OUTPUT);
  pinMode(right_motor_low_stop_pin, OUTPUT);

  pinMode(left_hall_A_pin, INPUT);
  pinMode(left_hall_B_pin, INPUT);
  pinMode(left_hall_C_pin, INPUT);
  left_motor.attach(left_motor_speed_pin);
  pinMode(left_motor_directifront_pin, OUTPUT);
  pinMode(left_motor_low_speed_pin, OUTPUT);
  pinMode(left_motor_high_speed_pin, OUTPUT);
  pinMode(left_motor_low_stop_pin, OUTPUT);

  pinMode(right_lineer_upper_pin, OUTPUT);
  pinMode(right_lineer_lower_pin, OUTPUT);
  pinMode(right_lineer_speed_pin, OUTPUT);

  pinMode(left_lineer_upper_pin, OUTPUT);
  pinMode(left_lineer_lower_pin, OUTPUT);
  pinMode(left_lineer_speed_pin, OUTPUT);

  pinMode(front_left_line_pin, INPUT);
  pinMode(front_middle_line_pin, INPUT);
  pinMode(front_right_line_pin, INPUT);

  pinMode(middle_left_line_pin, INPUT);
  pinMode(middle_middle_line_pin, INPUT);
  pinMode(middle_right_line_pin, INPUT);

  pinMode(left_distance_trig_pin, OUTPUT);
  pinMode(left_distance_echo_pin, INPUT);

  pinMode(middle_distance_trig_pin, OUTPUT);
  pinMode(middle_distance_echo_pin, INPUT);

  pinMode(right_distance_trig_pin, OUTPUT);
  pinMode(right_distance_echo_pin, INPUT);

  pinMode(right_lantern_pin, OUTPUT);
  pinMode(left_lantern_pin, OUTPUT);

  pinMode(buzzer_pin, OUTPUT);

  /*attachInterrupt(digitalPinToInterrupt(left_distance_echo_pin), setLeftDistance, FALLING);
    attachInterrupt(digitalPinToInterrupt(middle_distance_echo_pin), setMiddleDistance, FALLING);
    attachInterrupt(digitalPinToInterrupt(right_distance_echo_pin), setRightDistance, FALLING);*/

  attachPinChangeInterrupt(digitalPinToPCINT(right_hall_A_pin), updateRightHallA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(right_hall_B_pin), updateRightHallB, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(right_hall_C_pin), updateRightHallC, CHANGE);

  attachPinChangeInterrupt(digitalPinToPCINT(left_hall_A_pin), updateLeftHallA, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(left_hall_B_pin), updateLeftHallB, CHANGE);
  attachPinChangeInterrupt(digitalPinToPCINT(left_hall_C_pin), updateLeftHallC, CHANGE);

  right_motor.writeMicroseconds(SERVO_STOP);
  digitalWrite(right_motor_directifront_pin, HIGH);
  digitalWrite(right_motor_low_speed_pin, HIGH);
  digitalWrite(right_motor_high_speed_pin, HIGH);
  digitalWrite(right_motor_low_stop_pin, HIGH);

  left_motor.writeMicroseconds(SERVO_STOP);
  digitalWrite(left_motor_directifront_pin, HIGH);
  digitalWrite(left_motor_low_speed_pin, HIGH);
  digitalWrite(left_motor_high_speed_pin, HIGH);
  digitalWrite(left_motor_low_stop_pin, HIGH);

  analogWrite(right_lineer_speed_pin, 0);
  digitalWrite(right_lineer_upper_pin, LOW);
  digitalWrite(right_lineer_lower_pin, LOW);

  analogWrite(left_lineer_speed_pin, 0);
  digitalWrite(left_lineer_upper_pin, LOW);
  digitalWrite(left_lineer_lower_pin, LOW);

  digitalWrite(right_lantern_pin, HIGH);
  digitalWrite(left_lantern_pin, HIGH);

  digitalWrite(buzzer_pin, LOW);

  ina226.init();
  //ina226.setResistorRange(0.1, 1.3);
  //ina226.setCorrectionFactor(0.93);
  ina226.waitUntilConversionCompleted();

  mpu6050.begin();
  //mpu6050.calcGyroOffsets(true);

  //filter.begin(25);

  leverPort.begin(9600);

  Timer1.initialize(100000);
  Timer1.attachInterrupt(motorSpeedControl);

  /*Timer3.initialize(100000);
    Timer3.attachInterrupt(distanceControl);*/

  //Serial.println("Sistem baslatildi.");
}

void loop() {

  if (Serial.available() > 0) {

    byte receiving_row = Serial.read();

    switch (receiving_row) {

      case RECEIVING_RIGHT_MOTOR_VAL_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.right_motor_val, sizeof(amr_vehicle_receiving.right_motor_val));
        right_motor.writeMicroseconds(amr_vehicle_receiving.right_motor_val);
        Serial.write('+');
        break;

      case RECEIVING_LEFT_MOTOR_VAL_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.left_motor_val, sizeof(amr_vehicle_receiving.left_motor_val));
        left_motor.writeMicroseconds(amr_vehicle_receiving.left_motor_val);
        Serial.write('+');
        break;

      case RECEIVING_RIGHT_MOTOR_DIRECTION_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.right_motor_direction, sizeof(amr_vehicle_receiving.right_motor_direction));
        digitalWrite(right_motor_directifront_pin, not amr_vehicle_receiving.right_motor_direction);
        Serial.write('+');
        break;

      case RECEIVING_LEFT_MOTOR_DIRECTION_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.left_motor_direction, sizeof(amr_vehicle_receiving.left_motor_direction));
        digitalWrite(left_motor_directifront_pin, not amr_vehicle_receiving.left_motor_direction);
        Serial.write('+');
        break;

      case RECEIVING_RIGHT_MOTOR_SPEED_MODE_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.right_motor_speed_mode, sizeof(amr_vehicle_receiving.right_motor_speed_mode));
        setMotorSpeedMode(RIGHT_MOTOR_SPEED_MODE, amr_vehicle_receiving.right_motor_speed_mode);
        Serial.write('+');
        break;

      case RECEIVING_LEFT_MOTOR_SPEED_MODE_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.left_motor_speed_mode, sizeof(amr_vehicle_receiving.left_motor_speed_mode));
        setMotorSpeedMode(LEFT_MOTOR_SPEED_MODE, amr_vehicle_receiving.left_motor_speed_mode);
        Serial.write('+');
        break;

      case RECEIVING_RIGHT_MOTOR_STOP_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.right_motor_stop, sizeof(amr_vehicle_receiving.right_motor_stop));
        digitalWrite(right_motor_low_stop_pin, not amr_vehicle_receiving.right_motor_stop);
        Serial.write('+');
        break;

      case RECEIVING_LEFT_MOTOR_STOP_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.left_motor_stop, sizeof(amr_vehicle_receiving.left_motor_stop));
        digitalWrite(left_motor_low_stop_pin, not amr_vehicle_receiving.left_motor_stop);
        Serial.write('+');
        break;

      case RECEIVING_RIGHT_LINEER_SPEED_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.right_lineer_val, sizeof(amr_vehicle_receiving.right_lineer_val));
        analogWrite(right_lineer_speed_pin, amr_vehicle_receiving.right_lineer_val);
        Serial.write('+');
        break;

      case RECEIVING_LEFT_LINEER_SPEED_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.left_lineer_val, sizeof(amr_vehicle_receiving.left_lineer_val));
        analogWrite(left_lineer_speed_pin, amr_vehicle_receiving.left_lineer_val);
        Serial.write('+');
        break;

      case RECEIVING_RIGHT_LINEER_DIRECTION_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.right_lineer_direction, sizeof(amr_vehicle_receiving.right_lineer_direction));
        setLineerHeight(RIGHT_LINEER_HEIGHT, amr_vehicle_receiving.right_lineer_direction);
        Serial.write('+');
        break;

      case RECEIVING_LEFT_LINEER_DIRECTION_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.left_lineer_direction, sizeof(amr_vehicle_receiving.left_lineer_direction));
        setLineerHeight(LEFT_LINEER_HEIGHT, amr_vehicle_receiving.left_lineer_direction);
        Serial.write('+');
        break;

      case RECEIVING_RIGHT_LANTERN_VAL_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.right_lantern_val, sizeof(amr_vehicle_receiving.right_lantern_val));
        digitalWrite(right_lantern_pin, not amr_vehicle_receiving.right_lantern_val);
        Serial.write('+');
        break;

      case RECEIVING_LEFT_LANTERN_VAL_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.left_lantern_val, sizeof(amr_vehicle_receiving.left_lantern_val));
        digitalWrite(left_lantern_pin, not amr_vehicle_receiving.left_lantern_val);
        Serial.write('+');
        break;

      case RECEIVING_STRIP_LED_VAL_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.strip_led_val, sizeof(amr_vehicle_receiving.strip_led_val));
        setStripLed(amr_vehicle_receiving.strip_led_val);
        Serial.write('+');
        break;

      case RECEIVING_BUZZER_VAL_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.buzzer_val, sizeof(amr_vehicle_receiving.buzzer_val));
        digitalWrite(buzzer_pin, amr_vehicle_receiving.buzzer_val);
        Serial.write('+');
        break;

      case RECEIVING_DIRECTION_RESET_ROW:
        Serial.readBytes((uint8_t*)&amr_vehicle_receiving.direction_reset_val, sizeof(amr_vehicle_receiving.direction_reset_val));
        if (amr_vehicle_receiving.direction_reset_val) filter.begin(25);
        Serial.write('+');
        break;

      //-----------------------------------------------------------------

      case SENDING_RIGHT_MOTOR_SPEED_ROW:
        amr_vehicle_sending.right_motor_speed = right_speed;
        Serial.write((uint8_t*)&amr_vehicle_sending.right_motor_speed, sizeof(amr_vehicle_sending.right_motor_speed));
        break;

      case SENDING_LEFT_MOTOR_SPEED_ROW:
        amr_vehicle_sending.left_motor_speed = left_speed;
        Serial.write((uint8_t*)&amr_vehicle_sending.left_motor_speed, sizeof(amr_vehicle_sending.left_motor_speed));
        break;

      case SENDING_FRONT_LEFT_LINE_VAL_ROW:
        amr_vehicle_sending.front_left_line_val = analogRead(front_left_line_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.front_left_line_val, sizeof(amr_vehicle_sending.front_left_line_val));
        break;

      case SENDING_FRONT_MIDDLE_LINE_VAL_ROW:
        amr_vehicle_sending.front_middle_line_val = analogRead(front_middle_line_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.front_middle_line_val, sizeof(amr_vehicle_sending.front_middle_line_val));
        break;

      case SENDING_FRONT_RIGHT_LINE_VAL_ROW:
        amr_vehicle_sending.front_right_line_val = analogRead(front_right_line_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.front_right_line_val, sizeof(amr_vehicle_sending.front_right_line_val));
        break;

      case SENDING_CENTER_LEFT_LINE_VAL_ROW:
        amr_vehicle_sending.middle_left_line_val = analogRead(middle_left_line_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.middle_left_line_val, sizeof(amr_vehicle_sending.middle_left_line_val));
        break;

      case SENDING_CENTER_MIDDLE_LINE_VAL_ROW:
        amr_vehicle_sending.middle_middle_line_val = analogRead(middle_middle_line_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.middle_middle_line_val, sizeof(amr_vehicle_sending.middle_middle_line_val));
        break;

      case SENDING_CENTER_RIGHT_LINE_VAL_ROW:
        amr_vehicle_sending.middle_right_line_val = analogRead(middle_right_line_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.middle_right_line_val, sizeof(amr_vehicle_sending.middle_right_line_val));
        break;

      case SENDING_LEFT_DISTANCE_VAL_ROW:
        amr_vehicle_sending.left_distance_val = getDistance(left_distance_trig_pin, left_distance_echo_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.left_distance_val, sizeof(amr_vehicle_sending.left_distance_val));
        break;

      case SENDING_MIDDLE_DISTANCE_VAL_ROW:
        amr_vehicle_sending.middle_distance_val = getDistance(middle_distance_trig_pin, middle_distance_echo_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.middle_distance_val, sizeof(amr_vehicle_sending.middle_distance_val));
        break;

      case SENDING_RIGHT_DISTANCE_VAL_ROW:
        amr_vehicle_sending.right_distance_val = getDistance(right_distance_trig_pin, right_distance_echo_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.right_distance_val, sizeof(amr_vehicle_sending.right_distance_val));
        break;

      case SENDING_VOLTAGE_VAL_ROW:
        amr_vehicle_sending.voltage = getVoltage();
        Serial.write((uint8_t*)&amr_vehicle_sending.voltage, sizeof(amr_vehicle_sending.voltage));
        break;

      case SENDING_CURRENT_VAL_ROW:
        amr_vehicle_sending.current = getCurrent();
        Serial.write((uint8_t*)&amr_vehicle_sending.current, sizeof(amr_vehicle_sending.current));
        break;

      case SENDING_MASS_VAL_ROW:
        amr_vehicle_sending.mass = getMass();
        Serial.write((uint8_t*)&amr_vehicle_sending.mass, sizeof(amr_vehicle_sending.mass));
        break;

      case SENDING_UPPER_DISTANCE_VAL_ROW:
        amr_vehicle_sending.upper_distance_val = getUpperDistance();
        Serial.write((uint8_t*)&amr_vehicle_sending.upper_distance_val, sizeof(amr_vehicle_sending.upper_distance_val));
        break;

      case SENDING_UPPER_BOX_REM_CHRG_ROW:
        amr_vehicle_sending.rem_chrg_upper_box_val = getRemChrgUpperBox();
        Serial.write((uint8_t*)&amr_vehicle_sending.rem_chrg_upper_box_val, sizeof(amr_vehicle_sending.rem_chrg_upper_box_val));
        break;

      case SENDING_ROLL_VAL_ROW:
        amr_vehicle_sending.roll = getRoll();
        Serial.write((uint8_t*)&amr_vehicle_sending.roll, sizeof(amr_vehicle_sending.roll));
        break;

      case SENDING_PITCH_VAL_ROW:
        amr_vehicle_sending.pitch = getPitch();
        Serial.write((uint8_t*)&amr_vehicle_sending.pitch, sizeof(amr_vehicle_sending.pitch));
        break;

      case SENDING_YAW_VAL_ROW:
        amr_vehicle_sending.yaw = getYaw();
        Serial.write((uint8_t*)&amr_vehicle_sending.yaw, sizeof(amr_vehicle_sending.yaw));
        break;

      case SENDING_LDR_VAL_ROW:
        amr_vehicle_sending.ldr_val = analogRead(ldr_pin);
        Serial.write((uint8_t*)&amr_vehicle_sending.ldr_val, sizeof(amr_vehicle_sending.ldr_val));
        break;
    }
  }
}
