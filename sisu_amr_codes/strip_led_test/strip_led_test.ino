#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <TimerOne.h>

#define neopixel_pin 2

#define NUMPIXELS 240
#define num_strip_led 180
#define num_floating_led 20

#define I2C_COMM_ADDRESS 0x08

Adafruit_NeoPixel pixels(NUMPIXELS, neopixel_pin, NEO_GRB + NEO_KHZ800);

int neopixel_row = 0;

char strip_led_color = 1;

void setNeoPixel() {

  pixels.clear();

  switch (strip_led_color) {

    case 0:
      for (int i = 0; i < num_floating_led; i++) {
        pixels.setPixelColor((neopixel_row + i) % (num_strip_led), pixels.Color(0, 0, 0));
      }
      break;

    case 1:
      for (int i = 0; i < num_floating_led; i++) {
        pixels.setPixelColor((neopixel_row + i) % (num_strip_led), pixels.Color(0, 0, 255));
      }
      break;

    case 2:
      for (int i = 0; i < num_floating_led; i++) {
        pixels.setPixelColor((neopixel_row + i) % (num_strip_led), pixels.Color(0, 255, 0));
      }
      break;

    case 3:
      for (int i = 0; i < num_floating_led; i++) {
        pixels.setPixelColor((neopixel_row + i) % (num_strip_led), pixels.Color(0, 255, 255));
      }
      break;

    case 4:
      for (int i = 0; i < num_floating_led; i++) {
        pixels.setPixelColor((neopixel_row + i) % (num_strip_led), pixels.Color(255, 0, 0));
      }
      break;

    case 5:
      for (int i = 0; i < num_floating_led; i++) {
        pixels.setPixelColor((neopixel_row + i) % (num_strip_led), pixels.Color(255, 0, 255));
      }
      break;

    case 6:
      for (int i = 0; i < num_floating_led; i++) {
        pixels.setPixelColor((neopixel_row + i) % (num_strip_led), pixels.Color(255, 255, 0));
      }
      break;

    case 7:
      for (int i = 0; i < num_floating_led; i++) {
        pixels.setPixelColor((neopixel_row + i) % (num_strip_led), pixels.Color(255, 255, 255));
      }
      break;
  }

  pixels.fill(pixels.Color(255, 255, 255), 180, 60);

  pixels.show();

  neopixel_row++;
  neopixel_row %= num_strip_led;
}

void receiveEvent() {
  while (Wire.available()) {
    char val = Wire.read();
    if (val >= '0' and val <= '7')
      strip_led_color = val - '0';
  }
}

void setup() {

  Serial.begin(115200);

  /*Wire.begin(I2C_COMM_ADDRESS);
  Wire.onReceive(receiveEvent);*/

  pixels.begin();
  pixels.clear();
  pixels.fill(pixels.Color(255, 255, 255), 60, 60);
  pixels.show();

  /*Timer1.initialize(10000);
  Timer1.attachInterrupt(setNeoPixel);*/
}

void loop() {
  if (Serial.available() > 0) {
    char val = Serial.read();
    if (val >= '0' and val <= '7')
      strip_led_color = val - '0';
  }
  /*if (iletisim.available() > 0) {
    char val = iletisim.read();
    if (val >= '0' and val <= '7')
      strip_led_color = val - '0';
    }*/
}
