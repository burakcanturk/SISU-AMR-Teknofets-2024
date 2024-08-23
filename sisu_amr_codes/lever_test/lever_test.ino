#include <SoftwareSerial.h>
#include <HX711.h>

#define hc12_rx 2
#define hc12_tx 3

#define scale_clock_pin 4
#define scale_data_pin 5

#define mz80_pin 6

#define charge_pin A0

#define buzzer_pin 11

SoftwareSerial hc12(hc12_rx, hc12_tx);
HX711 scale;

struct amrVehicleReceiving {
  bool mz80_val;
  float mass_g;
  int charge_val;
} amr_vehicle_data;

void setup() {

  Serial.begin(115200);

  hc12.begin(9600);

  scale.begin(scale_data_pin, scale_clock_pin);
  //scale.set_offset(4294112253);
  scale.set_scale(5.350247);
  scale.tare(20);
  while (not scale.is_ready());

  pinMode(mz80_pin, INPUT);

  pinMode(charge_pin, INPUT);

  pinMode(buzzer_pin, OUTPUT);

  Serial.println("System started.");
}

void loop() {

  static bool time_set = false;
  static unsigned long begin_time = millis();

  float mass_g = scale.get_units(1);
  mass_g = mass_g < 0 ? 0 : mass_g;
  bool mz80_val = digitalRead(mz80_pin);
  int charge_val = analogRead(charge_pin);

  amr_vehicle_data.mz80_val = mz80_val;
  amr_vehicle_data.mass_g = mass_g;
  amr_vehicle_data.charge_val = charge_val;

  if (hc12.available() > 0) {
    time_set = false;
    digitalWrite(buzzer_pin, LOW);
    char deger = hc12.read();
    if (deger == '+') {
      hc12.write((uint8_t*)&amr_vehicle_data, sizeof(amr_vehicle_data));
    }
  }

  else {
    if (not time_set) {
      begin_time = millis();
      time_set = true;
    }
    if (millis() - begin_time >= 60000) {
      if (millis() % 500 < 250) digitalWrite(buzzer_pin, HIGH);
      else digitalWrite(buzzer_pin, LOW);
    }
  }

  Serial.print("MZ80: ");
  Serial.print(mz80_val);
  Serial.print(" Scale: ");
  Serial.print(mass_g);
  Serial.print(" charge: ");
  Serial.println(charge_val);
}
