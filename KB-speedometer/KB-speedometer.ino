/*\
   KidBright Speed Detector

   -------------
   Pin connnect
   ----------------------------------------------------------------
   KidBright    VNCL4200(KB Chain Proximiter)  VL53L0X(GYVL53L0XV2)
   ----------------------------------------------------------------
   5V                       5V                         5V
   3V3                      3V3                        3V3
   SDA0 (21)                SDA                        SDA
   SCL0 (22)                SCL                        SCL
   GND                      GND                        GND
   ----------------------------------------------------------------

   ref : http://electron6.phys.utk.edu/101/CH1/speed_and_velocity.htm

*/

#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"

Adafruit_8x16minimatrix matrix = Adafruit_8x16minimatrix();
Adafruit_VL53L0X lox = Adafruit_VL53L0X();

//TwoWire Wire1;/

#define SENSOR_ADDR 0x51 //vcnl4200
#define distance_m 0.3 // 30 cm to meter (30/100 = 0.3)

boolean GotFirstSensor = false, GotSecondSensor = false;
int first_sensor_distance = 0, secound_sensor_distance  = 0;
unsigned long t1_millis = 0, t2_millis = 0;
double time_ms, time_s;
double KMH;
double velocity;

void write_register(int device_addr, int register_num, int low_byte, int high_byte)
{
  Wire1.beginTransmission(device_addr);
  Wire1.write(register_num);
  Wire1.write(low_byte);
  Wire1.write(high_byte);
  Wire1.endTransmission(false);
}

void read_first_sensor() {
  int reg = 0x8;
  Wire1.beginTransmission(SENSOR_ADDR);
  Wire1.write(reg);
  Wire1.endTransmission(false);
  byte data[2] = {0, 0};
  Wire1.requestFrom(SENSOR_ADDR, 2);
  data[0] = Wire1.read();
  data[1] = Wire1.read();
  first_sensor_distance = int(data[1]) * 256 + int(data[0]);
  //  Serial.print("first_sensor_distance = ");/
  //  Serial.println(first_sensor_distance);/
}

void read_secound_sensor() {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  secound_sensor_distance = measure.RangeMilliMeter;
  //  Serial.print("secound_sensor_distance = ");/
  //  Serial.println(secound_sensor_distance);/
}

void setup() {

  Serial.begin(115200);
  Wire.begin(21, 22);
  Wire1.begin(4, 5);
  matrix.begin(0x70);
  matrix.setRotation(1);
  matrix.setTextSize(1);
  matrix.setTextColor(LED_ON);

  for (int8_t x = 0; x >= -128; x--) {
    matrix.clear();
    matrix.setCursor(x, 0);
    matrix.print("KidBright Speedometer");
    matrix.writeDisplay();
    delay(100);
  }
  matrix.clear();

  // init vcnl4200
  write_register(SENSOR_ADDR, 0x3, B00011010, B00001000);
  write_register(SENSOR_ADDR, 0x0, B01000001, B00000000);
  write_register(SENSOR_ADDR, 0x6, B00010000, B00000000);
  write_register(SENSOR_ADDR, 0x4, B01110000, B00000111);

  // init vl53l0x
  lox.begin();

}

void loop() {
  read_first_sensor();
  read_secound_sensor();

  if (first_sensor_distance > 200 && GotSecondSensor == false) {
    //    t1_millis = millis();
    t1_millis = micros();
    GotFirstSensor = true;
  }

  if (secound_sensor_distance < 150 && GotFirstSensor == true) {
    //    t2_millis = millis();
    t2_millis = micros();
    GotSecondSensor = true;
  }

  if (GotFirstSensor == true && GotSecondSensor == true) {

    Serial.print("t1_us ");
    Serial.print(t1_millis);
    Serial.print(" t2_millis ");
    Serial.println(t2_millis);

    time_ms = t2_millis - t1_millis;
    Serial.print("t micro ");
    Serial.println(time_ms);

    time_ms = time_ms / 1000;  // convert microsecond to millisecond
    Serial.print("time_ms ");
    Serial.println(time_ms);

    time_s = time_ms / 1000; // convert millisecond to second
    Serial.print("time_s ");
    Serial.println(time_s);

    KMH = (distance_m / time_s); // speed = distance / time
    Serial.print("distance_m / time_s ");
    Serial.println(KMH);

    KMH = KMH * 3600; // 3600 = 60*60 seconds per hr
    Serial.print("Meter / Hour ");
    Serial.println(KMH);

    KMH = KMH / 1000; // meters per Km

    Serial.print("speed = ");
    Serial.print(KMH);
    Serial.print(" km/h");
    Serial.print("\n");

    for (int8_t x = 0; x >= -100; x--) {
      matrix.clear();
      matrix.setCursor(x, 0);
      matrix.print("Speed = ");
      matrix.print(KMH);
      matrix.print(" km/h");
      matrix.writeDisplay();
      delay(100);
    }
    matrix.clear();

    // reset to detector speed again

    GotFirstSensor = false;
    GotSecondSensor = false;
  } else {
    matrix.setCursor(0, 0);
    matrix.print("  ");
    matrix.writeDisplay();
    Serial.print("1st = ");
    Serial.print(first_sensor_distance);
    Serial.print(" 2st = ");
    Serial.println(secound_sensor_distance);
  }
}
