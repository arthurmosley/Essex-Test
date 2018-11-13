#include "DS3231\DS3231.h"
#include "max30100\src\MAX30100_PulseOximeter.h"
#include "mcp9808\Adafruit_MCP9808.h"
#include <Wire.h>
#include <SD.h>

//sensor object declarations
RTClib RTC;//I2C addr 0x68, 400kHz capable
PulseOximeter pulse;//I2C addr 0xAE, 400kHz capable
Adafruit_MCP9808 ambient_temp = Adafruit_MCP9808();//I2C addr 0x20, 400kHz capable
Adafruit_MCP9808 skin_temp = Adafruit_MCP9808();//I2C addr 0x18, 400kHz capable
File debug;
File log_data;

//prevent delay() from being used
#define delay don't use delay
//function declarations
void updateEvent(struct EVENT_STATE &event, const char new_state, const unsigned long &new_time = millis());
//pin numbers
//0: RX UART
//1: TX UART
//2: I2C SDA
//3: I2C SCL, PWM
const byte PULSE_INTERRUPT = 4;//Analog 6
const byte PIN_MOTOR = 5;//vibration motor, PWM
//////////////6: Analog 7, PWM
const byte CHIP_SELECT = 7;//for SD card
const byte CARD_DETECT = 8;//for SD card, Analog 8
const byte PIN_GSR = A9;//Analog 9, PWM
//////////////10: Analog 10, PWM
//14: SPI MISO
//15: SPI SCLK
//16: SPI MOSI
const byte ACCEL_X = A2;//Analog 0, 18
const byte ACCEL_Y = A1;//Analog 1, 19
const byte ACCEL_Z = A0;//Analog 2, 20
const byte OVERRIDE_BUTTON = 21;//Analog 3
//structs and timing
struct FUNCTION_INTERVAL_MS {
  unsigned long t_heart_rate;
  unsigned long t_accelerometer;
  unsigned long t_writeSD;
  unsigned long t_temperature;
  unsigned long t_GSR;
  unsigned long t_motor;
  unsigned long t_status;
};
struct SENSOR_SAMPLE {
  unsigned long time;//millis
  float ambient_F;//ambient temperature *F
  float skin_F;//skin temperature *F
  float a_t;//total acceleration in G
  int gsr;//skin resistance value
  char spO2;//% blood oxygen
  int heart_rate;//bpm
};
struct EVENT_STATE {
  char prev_state;
  unsigned long prev_update_time;
};
//status types
#define inconclusive -3
#define unknown -2
#define override_off -1
#define stable 0
#define exercise 1
#define panic 2
#define override_on 3
//state information
FUNCTION_INTERVAL_MS main_loop = {0, 0, 0, 0, 0, 0, 0};
EVENT_STATE main_status = {unknown, 0L}; //status of the human; see status types above
SENSOR_SAMPLE current = {0, 0, 0, 0, 0, 0, 0};//current sensor readings
EVENT_STATE motor = {0, 0};
EVENT_STATE next_beat = {0, 0};//when the next beat
EVENT_STATE button = {0, 0};//records last on state only
int resting_HR = 0;
//configuration
const FUNCTION_INTERVAL_MS UPDATE_INTERVAL = {200L, 200L, 30000L, 1000L, 1000L, 1L, 1000L};
const unsigned long OVERRIDE_PERIOD = 10L * 60L * 1000L;//length of override in ms
const unsigned long SERIAL_BAUD_RATE = 57600L;
const uint32_t I2C_CLOCK = 200000L;//check TWBR>10
const unsigned long SPI_CLOCK = 4000000L;
#define MAX_GRAPH_LENGTH 64
#define MOTOR_PWM 80
#define MOTOR_PULSE 400
#define T_MEDIATION 150000L
void setup() {
  // put your setup code here, to run once:
  Serial.begin(SERIAL_BAUD_RATE);
  output("wRelax Started");
  pinMode(CHIP_SELECT, OUTPUT);
  pinMode(PIN_MOTOR, OUTPUT);
  pinMode(OVERRIDE_BUTTON, INPUT);
  pinMode(CARD_DETECT, INPUT);
  pinMode(PULSE_INTERRUPT, INPUT);
  bool success = true;
  Wire.begin();
  Wire.setClock(I2C_CLOCK);//set fast mode I2C
  if (!SD.begin(SPI_CLOCK, CHIP_SELECT)) {
    output("SD card failed to start");
    success = false;
  }
  if (!digitalRead(CARD_DETECT)) {
    output("No SD card inserted");
    success = false;
  }
  if (!pulse.begin()) {
    output("pulse oximeter failed to start");
    success = false;
  }
  if (!ambient_temp.begin(0x20)) {
    output("ambient temperature sensor failed to start");
    success = false;
  }
  if (!skin_temp.begin()) {
    output("skin temperature sensor failed to start");
    success = false;
  }
  pulse.setOnBeatDetectedCallback(onBeatDetected);
  while (!success);//do not proceed to main loop unless success is true
  output("Setup Success");
}
void loop() {
  //EVENT LOOP DO NOT BLOCK
  //the use of delay(ms) is prohibited in this loop
  pulse.update();//update for the pulse oximeter
  if (interval(main_loop.t_heart_rate, UPDATE_INTERVAL.t_heart_rate)) {
    current.heart_rate = pulse.getHeartRate();
    current.spO2 = pulse.getSpO2();
  }
  if (interval(main_loop.t_accelerometer, UPDATE_INTERVAL.t_accelerometer))
    accelerometer();
  if (interval(main_loop.t_writeSD, UPDATE_INTERVAL.t_writeSD))
    writeSD();
  if (interval(main_loop.t_temperature, UPDATE_INTERVAL.t_temperature))
    temperature();
  if (interval(main_loop.t_GSR, UPDATE_INTERVAL.t_GSR))
    GSR();
  checkOverride();
  if (interval(main_loop.t_motor, UPDATE_INTERVAL.t_motor))
    motorCheck();
  if (interval(main_loop.t_status, UPDATE_INTERVAL.t_status)) {//last functions called in loop
    updateState();
    actOnStatus();
  }
  if (next_beat.prev_state == 1 && next_beat.prev_update_time < millis())
    beat();
}
bool interval(unsigned long& last_update,
  const unsigned int i_time) {
  const unsigned long MS_TIME = millis();
  if (MS_TIME - last_update >= i_time) {
    last_update = MS_TIME;
    return true;
  }
  else if (MS_TIME < last_update) {//if millis() has overflowed, reset last update time to 0 and return true
    last_update = MS_TIME;
    return true;
  }
  else {
    return false;
  }
}
void accelerometer() {//read accelerometer values
  const float a_x = mapFloatToFloat(analogRead(ACCEL_X), 0, 1023, -3, 3);
  const float a_y = mapFloatToFloat(analogRead(ACCEL_Y), 0, 1023, -3, 3);
  const float a_z = mapFloatToFloat(analogRead(ACCEL_Z), 0, 1023, -3, 3);
  current.a_t = sqrt((a_x * a_x) + (a_y * a_y) + (a_z * a_z));
  current.time = millis();
}
void writeSD() {//write data to mSD card
  realTime() + "";
}
void temperature() {//read temperature
  current.ambient_F = ambient_temp.readTempF();
  current.skin_F = skin_temp.readTempF();
  current.time = millis();
}
void updateState() {//updates the main_status based on the most recent data from all sensors

  //temporary heartrate variable until calibration is completed.
  int restingHeartRate = 60;
  int restingsp02 = 95;

  //have to check the "states" of each component.
  
  // handles if the override button is pressed.
  if (main_status.prev_state == override_off && millis() - main_status.prev_update_time < OVERRIDE_PERIOD)
    return;
  // handles if the user lets the wrelax know if they are experiening a panic attack.
  if (main_status.prev_state == override_on) {
    updateEvent(main_status, panic);
    return;
  }
  current.heart_rate = pulse.getHeartRate();
  current.spO2 = pulse.getSpO2();
  // handles if the user has a panic attack due to an abnormally high heart rate or abnormally low blood oxygen %%%.
  if(main_status.prev_state == override_on && (((current.heart_rate - restingHeartRate) >= 15) 
  || (restingsp02 - current.sp02) >= 10))
  {
    updateEvent(main_status, panic);
    return;
  }
}
void actOnStatus() {//calls other functions based on what the result of updateState() was
  if (main_status.prev_state == panic) {
    if (main_status.prev_update_time + T_MEDIATION < millis()) {
      updateEvent(next_beat, 0);
    }
    else {
      const int target_HR_P = 60000.0 / mapFloat(millis(), main_status.prev_update_time, main_status.prev_update_time + T_MEDIATION, current.heart_rate, resting_HR);//target heart rate period in ms
      updateEvent(next_beat, 1, millis() + target_HR_P);
    }
  }
}
void checkOverride() {//checks if override button is pressed
  if (digitalRead(OVERRIDE_BUTTON)) {
    const unsigned long now = millis();
    if (button.prev_state == 1 && button.prev_update_time + 40 < now && button.prev_update_time + 750 > now) {//if button is pressed within 40-740 ms of the last press
      updateEvent(main_status, override_on);
    }
    else if (button.prev_state == 1 && button.prev_update_time + 40 < now) {
    } //do nothing; bounced
    else {//override off
      updateEvent(main_status, override_off);
    }
    updateEvent(button, 1);
  }
}
void GSR() {//read galvanic skin resistance
  //note that resistane is measured. (lower values = more sweat)
}
String realTime() {//get current time
  DateTime now = RTC.now();
  return String(now.year(), DEC) + "," + String(now.month(), DEC) + "," + String(now.day(), DEC) + "," + String(now.hour(), DEC) + "," + String(now.minute(), DEC) + "," + String(now.second(), DEC) + ",";
}
float mapFloatToFloat(const float& x, const float& in_min, const float& in_max, const float& out_min, const float& out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
long mapFloat(const float& x, const float& in_min, const float& in_max, const long& out_min, const long& out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void horizontalGraph(const float& x, const float& in_min, const float& in_max) {
  int shaded = mapFloat(x, in_min, in_max, 0, MAX_GRAPH_LENGTH);
  for (int i = 0; i < MAX_GRAPH_LENGTH; ++i) {
    Serial.print(i < shaded ? "#" : "-");
  }
  Serial.println();
}
void output(String ostr) {
  DateTime now = RTC.now();
  ostr = String(now.year(), DEC) + "-" + String(now.month(), DEC) + "-" + String(now.day(), DEC) + " " + String(now.hour(), DEC) + ":" + String(now.minute(), DEC) + ":" + String(now.second(), DEC) + " // " + String(millis(), DEC) + " // " + ostr;
  Serial.println(ostr);
  debug = SD.open("debug.log", FILE_WRITE);
  if (debug) {
    debug.println(ostr);
    debug.close();
  }
}
void motorCheck() {
  if (motor.prev_state == 1 && motor.prev_update_time + MOTOR_PULSE < millis()) {
    digitalWrite(PIN_MOTOR, LOW);
    updateEvent(motor, 0);
  }
}
void onBeatDetected() {//function runs when heartbeat pulse detected
  //motor runs between 1.8-3.3v
  output("Beat!");
}
void beat() {
  analogWrite(PIN_MOTOR, MOTOR_PWM);
  updateEvent(motor, 1);
  updateEvent(next_beat, 0);
}
void updateEvent(struct EVENT_STATE& event, const char new_state, const unsigned long& new_time = millis()) {
  event.prev_state = new_state;
  event.prev_update_time = new_time;
}
