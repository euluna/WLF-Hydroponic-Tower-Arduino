/*
REQUIRES the Arduino libraries:
- DHT Sensor Library: https://github.com/adafruit/DHT-sensor-library
- Adafruit Unified Sensor Lib: https://github.com/adafruit/Adafruit_Sensor
- OneWire Library: https://github.com/PaulStoffregen/OneWire
- DallasTemperature: https://github.com/milesburton/Arduino-Temperature-Control-Library

PH Sensor Connections:
- Red Wire to 5V
- Yellow Wire to phPin (A0 by default)
- Black Wire to GND

Light Sensor (BH1750) Connections:
  - VCC to 3V3 or 5V
  - GND to GND
  - SCL to SCL (Digital Pin 21)
  - SDA to SDA (Digital Pin 20)
  - ADD to (not connected) or GND

ADD pin is used to set sensor I2C address. If it has voltage greater or equal
to 0.7VCC voltage (e.g. you've connected it to VCC) the sensor address will be
0x5C. In other case (if ADD voltage less than 0.7 * VCC) the sensor address
will be 0x23 (by default).

Humidity Sensor (DHT11) Connections:
  - VCC to 5V
  - DATA to DHTPIN (Digital Pin 3 by default)
  - GND to GND

Flow Sensor (GR108) Connections:
  - Red Wire (VCC+) to 5V
  - Yellow Wire (of sensor 1) to flowPin_1 (Digital Pin 5 by default)
  - Yellow Wire (of sensor 1) to flowPin_2 (Digital Pin 6 by default)
  - Yellow Wire (of sensor 2) to flowPin_3 (Digital Pin 7 by default)
  - Yellow Wire (of sensor 3) to flowPin_4 (Digital Pin 8 by default)
  - Black Wire (GND-) to GND

Flow Rate (Amazon Datasheet):
  - F=(5.5*Q)±2%
  Where:
  - F is frequency in Hz
  - Q is L/Min
  - 5.5 is Sensor Constant

///!NOTE! = tankHeight variable has been randomly set to 30cm
//tankHeight variable is vertical disctance (cm) from Ultrasonic sensor's face to bottom of tank
// refer to line 145 to change

Water Level Sensors (2):
  - Trig (Of sensor 1) to TRIG_1 (Digital Pin 9 by default)
  - Echo (of sensor 1) to ECHO_1 (Digtal Pin 10 by default)
  - Trig (of sensor 2) to TRIG_2 (Digital Pin 11 by default)
  - Echo (of sensor 2) to ECHO_2 (Digtal Pin 12 by default)
  - GND to GND
  - VCC to 5V

TDs sensor:
  - Yellow wire to A3
  - Black wire to GND
  - Red wire to 5V

Temperature sensor:
  - Dat to (Digtal Pin 2)
  - GND to GND
  - VCC to 5V

Warning LED:
- Input to Digital Pin 15

Pump Shutoff:
- Input to Digital Pin 16

*/

enum LevelState
{
  Normal = 0,
  Warn = 1,
  Shutoff = 2
};

#include "DHT.h"
#include <BH1750.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 2 // digital pin connected to DAT on sensor
#define DHTPIN 3       // alternatively, use pins 3, 4, 5, 12, 13, or 14
#define DHTTYPE DHT11
// waterlevel
#define TRIG_1 9  // sound pluse is to pin 9
#define ECHO_1 10 // echo is pin 10
#define TRIG_2 11 // sound pluse is to pin 9
#define ECHO_2 12 // echo is pin 10

LevelState levelState = Normal;
// FOR JACOB: Determine these values
const int warnLEDPin = 15;
const int pumpShutoffPin = 16;
float maxLevel = 50;                   // the max level to shutoff before the reservoir overflows
float highWarnLevel = 45;              // reservoir high water level warning
float lowWarnLevel = 10;               // reservoir low water level warning
float minLevel = 5;                    // the min level to shutoff before reservoir gets too low
const unsigned long levelDelay = 5000; // 5 second delay between waterlevel trigger and event firing (shutoff / warning)

int phPin = A0;
float phVolt;
float ph = 0;
// calibration constants
float slope = 4.87012897;
float offset = -2.306816462;

DHT dht(DHTPIN, DHTTYPE);
bool printLabels = true; // if CSV header should be included
float humPercent;
float ambTemp;

int flowPin_1 = 5;              // pin can be modified as desired
int flowPin_2 = 6;              // pin can be modified as desired
int flowPin_3 = 7;              // pin can be modified as desired
int flowPin_4 = 8;              // pin can be modified as desired
double flowRate_1;              // main value to record
double flowRate_2;              // main value to record
double flowRate_3;              // main value to record
double flowRate_4;              // main value to record
float flowSensorConstant = 5.5; // according to datasheet, should be 5.5 (uncalibrated)
volatile int flowCount_1;
volatile int flowCount_2;
volatile int flowCount_3;
volatile int flowCount_4;

BH1750 lightMeter_1;
// BH1750 lightMeter_2;
// BH1750 lightMeter_3;
int recordPeriod = 2; // delay between data readings, in seconds

float lux_1;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

int tdsPin = A3;
int tdsOutput = 0;

float tempC = 0;

float voltagetds = 0;
float ppm = 0;

// -------------------------------------------------------------
// NEW: The distance of the Ultrasonic sensor to bottom of reservoir
// Measure from the sensor face down to the bottom of the reservoir
// -------------------------------------------------------------
float tankHeight_1 = 30.0; // <-- NEW (change this value)
float tankHeight_2 = 30.0; // <-- NEW (change this value)

// functions
void Flow1();
void Flow2();
void Flow3();
void Flow4();
void initHardware();
void readEnvironment();
void readWaterQuality();
void readWaterLevel();
void processFlowRatesAndDelay();
void updateSystemControls();
void logData();

unsigned long levelTime = 0; // time when water level event was sensed

// Global variables needed for the refactored helper functions
unsigned long currTime = 0;
unsigned long lastRecordTime = 0;
float waterLevel_1; // <-- NEW Variable to store calculated water level
float waterLevel_2;
float avg_level;

void setup()
{
  initHardware();
  if (printLabels)
    Serial.println("Time (ms), PH, Light (lx), Humidity (%), Ambient Temp (°C), Liquid Temp (°C), TDS (ppm), Water Level 1 (cm), Water Level 2 (cm), Flow Rate 1 (L/min), Flow Rate 2 (L/min), Flow Rate 3 (L/min)"); // CSV header
}

void loop()
{
  currTime = millis(); // Update time at the very start

  readWaterLevel();
  updateSystemControls();

  // Runs only every "recordPeriod" (2 seconds)
  if (currTime - lastRecordTime >= (1000 * recordPeriod))
  {
    readEnvironment(); // FIX: No longer aborts the loop if this fails
    readWaterQuality();
    processFlowRates();
    logData();

    lastRecordTime = currTime; // Reset the timer
  }

  // Small delay to prevent ultrasonic echo overlap
  delay(50);
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

void initHardware()
{
  // Initialize the flow sensors
  pinMode(flowPin_1, INPUT); // set input pins
  pinMode(flowPin_2, INPUT);
  pinMode(flowPin_3, INPUT);
  pinMode(flowPin_4, INPUT);
  pinMode(warnLEDPin, OUTPUT);
  pinMode(pumpShutoffPin, OUTPUT);
  digitalWrite(warnLEDPin, LOW);
  digitalWrite(pumpShutoffPin, LOW); // initialize pump shutoff and LED as off
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(flowPin_1), Flow1, RISING);
  attachInterrupt(digitalPinToInterrupt(flowPin_2), Flow2, RISING);
  attachInterrupt(digitalPinToInterrupt(flowPin_3), Flow3, RISING);
  attachInterrupt(digitalPinToInterrupt(flowPin_4), Flow4, RISING);
  // Initialize the I2C bus (BH1750 library doesn't do this automatically)
  Wire.begin();
  // Wire2.begin(18,19);
  // Wire3.begin(16,17);
  lightMeter_1.begin();
  lightMeter_1.configure(BH1750::CONTINUOUS_HIGH_RES_MODE);
  // lightMeter_2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  // lightMeter_3.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire3);
  //  Initialize the DHT sensor
  dht.begin();
  // initialize tds
  sensors.begin();
  // waterlevel
  pinMode(TRIG_1, OUTPUT); // sound pulse is output
  pinMode(ECHO_1, INPUT);  // Echo is input
  pinMode(TRIG_2, OUTPUT); // sound pulse is output
  pinMode(ECHO_2, INPUT);  // Echo is input
  // pullup resistor temp sensor
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);
}

void readEnvironment()
{
  humPercent = dht.readHumidity();
  ambTemp = dht.readTemperature();

  // FIX:
  // If the DHT fails, we log it and exit THIS function early,
  // but we allow the rest of the system (like water shutoffs) to keep running.
  if (isnan(humPercent) || isnan(ambTemp))
  {
    // Serial.println("DHT Read Error");
    return;
  }

  // light
  lux_1 = lightMeter_1.readLightLevel();
  // lux_2 = lightMeter_2.readLightLevel();
  // lux_3 = lightMeter_3.readLightLevel();
}

void readWaterQuality()
{
  // ph
  int sensorValue = analogRead(phPin);
  phVolt = sensorValue * (5.0 / 1023.0); // convert reading to voltage
  ph = slope * phVolt + offset;

  // temp sensor
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0); // read first sensor

  // analog tds sensor
  tdsOutput = analogRead(tdsPin);
  voltagetds = tdsOutput * (5.0 / 1023.0);
  ppm = (voltagetds * 1000 / 500) * 1.1;
}

void readWaterLevel()
{
  long duration_1;
  long duration_2;
  float distance_1;
  float distance_2;

  // Trigger ultrasonic pulse
  digitalWrite(TRIG_1, LOW); // when pin is low no sound is emitted
  digitalWrite(TRIG_2, LOW); // when pin is low no sound is emitted
  delayMicroseconds(2);
  digitalWrite(TRIG_1, HIGH); // pulse sent
  digitalWrite(TRIG_2, HIGH); // pulse sent
  delayMicroseconds(10);
  digitalWrite(TRIG_1, LOW); // ends pulse
  digitalWrite(TRIG_2, LOW); // ends pulse

  // Measure return time
  duration_1 = pulseIn(ECHO_1, HIGH); // pulseIn() function measures time in this case the time for which the echo pin is high
  duration_2 = pulseIn(ECHO_2, HIGH);

  // Convert to distance (in cm)
  distance_1 = duration_1 * 0.034 / 2; // 0.034 is the speed of sound  in cm per mircosecond
  distance_2 = duration_2 * 0.034 / 2;

  // -------------------------------------------------------------
  // NEW: Calculate the actual water level in the reservoir
  // -------------------------------------------------------------
  waterLevel_1 = tankHeight_1 - distance_1; // <-- NEW
  waterLevel_2 = tankHeight_2 - distance_2;
  avg_level = (waterLevel_1 + waterLevel_2) / 2; // get average water level
}

void processFlowRates()
{
  // disable interrupts to safely copy and reset the volatile counters
  noInterrupts();
  int count1 = flowCount_1;
  flowCount_1 = 0;
  int count2 = flowCount_2;
  flowCount_2 = 0;
  int count3 = flowCount_3;
  flowCount_3 = 0;
  int count4 = flowCount_4;
  flowCount_4 = 0;
  interrupts(); // turn back on

  // convert total pulses to Frequency (Hz) based on elapsed time
  float hz_1 = (float)count1 / recordPeriod;
  float hz_2 = (float)count2 / recordPeriod;
  float hz_3 = (float)count3 / recordPeriod;
  float hz_4 = (float)count4 / recordPeriod;

  // calculate L/min (Q = F / 5.5)
  flowRate_1 = hz_1 / flowSensorConstant;
  flowRate_2 = hz_2 / flowSensorConstant;
  flowRate_3 = hz_3 / flowSensorConstant;
  flowRate_4 = hz_4 / flowSensorConstant;
}

void updateSystemControls()
{
  // ---------------------------------------------------------- SYSTEM CONTROLS -------------------------------------------------------
  // Send power to / away from hardware as needed

  if (avg_level >= maxLevel || avg_level <= minLevel) // we are above max level, or below min level
  {
    switch (levelState)
    {
    case Normal: // we were operating normally last iteration (sudden flood or sensor interference)
      // upgrade to warn mode, set timer
      levelState = Warn;
      levelTime = currTime;
      break;
    case Warn:
      // Upgrade to shutoff mode, set timer
      levelState = Shutoff;
      levelTime = currTime;
      break;
    default:
      // We are still in shutoff state after levelDelay seconds
      if (currTime - levelTime >= levelDelay)
      {
        digitalWrite(warnLEDPin, HIGH); // for now, warn LED also goes on when in shutoff
        digitalWrite(pumpShutoffPin, HIGH);
      }
      break;
    }
  }
  else if (highWarnLevel > avg_level && avg_level > lowWarnLevel) // we are in ideal range
  {
    switch (levelState)
    {
    case Normal:
      // continue ideal operation
      break;
    default: // we recovered from a warning or shutoff state
      levelState = Normal;
      digitalWrite(warnLEDPin, LOW);
      digitalWrite(pumpShutoffPin, LOW);
      break;
    }
  }
  else if (avg_level >= highWarnLevel || avg_level <= lowWarnLevel) // we are in either warning zone
  {
    switch (levelState)
    {
    case Normal: // was in normal operation at last cycle
      // set timer, and enter warning mode
      levelState = Warn;
      levelTime = currTime;
      break;
    case Warn:
      // We are still in warning state after levelDelay seconds
      if (currTime - levelTime >= levelDelay)
      {
        digitalWrite(warnLEDPin, HIGH); // turn on warning LED
      }
      break;
    default: // came from shutoff state lowered into warning state
      levelState = Warn;
      // in this case, we are probably still very close to the shutoff region, so the pumps will remain off until we return to the ideal range
      break;
    }
  }
}

void logData()
{
  Serial.print(currTime);
  Serial.print(",");
  Serial.print(ph);
  Serial.print(",");
  Serial.print(lux_1);
  Serial.print(",");
  Serial.print(humPercent);
  Serial.print(",");
  Serial.print(ambTemp);
  Serial.print(",");
  Serial.print(tempC);
  Serial.print(",");
  Serial.print(ppm);
  Serial.print(",");
  Serial.print(waterLevel_1);
  Serial.print(",");
  Serial.print(waterLevel_2);
  Serial.print(",");
  Serial.print(flowRate_1);
  Serial.print(",");
  Serial.print(flowRate_2);
  Serial.print(",");
  Serial.print(flowRate_3);
  Serial.print(",");
  Serial.print(flowRate_4);
  Serial.println();
}

void Flow1()
{
  flowCount_1++;
}
void Flow2()
{
  flowCount_2++;
}
void Flow3()
{
  flowCount_3++;
}
void Flow4()
{
  flowCount_4++;
}