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

#include "DHT.h"
#include <BH1750.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// ==========================================
// PIN DEFINITIONS & CONSTANTS
// ==========================================

// Environment Sensors
#define DHTPIN 3
#define DHTTYPE DHT11

// Water Level (Ultrasonic)
#define TRIG_1 9
#define ECHO_1 10
#define TRIG_2 11
#define ECHO_2 12

// Water Quality
#define ONE_WIRE_BUS 2
const int phPin = A0;
const int tdsPin = A3;

// Flow Sensors
const int flowPin_1 = 5;
const int flowPin_2 = 6;
const int flowPin_3 = 7;
const int flowPin_4 = 8;

// System Controls
// FOR JACOB: Determine these values
const int warnLEDPin = 15;
const int pumpShutoffPin = 16;

// ==========================================
// CONFIGURATION VARIABLES
// ==========================================

// Reservoir Levels (cm)
// The distance of the Ultrasonic sensor to bottom of reservoir
// Measure from the sensor face down to the bottom of the reservoir
float tankHeight_1 = 30.0;  // <-- NEW (change this value)
float tankHeight_2 = 30.0;  // <-- NEW (change this value)
float maxLevel = 50.0;      // Max level to shutoff before overflow
float highWarnLevel = 45.0; // High water level warning
float lowWarnLevel = 10.0;  // Low water level warning
float minLevel = 5.0;       // Min level to shutoff before running dry

// Timing & Calibration
const unsigned long levelDelay = 5000; // 5-second delay before triggering state changes
const int recordPeriod = 2;            // Delay between data readings (seconds)
const float flowSensorConstant = 5.5;  // Flow sensor calibration constant
const float phSlope = 4.87012897;
const float phOffset = -2.306816462;
const bool printLabels = true;

// ==========================================
// GLOBAL STATE & SENSOR OBJECTS
// ==========================================

enum LevelState
{
  Normal = 0,
  Warn = 1,
  Shutoff = 2
};
LevelState levelState = Normal;
unsigned long levelTime = 0;
unsigned long currTime = 0;

// Sensor Objects
DHT dht(DHTPIN, DHTTYPE);
BH1750 lightMeter_1;
// BH1750 lightMeter_2;
// BH1750 lightMeter_3;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Sensor Readings
float humPercent, ambTemp;
float lux_1;
// float lux_2;
// float lux_3;
float phVolt, ph;
float tempC;
float voltagetds, ppm;
float waterLevel_1, waterLevel_2, avg_level;

// Flow Rates & Counters
double flowRate_1, flowRate_2, flowRate_3, flowRate_4;
volatile int flowCount_1 = 0;
volatile int flowCount_2 = 0;
volatile int flowCount_3 = 0;
volatile int flowCount_4 = 0;

// Interrupt Service Routines
void Flow1() { flowCount_1++; }
void Flow2() { flowCount_2++; }
void Flow3() { flowCount_3++; }
void Flow4() { flowCount_4++; }

// ==========================================
// SETUP
// ==========================================

void setup()
{
  Serial.begin(9600);
  initHardware();

  if (printLabels)
  {
    Serial.println("Time (ms), PH, Light (lx), Humidity (%), Ambient Temp (°C), Liquid Temp (°C), TDS (ppm), Water Level 1 (cm), Water Level 2 (cm), Flow Rate 1 (L/min), Flow Rate 2 (L/min), Flow Rate 3 (L/min)"); // CSV header
  }
}

// ==========================================
// MAIN LOOP
// ==========================================

void loop()
{
  readEnvironment();
  readWaterQuality();
  readWaterLevel();

  processFlowRatesAndDelay(); // Handles the interrupt counting window

  currTime = millis();
  updateSystemControls();
  logData();
}

// ==========================================
// HELPER FUNCTIONS
// ==========================================

void initHardware()
{
  // Actuators
  pinMode(warnLEDPin, OUTPUT);
  pinMode(pumpShutoffPin, OUTPUT);
  digitalWrite(warnLEDPin, LOW);
  digitalWrite(pumpShutoffPin, LOW); // initialize pump shutoff and LED as off

  // Flow Sensors
  pinMode(flowPin_1, INPUT);
  pinMode(flowPin_2, INPUT);
  pinMode(flowPin_3, INPUT);
  pinMode(flowPin_4, INPUT);
  attachInterrupt(digitalPinToInterrupt(flowPin_1), Flow1, RISING);
  attachInterrupt(digitalPinToInterrupt(flowPin_2), Flow2, RISING);
  attachInterrupt(digitalPinToInterrupt(flowPin_3), Flow3, RISING);
  attachInterrupt(digitalPinToInterrupt(flowPin_4), Flow4, RISING);

  // Ultrasonic Sensors
  pinMode(TRIG_1, OUTPUT); // sound pulse is output
  pinMode(ECHO_1, INPUT);  // Echo is input
  pinMode(TRIG_2, OUTPUT); // sound pulse is output
  pinMode(ECHO_2, INPUT);  // Echo is input

  // Temperature Pullup
  pinMode(ONE_WIRE_BUS, INPUT_PULLUP);

  // Initialize I2C and Sensors
  Wire.begin();
  // Wire2.begin(18,19);
  // Wire3.begin(16,17);

  lightMeter_1.begin();
  lightMeter_1.configure(BH1750::CONTINUOUS_HIGH_RES_MODE);
  // lightMeter_2.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire);
  // lightMeter_3.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &Wire3);

  dht.begin();
  sensors.begin();
}

void readEnvironment()
{
  humPercent = dht.readHumidity();
  ambTemp = dht.readTemperature();

  // Skip further readings if DHT fails (will retry next loop)
  if (isnan(humPercent) || isnan(ambTemp))
    return;

  lux_1 = lightMeter_1.readLightLevel();
  // lux_2 = lightMeter_2.readLightLevel();
  // lux_3 = lightMeter_3.readLightLevel();
}

void readWaterQuality()
{
  // PH
  int sensorValue = analogRead(phPin);
  phVolt = sensorValue * (5.0 / 1023.0); // convert reading to voltage
  ph = phSlope * phVolt + phOffset;

  // Temperature
  sensors.requestTemperatures();
  tempC = sensors.getTempCByIndex(0); // read first sensor

  // TDS
  int tdsOutput = analogRead(tdsPin);
  voltagetds = tdsOutput * (5.0 / 1023.0);
  ppm = (voltagetds * 1000 / 500) * 1.1;
}

void readWaterLevel()
{
  // Trigger ultrasonic pulses
  digitalWrite(TRIG_1, LOW); // when pin is low no sound is emitted
  digitalWrite(TRIG_2, LOW); // when pin is low no sound is emitted
  delayMicroseconds(2);
  digitalWrite(TRIG_1, HIGH); // pulse sent
  digitalWrite(TRIG_2, HIGH); // pulse sent
  delayMicroseconds(10);
  digitalWrite(TRIG_1, LOW); // ends pulse
  digitalWrite(TRIG_2, LOW); // ends pulse

  // Measure return time
  long duration_1 = pulseIn(ECHO_1, HIGH); // measures time echo pin is high
  long duration_2 = pulseIn(ECHO_2, HIGH);

  // Convert to distance (in cm)
  float distance_1 = duration_1 * 0.034 / 2; // 0.034 is speed of sound in cm/us
  float distance_2 = duration_2 * 0.034 / 2;

  // Calculate actual water level
  waterLevel_1 = tankHeight_1 - distance_1;
  waterLevel_2 = tankHeight_2 - distance_2;
  avg_level = (waterLevel_1 + waterLevel_2) / 2.0;
}

void processFlowRatesAndDelay()
{
  // Reset counters
  flowCount_1 = 0;
  flowCount_2 = 0;
  flowCount_3 = 0;
  flowCount_4 = 0;

  // Allow interrupts to count pulses for the record period
  interrupts();
  delay(1000 * recordPeriod);
  noInterrupts();

  // Calculate L/min
  // since F=(5.5*Q), Q = F/5.5, where Q is in L/min and F is in Hz
  // since count is # of pulses in last second, count is F in Hz
  flowRate_1 = flowCount_1 / flowSensorConstant;
  flowRate_2 = flowCount_2 / flowSensorConstant;
  flowRate_3 = flowCount_3 / flowSensorConstant;
  flowRate_4 = flowCount_4 / flowSensorConstant;
}

void updateSystemControls()
{
  if (avg_level >= maxLevel || avg_level <= minLevel)
  {
    // Extreme levels: we are above max level, or below min level
    switch (levelState)
    {
    case Normal:
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
  else if (avg_level < highWarnLevel && avg_level > lowWarnLevel)
  {
    // Ideal range: we are in ideal range
    switch (levelState)
    {
    case Normal:
      // continue ideal operation
      break;
    default:
      // we recovered from a warning or shutoff state
      levelState = Normal;
      digitalWrite(warnLEDPin, LOW);
      digitalWrite(pumpShutoffPin, LOW);
      break;
    }
  }
  else
  {
    // Intermediate levels: we are in either warning zone
    switch (levelState)
    {
    case Normal:
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
    default:
      // came from shutoff state lowered into warning state
      levelState = Warn;
      // in this case, we are probably still very close to the shutoff region,
      // so the pumps will remain off until we return to the ideal range
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