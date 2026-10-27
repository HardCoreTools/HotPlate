#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

#define PIN_MOSFET PIN_PC1
#define PIN_VDIV   PIN_PD4
#define PIN_TEMP   PIN_PD7
#define PIN_UP     PIN_PD5
#define PIN_DOWN   PIN_PD6

#define SCREEN_W 128
#define SCREEN_H 32

Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

const float ADC_REF_MV = 4340.0;
const float DIV_RATIO = 4.0;

const int SET_MIN = 30;
const int SET_MAX = 140;
const int SET_STEP = 5;

const float HYST = 2.0;
const float OVERTEMP = 145.0;
const float MIN_VIN = 4.0;
const float MAX_VIN = 15.0;

const uint8_t ADC_SAMPLES = 16;
const uint8_t TEMP_FILTER_SIZE = 5;
const unsigned long BUTTON_INTERVAL = 150;
const unsigned long DISPLAY_INTERVAL = 200;
const unsigned long CONTROL_INTERVAL = 100;

int setpoint = 100;

bool heaterOn = false;
bool fault = false;
bool sensorValid = false;
bool voltageValid = false;
bool displayReady = false;
bool filterReady = false;

float tempHistory[TEMP_FILTER_SIZE];
uint8_t tempIndex = 0;
uint8_t tempCount = 0;
float filteredTemp = NAN;
float vin = NAN;

unsigned long lastButton = 0;
unsigned long lastDisplay = 0;
unsigned long lastControl = 0;

float readMilliVolts(uint8_t pin) {
  analogRead(pin);

  uint32_t sum = 0;

  for (uint8_t i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(pin);
  }

  float adc = sum / (float)ADC_SAMPLES;
  return adc * ADC_REF_MV / 1023.0;
}

float readVin() {
  return readMilliVolts(PIN_VDIV) * DIV_RATIO / 1000.0;
}

float readTemperature() {
  float mv = readMilliVolts(PIN_TEMP);

  if (!isfinite(mv) || mv < 200.0 || mv > 2000.0) {
    return NAN;
  }

  float discriminant =
      8.194 * 8.194 +
      4.0 * 0.00262 * (1324.0 - mv);

  if (discriminant < 0.0 || !isfinite(discriminant)) {
    return NAN;
  }

  float temp =
      (8.194 - sqrt(discriminant)) /
      (2.0 * -0.00262) + 30.0;

  if (!isfinite(temp) || temp < -20.0 || temp > 150.0) {
    return NAN;
  }

  return temp;
}

void forceHeaterOff() {
  heaterOn = false;
  digitalWrite(PIN_MOSFET, LOW);
}

void handleButtons() {
  unsigned long now = millis();

  if (now - lastButton < BUTTON_INTERVAL) {
    return;
  }

  if (digitalRead(PIN_UP) == LOW) {
    setpoint = min(setpoint + SET_STEP, SET_MAX);
    lastButton = now;
  } else if (digitalRead(PIN_DOWN) == LOW) {
    setpoint = max(setpoint - SET_STEP, SET_MIN);
    lastButton = now;
  }
}

void updateTemperatureFilter(float temp) {
  if (!isfinite(temp)) {
    sensorValid = false;
    filterReady = false;
    tempCount = 0;
    tempIndex = 0;
    filteredTemp = NAN;
    return;
  }

  sensorValid = true;

  tempHistory[tempIndex] = temp;
  tempIndex = (tempIndex + 1) % TEMP_FILTER_SIZE;

  if (tempCount < TEMP_FILTER_SIZE) {
    tempCount++;
  }

  float sum = 0.0;

  for (uint8_t i = 0; i < tempCount; i++) {
    sum += tempHistory[i];
  }

  filteredTemp = sum / tempCount;
  filterReady = (tempCount == TEMP_FILTER_SIZE);
}

void drawScreen() {
  if (!displayReady) {
    return;
  }

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  display.setTextSize(2);
  display.setCursor(0, 0);

  if (!sensorValid || !filterReady) {
    display.print("T:ERR");
  } else {
    display.print("T:");
    display.print((int)(filteredTemp + 0.5));
    display.print("C");
  }

  display.setTextSize(1);
  display.setCursor(84, 0);
  display.print("SET ");
  display.print(setpoint);

  display.setCursor(0, 22);
  display.print("Vin ");

  if (isfinite(vin)) {
    display.print(vin, 1);
    display.print("V");
  } else {
    display.print("ERR");
  }

  display.setCursor(80, 22);

  if (fault) {
    display.print("FAULT");
  } else if (!voltageValid) {
    display.print("LOW/HI V");
  } else if (heaterOn) {
    display.print("HEAT ON");
  } else {
    display.print("HEAT OFF");
  }

  display.display();
}

void setup() {
  pinMode(PIN_MOSFET, OUTPUT);
  digitalWrite(PIN_MOSFET, LOW);

  pinMode(PIN_UP, INPUT_PULLUP);
  pinMode(PIN_DOWN, INPUT_PULLUP);

  analogReference(INTERNAL4V34);

  Wire.begin();

  displayReady = display.begin(
      SSD1306_SWITCHCAPVCC, 0x3C
  );

  if (displayReady) {
    display.clearDisplay();
    display.display();
  }

  forceHeaterOff();
}

void loop() {
  unsigned long now = millis();

  handleButtons();

  if (now - lastControl >= CONTROL_INTERVAL) {
    lastControl = now;

    vin = readVin();

    voltageValid = isfinite(vin) &&
                   vin >= MIN_VIN &&
                   vin <= MAX_VIN;

    float rawTemp = readTemperature();
    updateTemperatureFilter(rawTemp);

    if (!sensorValid || !isfinite(rawTemp)) {
      fault = true;
    }

    if (sensorValid && rawTemp >= OVERTEMP) {
      fault = true;
    }

    if (fault || !voltageValid || !filterReady) {
      forceHeaterOff();
    } else if (filteredTemp <= setpoint - HYST) {
      heaterOn = true;
    } else if (filteredTemp >= setpoint + HYST) {
      heaterOn = false;
    }

    digitalWrite(PIN_MOSFET, heaterOn ? HIGH : LOW);
  }

  if (now - lastDisplay >= DISPLAY_INTERVAL) {
    lastDisplay = now;
    drawScreen();
  }
}
