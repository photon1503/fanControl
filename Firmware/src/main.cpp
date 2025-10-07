/*
 * Fan Controller Firmware for Arduino Nano (ATmega328)
 * ----------------------------------------------------
 * Features:
 *   - Supports two 4-pin PWM fan groups (SIDE and BACK)
 *   - Closed-loop RPM control with PID algorithm
 *   - Serial command interface for tuning and control
 *   - Moving average RPM filtering for stability
 *   - Lookup tables for PWM/RPM/airflow mapping
 *   - Independent duty cycle and RPM tracking per group
 *   - Temperature-based back fan control:
 *       - If mirror temp is hotter than outside temp by >3°C, back fans run at high speed
 *       - If mirror temp is hotter by 0.5–3°C, back fans run at speed linearly scaled from 0 to max
 *       - If mirror temp is cooler or equal, back fans are off
 *
 * Serial Commands:
 *   R1#900   - Set target RPM for SIDE fan to 900
 *   R2#1200  - Set target RPM for BACK fan to 1200
 *   S1#100   - Set duty cycle for SIDE fan to 100 (disables PID)
 *   S2#150   - Set duty cycle for BACK fan to 150 (disables PID)
 *   P0.2     - Set Kp to 0.2
 *   I0.05    - Set Ki to 0.05
 *   D0.02    - Set Kd to 0.02
 *
 * PID Tuning Tips:
 *   - Increase Kp for faster response (may cause oscillation)
 *   - Increase Ki to eliminate steady-state error
 *   - Increase Kd to reduce overshoot
 *
 * Hardware:
 *   - Arduino Nano (ATmega328)
 *   - BME280 sensors for temperature (I2C addresses 0x76 and 0x77)
 *   - 4-pin PWM fans (SIDE: Pin 9, BACK: Pin 10. Optimized for Noctua NF-A8 PWM)
 *   - Tachometer inputs (SIDE: Pin 2, BACK: Pin 3)
 *
 * Author: Gerald Hitz
 * Date: 2025-10-07
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

Adafruit_BME280 bmeOutdoor; // I2C
Adafruit_BME280 bmeMirror;

void tachISR_SIDE();
void tachISR_BACK();
float readRPM(uint8_t group);
void setFanSpeed(float dutyCycle, uint8_t group);
void updatePID(uint8_t group);
float estimateAirflow(int rpm);

// PID stabilization timer
unsigned long pidStableStart = 0;
bool pidStableReady = false;

bool tempCompensationEnabled = false;

// RPM filtering variables
// Use longer filter for slow fans
#define RPM_FILTER_SIZE 10
float rpmFilterBuffer[RPM_FILTER_SIZE] = {0};
int rpmFilterIndex = 0;
float rpmFilteredBack = 0;
float rpmFilteredSide = 0;

// PID controller variables
float targetRPM_BACK = 0;
float targetRPM_SIDE = 0;
float integral = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;

// PID constants (tune these for your specific fan)
float Kp = 0.03;  // Even lower proportional gain
float Ki = 0.01;  // Lower integral gain
float Kd = 0.005; // Lower derivative gain

// Group 1 (Side)
#define TACH_PIN_SIDE 2
#define PWM_PIN_SIDE 9 // Use Pin 9 (Timer 1)

// Group 2 (Back)
#define TACH_PIN_BACK 3
#define PWM_PIN_BACK 10 // Use Pin 10 (Timer 1)

// Fan groups
#define BACK 2
#define SIDE 1

// RPM measurement variables
volatile unsigned long pulseCountSide = 0;
volatile unsigned long pulseCountBack = 0;
unsigned long lastPulseTime = 0;
unsigned long lastRpmTime = 0;
float currentRPM = 0;
int currentDutySide = 0;
int currentDutyBack = 0;

// Lookup tables
const int pwmPoints[] = {0, 26, 51, 77, 128, 179, 254, 255};
const int rpmPoints[] = {0, 205, 350, 560, 980, 1370, 1900, 2300};
const float airflowPoints[] = {0, 4.1, 8.3, 12.4, 16.6, 24.9, 33.2, 38.2};

void setup(void)
{
  Serial.begin(115200);
  while (!Serial)
    ;
  Serial.println("Fan Controller");

  // Set Timer 1 to 25kHz PWM for 4-pin fans (pin 9/10)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 320; // 25kHz

  pinMode(PWM_PIN_SIDE, OUTPUT);
  pinMode(PWM_PIN_BACK, OUTPUT);

  pinMode(TACH_PIN_SIDE, INPUT_PULLUP);
  pinMode(TACH_PIN_BACK, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(TACH_PIN_SIDE), tachISR_SIDE, FALLING);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN_BACK), tachISR_BACK, FALLING);

  lastRpmTime = millis();
  lastPIDTime = millis();

  setFanSpeed(100, SIDE);
  setFanSpeed(100, BACK);
  delay(500);
  setFanSpeed(0, SIDE);
  setFanSpeed(0, BACK);

  delay(1000);
  if (!bmeOutdoor.begin(0x76))
  {
    Serial.println("Could not find a valid BME280 Outdoor sensor, check wiring!");
  }
  else
  {
    Serial.println("BME280 Outoor sensor found!");
  }

  if (!bmeMirror.begin(0x77))
  {
    Serial.println("Could not find a valid BME280 Mirror sensor, check wiring!");
  }
  else
  {
    Serial.println("BME280 Mirror sensor found!");
  }
}

void tachISR_SIDE()
{
  pulseCountSide++;
}

void tachISR_BACK()
{
  pulseCountBack++;
}

float MovingAvgRPM(float newRPM, uint8_t group)
{
  static const int size = 10;
  static float bufferSide[size] = {0};
  static int indexSide = 0;
  static float sumSide = 0;
  static float bufferBack[size] = {0};
  static int indexBack = 0;
  static float sumBack = 0;

  if (group == SIDE)
  {
    sumSide -= bufferSide[indexSide];
    bufferSide[indexSide] = newRPM;
    sumSide += newRPM;
    indexSide = (indexSide + 1) % size;
    return sumSide / size;
  }
  else if (group == BACK)
  {
    sumBack -= bufferBack[indexBack];
    bufferBack[indexBack] = newRPM;
    sumBack += newRPM;
    indexBack = (indexBack + 1) % size;
    return sumBack / size;
  }
  return newRPM;
}

float readRPM(uint8_t group)
{
  static unsigned long lastCalcTimeSide = 0;
  static unsigned long lastPulseCountSide = 0;
  static float lastRPM_Side = 0;
  static unsigned long lastCalcTimeBack = 0;
  static unsigned long lastPulseCountBack = 0;
  static float lastRPM_Back = 0;

  unsigned long currentTime = millis();
  unsigned long timeDiff;
  unsigned long pulses;
  float rpm = 0;

  if (group == SIDE)
  {
    timeDiff = currentTime - lastCalcTimeSide;
    noInterrupts();
    pulses = pulseCountSide - lastPulseCountSide;
    interrupts();
    if (timeDiff >= 1000 && pulses > 0)
    {
      rpm = (pulses * 60000.0) / (2.0 * timeDiff);
      lastRPM_Side = rpm;
      lastPulseCountSide = pulseCountSide;
      lastCalcTimeSide = currentTime;
      return rpm;
    }
    else if (timeDiff >= 1000)
    {
      lastRPM_Side = 0;
      lastPulseCountSide = pulseCountSide;
      lastCalcTimeSide = currentTime;
      return 0;
    }
    return lastRPM_Side;
  }
  else if (group == BACK)
  {
    timeDiff = currentTime - lastCalcTimeBack;
    noInterrupts();
    pulses = pulseCountBack - lastPulseCountBack;
    interrupts();
    if (timeDiff >= 1000 && pulses > 0)
    {
      rpm = (pulses * 60000.0) / (2.0 * timeDiff);
      lastRPM_Back = rpm;
      lastPulseCountBack = pulseCountBack;
      lastCalcTimeBack = currentTime;
      return rpm;
    }
    else if (timeDiff >= 1000)
    {
      lastRPM_Back = 0;
      lastPulseCountBack = pulseCountBack;
      lastCalcTimeBack = currentTime;
      return 0;
    }
    return lastRPM_Back;
  }
  return 0;
}

void setFanSpeed(float dutyCycle, uint8_t group)
{
  if (group == SIDE)
  {
    analogWrite(PWM_PIN_SIDE, (uint8_t)dutyCycle);
    currentDutySide = dutyCycle;
  }
  else if (group == BACK)
  {
    analogWrite(PWM_PIN_BACK, (uint8_t)dutyCycle);
    currentDutyBack = dutyCycle;
  }
}

void updatePID(uint8_t group)
{
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0; // Convert to seconds
  if (now - lastPIDTime < 2000)
    return; // Run PID at most every 200ms

  float currentRPM = (group == SIDE) ? rpmFilteredSide : rpmFilteredBack;
  float targetRPM = (group == SIDE) ? targetRPM_SIDE : targetRPM_BACK;
  float error = targetRPM - currentRPM;

  // Only use PID if we're reasonably close to target
  int currentDuty = (group == SIDE) ? currentDutySide : currentDutyBack;
  if (targetRPM > 100 && abs(error) > 50)
  {
    // Large error - use more aggressive approach
    float step = (error > 0 ? 4 : -4); // Lower step for large error
    float newDuty = currentDuty + step;

    newDuty = constrain(newDuty, 0, 255);
    setFanSpeed(newDuty, group);
    integral = 0; // Reset integral
  }
  else
  {
    // ignore small errors
    if (abs(error) < 20)
      error = 0;
    // Normal PID control
    integral += error * dt;
    // Anti-windup: limit integral term
    integral = constrain(integral, -100, 100);
    float derivative = (error - lastError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    // Limit output change rate
    float maxChange = 1.0; // Max 1% change per cycle
    output = constrain(output, -maxChange, maxChange);
    float newDuty = currentDuty + output;
    newDuty = constrain(newDuty, 0, 255);

    setFanSpeed(newDuty, group);
    lastError = error;
  }

  lastPIDTime = now;
}

float estimateAirflow(int rpm)
{
  if (rpm <= rpmPoints[0])
    return airflowPoints[0];

  for (int i = 0; i < sizeof(rpmPoints) / sizeof(rpmPoints[0]) - 1; i++)
  {
    if (rpm >= rpmPoints[i] && rpm <= rpmPoints[i + 1])
    {
      float ratio = (float)(rpm - rpmPoints[i]) / (rpmPoints[i + 1] - rpmPoints[i]);
      return airflowPoints[i] + ratio * (airflowPoints[i + 1] - airflowPoints[i]);
    }
  }
  return airflowPoints[sizeof(airflowPoints) / sizeof(airflowPoints[0]) - 1];
}

void checkPID(uint8_t group)
{
  // Read RPM more frequently
  if (group == BACK)
  {
    float currentRPM = readRPM(group);
    rpmFilteredBack = MovingAvgRPM(currentRPM, BACK);
  }

  if (group == SIDE)
  {
    float currentRPM = readRPM(group);
    rpmFilteredSide = MovingAvgRPM(currentRPM, SIDE);
  }

  if (group == BACK && targetRPM_BACK > 0)
  {

    if (targetRPM_BACK > 0)
    {
      // Wait for RPM to stabilize for at least 10 seconds before PID control
      if (!pidStableReady)
      {
        if (pidStableStart == 0)
          pidStableStart = millis();
        if (millis() - pidStableStart >= 10000)
          pidStableReady = true;
      }
      if (pidStableReady)
      {
        updatePID(BACK);
        pidStableReady = false;    // Reset until next stabilization period
        pidStableStart = millis(); // Restart stabilization timer
      }
    }
    else
    {
      // Reset stabilization timer if PID is not active
      pidStableStart = 0;
      pidStableReady = false;
    }
  }
  else if (group == SIDE && targetRPM_SIDE > 0)
  {

    static unsigned long pidStableStartSide = 0;
    static bool pidStableReadySide = false;

    if (targetRPM_SIDE > 0)
    {
      // Wait for RPM to stabilize for at least 10 seconds before PID control
      if (!pidStableReadySide)
      {
        if (pidStableStartSide == 0)
          pidStableStartSide = millis();
        if (millis() - pidStableStartSide >= 10000)
          pidStableReadySide = true;
      }
      if (pidStableReadySide)
      {
        updatePID(SIDE);
        pidStableReadySide = false;    // Reset until next stabilization period
        pidStableStartSide = millis(); // Restart stabilization timer
      }
    }
    else
    {
      // Reset stabilization timer if PID is not active
      pidStableStartSide = 0;
      pidStableReadySide = false;
    }
  }
}

void PrintStatus()
{
  // Print status every second

  Serial.print("SIDE: ");
  Serial.print("RPM: ");
  Serial.print(rpmFilteredSide, 1);
  Serial.print(" | Duty: ");
  Serial.print(currentDutySide, 1);
  Serial.print(" | Target: ");
  Serial.print(targetRPM_SIDE, 1);
  Serial.print(" | Error: ");
  Serial.print(targetRPM_SIDE - rpmFilteredSide, 1);
  Serial.print(" | PID: ");
  Serial.print(Kp, 3);
  Serial.print(",");
  Serial.print(Ki, 3);
  Serial.print(",");
  Serial.print(Kd, 3);
  Serial.print(" | Airflow: ");
  Serial.print(estimateAirflow((int)rpmFilteredSide));
  Serial.println(" m³/h");

  Serial.print("BACK: ");
  Serial.print("RPM: ");
  Serial.print(rpmFilteredBack, 1);
  Serial.print(" | Duty: ");
  Serial.print(currentDutyBack, 1);
  Serial.print(" | Target: ");
  Serial.print(targetRPM_BACK, 1);
  Serial.print(" | Error: ");
  Serial.print(targetRPM_BACK - rpmFilteredBack, 1);
  Serial.print(" | PID: ");
  Serial.print(Kp, 3);
  Serial.print(",");
  Serial.print(Ki, 3);
  Serial.print(",");
  Serial.print(Kd, 3);
  Serial.print(" | Airflow: ");
  Serial.print(estimateAirflow((int)rpmFilteredBack));
  Serial.println(" m³/h");

  Serial.print("OUT: T = ");
  Serial.print(bmeOutdoor.readTemperature());
  Serial.print(" °C, H = ");
  Serial.print(bmeOutdoor.readHumidity());
  Serial.print(" %, P = ");
  Serial.print(bmeOutdoor.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("MIRROR: T = ");
  Serial.print(bmeMirror.readTemperature());
  Serial.print(" °C, H = ");
  Serial.print(bmeMirror.readHumidity());
  Serial.print(" %, P = ");
  Serial.print(bmeMirror.readPressure() / 100.0F);
  Serial.println(" hPa");
}

// Add temperature-based fan control logic
float getBackFanDutyFromTemps(float mirrorTemp, float outsideTemp)
{
  float delta = mirrorTemp - outsideTemp;
  if (delta > 3.0)
  {
    // If mirror is much hotter, run back fans at full speed
    return 255.0;
  }
  else if (delta > 0.5)
  {
    // If mirror is slightly hotter, run back fans linearly from 0 to 255
    float duty = (delta / 3.0) * 254.0; // Scale to 0-254
    // constrain to 40-254
    duty = constrain(duty, 60.0, 254.0);
    return duty;
  }
  else
  {
    // If mirror is cooler or equal, turn off back fans
    return 0.0;
  }
}

void loop(void)
{
  static unsigned long lastPrintMillis = 0;
  unsigned long now = millis();
  if (now - lastPrintMillis >= 1000)
  {
    PrintStatus();

    if (tempCompensationEnabled)
    {
      float outdoorTemp = bmeOutdoor.readTemperature();
      float mirrorTemp = bmeMirror.readTemperature();

      int backDuty = (int)getBackFanDutyFromTemps(mirrorTemp, outdoorTemp);
      setFanSpeed(backDuty, BACK);
      targetRPM_BACK = 0; // Disable PID control when temp compensation is active
      Serial.print("[TEMP] Mirror: ");
      Serial.print(mirrorTemp);
      Serial.print(" °C, Outdoor: ");
      Serial.print(outdoorTemp);
      Serial.print(" °C, Back Fan Duty: ");
      Serial.println(backDuty);

    }

    lastPrintMillis = now;
  }

  checkPID(BACK);
  checkPID(SIDE);

  // Handle serial commands
  if (Serial.available() > 0)
  {
    String line = Serial.readStringUntil('\n');
    line.trim();

    // R#group#rpm (e.g. R1#900 or R2#1200)
    if (line.startsWith("R") && line.indexOf('#') > 0)
    {
      int group = line.substring(1, line.indexOf('#')).toInt();
      int rpm = line.substring(line.indexOf('#') + 1).toInt();
      int groupId = (group == 1) ? SIDE : BACK;
      float *targetRPM = (groupId == SIDE) ? &targetRPM_SIDE : &targetRPM_BACK;
      *targetRPM = constrain(rpm, 0, 3000);
      integral = 0;
      lastError = 0;
      int initialDuty = 0;
      for (int i = 0; i < sizeof(rpmPoints) / sizeof(rpmPoints[0]) - 1; i++)
      {
        if (*targetRPM >= rpmPoints[i] && *targetRPM <= rpmPoints[i + 1])
        {
          float ratio = (float)(*targetRPM - rpmPoints[i]) / (rpmPoints[i + 1] - rpmPoints[i]);
          initialDuty = pwmPoints[i] + ratio * (pwmPoints[i + 1] - pwmPoints[i]);
          break;
        }
      }
      if (groupId == SIDE)
        currentDutySide = initialDuty;
      else
        currentDutyBack = initialDuty;
      setFanSpeed(initialDuty, groupId);
      Serial.print("[CMD] Group ");
      Serial.print(group);
      Serial.print(" Target RPM set to: ");
      Serial.println(*targetRPM);
    }
    else if (line.startsWith("P"))
    {
      Kp = line.substring(1).toFloat();
      Serial.print("[CMD] Kp set to: ");
      Serial.println(Kp);
    }
    else if (line.startsWith("I"))
    {
      Ki = line.substring(1).toFloat();
      Serial.print("[CMD] Ki set to: ");
      Serial.println(Ki);
    }
    else if (line.startsWith("D"))
    {
      Kd = line.substring(1).toFloat();
      Serial.print("[CMD] Kd set to: ");
      Serial.println(Kd);
    }
    if (line.startsWith("RESET"))
    {
      targetRPM_SIDE = 0;
      targetRPM_BACK = 0;
      setFanSpeed(0, SIDE);
      setFanSpeed(0, BACK);
      integral = 0;
      lastError = 0;
      Serial.println("[CMD] System reset: Targets cleared and fans stopped.");
    }
    if (line.startsWith("REBOOT"))
    {
      asm volatile("  jmp 0"); // Jump to address 0 to reboot
    }
    if (line.startsWith("T#"))

    {
      // read value after #
      String valueStr = line.substring(2);
      valueStr.trim();
      if (valueStr == "ON")
      {
        tempCompensationEnabled = true;
        Serial.println("[CMD] Temperature compensation enabled.");
      }
      else if (valueStr == "OFF")
      {
        tempCompensationEnabled = false;
        Serial.println("[CMD] Temperature compensation disabled.");
      }
    }
    if (line.startsWith("S") && line.indexOf('#') > 0)
    {
      int group = line.substring(1, line.indexOf('#')).toInt();
      int duty = line.substring(line.indexOf('#') + 1).toInt();
      int groupId = (group == 1) ? SIDE : BACK;

      if (groupId == SIDE)
      {
        targetRPM_SIDE = 0; // Disable PID control
        setFanSpeed(constrain(duty, 0, 255), SIDE);
      }
      else if (groupId == BACK)
      {
        targetRPM_BACK = 0; // Disable PID control
        setFanSpeed(constrain(duty, 0, 255), BACK);
      }
    }
  }

  // Example usage in loop (replace with your actual temp readings):
  // float mirrorTemp = ...;
  // float outsideTemp = ...;
  // int backDuty = (int)getBackFanDutyFromTemps(mirrorTemp, outsideTemp);
  // setFanSpeed(backDuty, BACK);

  delay(50);
}