// PID controller variables
float targetRPM = 0;
float pidKp = 0.15;
float pidKi = 0.005;
float pidKd = 0.02;
float pidIntegral = 0;
float pidLastError = 0;
float pidOutput = 0;
#include <Arduino.h>

void setFanSpeed(float dutyCycle);

void tachISR();


// Arduino Nano pins
#define TACH_PIN 2
#define PWM_PIN 9 // Use Pin 9 (Timer 1)

unsigned long lastRpmCalcTime = 0;
unsigned long lastPrintTime = 0;
unsigned long lastPulseTime = 0;
unsigned int currentRPM = 0;
byte currentDutyCycle = 0;
volatile bool sampleWindowOpen = false;
volatile unsigned long syncPulseCount = 0;
volatile unsigned long pulseCount = 0;
volatile unsigned long lastAcceptedPulseAtMs = 0;
unsigned long totalSampleTime = 0;
unsigned long lastSampleResetTime = 0;
bool lastPWMState = LOW;

unsigned long lastTachReadTime = 0;
bool pwmInterrupted = false;
uint32_t savedPwmValue = 0;

const byte pwmPoints[] = {0, 10, 20, 30, 50, 70, 100};
const int rpmPoints[] = {0, 225, 450, 900, 1800, 2000, 2200};
const float airflowPoints[] = {0, 5.8, 11.5, 23, 39, 48, 55.5};

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Fan Controller");


  pinMode(PWM_PIN, OUTPUT);
  // Set Timer 1 to 25kHz PWM for 4-pin fans (pin 9/10)
  // Fast PWM, TOP=0x00FF, prescaler=1
TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 320; // 25kHz
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);
  lastRpmCalcTime = millis();
  lastPrintTime = millis();
  setFanSpeed(100);
  delay(500);
  setFanSpeed(0);
}

void tachISR()
{

  pulseCount++;

}

float readRPM()
{
  // Measure pulses over a 1000ms window
  pulseCount = 0;
  unsigned long start = millis();
  delay(1000);
  unsigned long elapsed = millis() - start;
  unsigned long pulses = pulseCount;
  // Calculate RPM based on elapsed time and pulses
  float rpm = (pulses * 60000.0) / (2.0 * elapsed); // 2 pulses per revolution
  return rpm;
}



void setFanSpeed(float dutyCycle)
{
  // Clamp duty cycle to 0-100
  if (dutyCycle < 0) dutyCycle = 0;
  if (dutyCycle > 100) dutyCycle = 100;
  currentDutyCycle = dutyCycle;
  // Arduino Nano uses 8-bit PWM (0-255)
  byte pwmValue = map((int)dutyCycle, 0, 100, 0, 255);
  analogWrite(PWM_PIN, pwmValue);
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


void loop(void)
{

  static unsigned long lastPrintMillis = 0;
  unsigned long now = millis();
  if (now - lastPrintMillis >= 1000)
  {
    float rpms = readRPM();
    // PID controller to reach target RPM (update only once per interval)
    if (targetRPM > 0) {
      float error = targetRPM - rpms;
      pidIntegral += error;
      float derivative = error - pidLastError;
      pidOutput = pidKp * error + pidKi * pidIntegral + pidKd * derivative;
      pidLastError = error;
  // Limit duty cycle change per update to ±5%
  float maxStep = 5.0;
  float delta = pidOutput;
  if (delta > maxStep) delta = maxStep;
  if (delta < -maxStep) delta = -maxStep;
  float newDuty = constrain(currentDutyCycle + delta, 0, 100);
  setFanSpeed(newDuty);
    }
  Serial.print("RPM: ");
  Serial.print(rpms, 2);
  Serial.print(" | Duty: ");
  Serial.print(currentDutyCycle, 1);
  Serial.print("% | Target: ");
  Serial.print(targetRPM, 1);
  Serial.println(" rpm");
    float airflow = estimateAirflow((int)rpms);
    Serial.print("Airflow: ");
    Serial.print(airflow);
    Serial.println(" m³/h");
    lastPrintMillis = now;
    pulseCount = 0;
  }

  if (Serial.available() > 0)
  {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("R")) {
      int rpm = line.substring(1).toInt();
      targetRPM = constrain(rpm, 0, 3000); // Set target RPM (adjust max as needed)
      Serial.print("[CMD] Target RPM set to: ");
      Serial.println(targetRPM);
    } else {
      int input = line.toInt();
      float target = constrain(input, 0, 100);
      setFanSpeed(target);
    }
  }

  // ...existing code...



  delay(50);
}