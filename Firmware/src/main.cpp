// RPM filtering variables
#define RPM_FILTER_SIZE 10
float rpmFilterBuffer[RPM_FILTER_SIZE] = {0};
int rpmFilterIndex = 0;
float rpmFiltered = 0;
// Auto-tune state variables
bool pidCalibrating = false;
unsigned long calStartTime = 0;
float calTargetRPM = 0;
float calHighDuty = 60;
float calLowDuty = 40;
float calLastCrossTime = 0;
float calLastRPM = 0;
int calCrossCount = 0;
float calPeriodSum = 0;
float calAmpSum = 0;
float calMaxRPM = 0;
float calMinRPM = 99999;
// PID controller variables
float targetRPM = 0;

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
  if (dutyCycle > 255) dutyCycle = 100;
  currentDutyCycle = dutyCycle;
  // Arduino Nano uses 8-bit PWM (0-255)
  byte pwmValue = dutyCycle;
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
    // Moving average RPM filter
    rpmFilterBuffer[rpmFilterIndex] = rpms;
    rpmFilterIndex = (rpmFilterIndex + 1) % RPM_FILTER_SIZE;
    float rpmSum = 0;
    for (int i = 0; i < RPM_FILTER_SIZE; i++) rpmSum += rpmFilterBuffer[i];
    rpmFiltered = rpmSum / RPM_FILTER_SIZE;
    // Simple duty cycle adjustment to reach target RPM
        if (targetRPM > 0) {
          float error = targetRPM - rpmFiltered;
          float deadband = 20.0; // No adjustment if error is within ±5 RPM
          float Kp = 0.03;      // Proportional gain (tune as needed)
          float maxStep = 4.0;  // Maximum duty cycle change per update
          float delta = 0;
          if (abs(error) > deadband) {
            delta = Kp * error;
            // Limit change per update
            if (delta > maxStep) delta = maxStep;
            if (delta < -maxStep) delta = -maxStep;
          }
          float newDuty = currentDutyCycle + delta;
          newDuty = constrain(newDuty, 0, 255);
          setFanSpeed(newDuty);
        }
    Serial.print("RPM: ");
    Serial.print(rpmFiltered, 2);
    Serial.print(" | Duty: ");
    Serial.print(currentDutyCycle, 1);
    Serial.print("% | Target: ");
    Serial.print(targetRPM, 1);
    Serial.println(" rpm");
    float airflow = estimateAirflow((int)rpmFiltered);
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
      targetRPM = constrain(rpm, 0, 3000);
      Serial.print("[CMD] Target RPM set to: ");
      Serial.println(targetRPM);
    }  else {
      int input = line.toInt();
      float target = constrain(input, 0, 255);
      setFanSpeed(target);
    }
  }
 
  





  delay(50);
}