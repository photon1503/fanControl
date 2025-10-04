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
  unsigned long pulses = pulseCount;
  return (pulses * 60.0) / 2.0; // 4 pulses per revolution
}



void setFanSpeed(float dutyCycle)
{
  currentDutyCycle = dutyCycle;
  
  // Arduino Nano uses 8-bit PWM (0-255)
  byte pwmValue = map(dutyCycle, 0, 100, 0, 255); 
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
  Serial.print("RPM: ");
  Serial.print(rpms);
  Serial.print(" | Duty: ");
  Serial.print(currentDutyCycle);
  Serial.println("%");
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
    int input = line.toInt();
    float target = constrain(input, 0, 100);
    setFanSpeed(target);
  }



  delay(50);
}