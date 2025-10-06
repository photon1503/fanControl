/*
Use serial commands to tune:

P0.2 - set Kp to 0.2

I0.05 - set Ki to 0.05

D0.02 - set Kd to 0.02

Increase Kp for faster response (but may cause oscillation)

Increase Ki to eliminate steady-state error

Increase Kd to reduce overshoot
*/
#include <Arduino.h>

void tachISR();
float readRPM();
void setFanSpeed(float dutyCycle);
void updatePID();
float estimateAirflow(int rpm);


// RPM filtering variables
#define RPM_FILTER_SIZE 5  // Reduced for faster response
float rpmFilterBuffer[RPM_FILTER_SIZE] = {0};
int rpmFilterIndex = 0;
float rpmFiltered = 0;

// PID controller variables
float targetRPM = 0;
float integral = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;

// PID constants (tune these for your specific fan)
float Kp = 0.15;   // Increased proportional gain
float Ki = 0.02;   // Integral gain
float Kd = 0.01;   // Derivative gain

// Arduino Nano pins
#define TACH_PIN 2
#define PWM_PIN 9 // Use Pin 9 (Timer 1)

// RPM measurement variables
volatile unsigned long pulseCount = 0;
unsigned long lastPulseTime = 0;
unsigned long lastRpmTime = 0;
float currentRPM = 0;
int currentDutyCycle = 0;

// Lookup tables
const int pwmPoints[] = {0, 26, 51, 77, 128, 179, 255};
const int rpmPoints[] = {0, 225, 450, 900, 1800, 2000, 2200};
const float airflowPoints[] = {0, 5.8, 11.5, 23, 39, 48, 55.5};

void setup(void) {
  Serial.begin(115200);
  Serial.println("Fan Controller - Improved Response");

  pinMode(PWM_PIN, OUTPUT);
  // Set Timer 1 to 25kHz PWM for 4-pin fans (pin 9/10)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 320; // 25kHz
  
  pinMode(TACH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);
  
  lastRpmTime = millis();
  lastPIDTime = millis();
  
  setFanSpeed(100);
  delay(500);
  setFanSpeed(0);
}

void tachISR() {
  pulseCount++;
}

float readRPM() {
  static unsigned long lastCalcTime = 0;
  static unsigned long lastPulseCount = 0;
  
  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - lastCalcTime;
  
  // Calculate RPM more frequently (every 200ms instead of 1000ms)
  if (timeDiff < 200) {
    return currentRPM; // Return last calculated RPM
  }
  
  noInterrupts();
  unsigned long pulses = pulseCount - lastPulseCount;
  pulseCount = 0;
  lastPulseCount = 0;
  interrupts();
  
  if (timeDiff > 0 && pulses > 0) {
    // 2 pulses per revolution for most fans
    currentRPM = (pulses * 60000.0) / (2.0 * timeDiff);
  } else {
    currentRPM = 0;
  }
  
  lastCalcTime = currentTime;
  return currentRPM;
}

void setFanSpeed(float dutyCycle) {
  // Clamp duty cycle to 0-100
  dutyCycle = constrain(dutyCycle, 0, 100);
  currentDutyCycle = dutyCycle;
  
  // Convert 0-100% to 0-255 PWM value
  uint8_t pwmValue = map(dutyCycle, 0, 100, 0, 255);
  analogWrite(PWM_PIN, pwmValue);
}

void updatePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0; // Convert to seconds
  
  if (dt < 0.1) return; // Run PID at most every 100ms
  
  float currentRPM = rpmFiltered;
  float error = targetRPM - currentRPM;
  
  // Only use PID if we're reasonably close to target
  if (targetRPM > 100 && abs(error) > 50) {
    // Large error - use more aggressive approach
    float newDuty = currentDutyCycle + (error > 0 ? 10 : -10);
    newDuty = constrain(newDuty, 0, 255);
    setFanSpeed(newDuty);
    integral = 0; // Reset integral
  } else {
    // Normal PID control
    integral += error * dt;
    
    // Anti-windup: limit integral term
    integral = constrain(integral, -100, 100);
    
    float derivative = (error - lastError) / dt;
    
    float output = Kp * error + Ki * integral + Kd * derivative;
    
    // Limit output change rate
    float maxChange = 5.0; // Max 5% change per cycle
    output = constrain(output, -maxChange, maxChange);
    
    float newDuty = currentDutyCycle + output;
    setFanSpeed(newDuty);
    
    lastError = error;
  }
  
  lastPIDTime = now;
}

float estimateAirflow(int rpm) {
  if (rpm <= rpmPoints[0]) return airflowPoints[0];
  
  for (int i = 0; i < sizeof(rpmPoints) / sizeof(rpmPoints[0]) - 1; i++) {
    if (rpm >= rpmPoints[i] && rpm <= rpmPoints[i + 1]) {
      float ratio = (float)(rpm - rpmPoints[i]) / (rpmPoints[i + 1] - rpmPoints[i]);
      return airflowPoints[i] + ratio * (airflowPoints[i + 1] - airflowPoints[i]);
    }
  }
  return airflowPoints[sizeof(airflowPoints) / sizeof(airflowPoints[0]) - 1];
}

void loop(void) {
  static unsigned long lastPrintMillis = 0;
  unsigned long now = millis();
  
  // Read RPM more frequently
  float currentRPM = readRPM();
  
  // Update moving average filter
  rpmFilterBuffer[rpmFilterIndex] = currentRPM;
  rpmFilterIndex = (rpmFilterIndex + 1) % RPM_FILTER_SIZE;
  
  float rpmSum = 0;
  for (int i = 0; i < RPM_FILTER_SIZE; i++) {
    rpmSum += rpmFilterBuffer[i];
  }
  rpmFiltered = rpmSum / RPM_FILTER_SIZE;
  
  // Update PID control more frequently
  if (targetRPM > 0) {
    updatePID();
  }
  
  // Print status every second
  if (now - lastPrintMillis >= 1000) {
    Serial.print("RPM: ");
    Serial.print(rpmFiltered, 1);
    Serial.print(" | Duty: ");
    Serial.print(currentDutyCycle, 1);
    Serial.print("% | Target: ");
    Serial.print(targetRPM, 1);
    Serial.print(" | Error: ");
    Serial.print(targetRPM - rpmFiltered, 1);
    Serial.println(" rpm");
    
    float airflow = estimateAirflow((int)rpmFiltered);
    Serial.print("Airflow: ");
    Serial.print(airflow);
    Serial.println(" m³/h");
    
    lastPrintMillis = now;
  }

  // Handle serial commands
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    
    if (line.startsWith("R")) {
      int rpm = line.substring(1).toInt();
      targetRPM = constrain(rpm, 0, 3000);
      integral = 0; // Reset integral when target changes
      lastError = 0;
      Serial.print("[CMD] Target RPM set to: ");
      Serial.println(targetRPM);
    } else if (line.startsWith("P")) {
      Kp = line.substring(1).toFloat();
      Serial.print("[CMD] Kp set to: ");
      Serial.println(Kp);
    } else if (line.startsWith("I")) {
      Ki = line.substring(1).toFloat();
      Serial.print("[CMD] Ki set to: ");
      Serial.println(Ki);
    } else if (line.startsWith("D")) {
      Kd = line.substring(1).toFloat();
      Serial.print("[CMD] Kd set to: ");
      Serial.println(Kd);
    } else {
      int input = line.toInt();
      targetRPM = 0; // Disable PID control
      setFanSpeed(constrain(input, 0, 100));
    }
  }
  
  delay(50);
}