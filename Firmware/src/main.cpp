
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

void tachISR_SIDE();
void tachISR_BACK();
float readRPM_1();
void setFanSpeed(float dutyCycle, uint8_t group);
void updatePID(uint8_t group);
float estimateAirflow(int rpm);


// PID stabilization timer
unsigned long pidStableStart = 0;
bool pidStableReady = false;

// RPM filtering variables
// Use longer filter for slow fans
#define RPM_FILTER_SIZE 10
float rpmFilterBuffer[RPM_FILTER_SIZE] = {0};
int rpmFilterIndex = 0;
float rpmFiltered = 0;

// PID controller variables
float targetRPM_BACK = 0;
float targetRPM_SIDE = 0;
float integral = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;

// PID constants (tune these for your specific fan)
float Kp = 0.03;   // Even lower proportional gain
float Ki = 0.01;   // Lower integral gain
float Kd = 0.005;  // Lower derivative gain


// Group 1 (Side)
#define TACH_PIN_1 2
#define PWM_PIN_1 9 // Use Pin 9 (Timer 1)

// Group 2 (Back)
#define TACH_PIN_2 3
#define PWM_PIN_2 10 // Use Pin 10 (Timer 1)

// Fan groups
#define BACK 2
#define SIDE 1

// RPM measurement variables
volatile unsigned long pulseCountSide = 0;
volatile unsigned long pulseCountBack = 0;  
unsigned long lastPulseTime = 0;
unsigned long lastRpmTime = 0;
float currentRPM = 0;
int currentDutyCycle = 0;

// Lookup tables
const int pwmPoints[] = {0, 26, 51, 77, 128, 179, 254, 255};
const int rpmPoints[] = {0, 205, 350, 560, 980, 1370, 1900, 2300};
const float airflowPoints[] = {0, 4.1, 8.3, 12.4, 16.6, 24.9, 33.2, 38.2};

void setup(void) {
  Serial.begin(115200);
  Serial.println("Fan Controller");


  pinMode(PWM_PIN_1, OUTPUT);
  pinMode(PWM_PIN_2, OUTPUT);
  // Set Timer 1 to 25kHz PWM for 4-pin fans (pin 9/10)
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM11);
  TCCR1B = _BV(WGM13) | _BV(CS10);
  ICR1 = 320; // 25kHz
  
  pinMode(TACH_PIN_1, INPUT_PULLUP);
  pinMode(TACH_PIN_2, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(TACH_PIN_1), tachISR_SIDE, FALLING);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN_2), tachISR_BACK, FALLING);

  lastRpmTime = millis();
  lastPIDTime = millis();

  setFanSpeed(100, SIDE);
  setFanSpeed(100, BACK);
  delay(500);
  setFanSpeed(0, SIDE);
  setFanSpeed(0, BACK);
}

void tachISR_SIDE() {
  pulseCountSide++;
}

void tachISR_BACK() {
  pulseCountBack  ++;
}

float MovingAvgRPM(float newRPM) {
  static const int size = 10;
  static float buffer[size] = {0};
  static int index = 0;
  static float sum = 0;



  sum -= buffer[index];
  buffer[index] = newRPM;
  sum += newRPM;
  
  index = (index + 1) % size;
  
  return sum / size;
}

float readRPM_1() {
  static unsigned long lastCalcTime = 0;
  static unsigned long lastPulseCount = 0;

  unsigned long currentTime = millis();
  unsigned long timeDiff = currentTime - lastCalcTime;

  // Calculate RPM every 1000ms for stability
  if (timeDiff < 1000) {
    return currentRPM; // Return last calculated RPM
  }

  noInterrupts();
  unsigned long pulses = pulseCountSide - lastPulseCount;
  lastPulseCount = pulseCountSide;
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

void setFanSpeed(float dutyCycle, uint8_t group) {
    if (group == 1) {
        analogWrite(PWM_PIN_1, (uint8_t)dutyCycle);
    } else if (group == 2) {
        analogWrite(PWM_PIN_2, (uint8_t)dutyCycle);
    }
    currentDutyCycle = dutyCycle;
}

void updatePID(uint8_t group) {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0; // Convert to seconds
  if (now - lastPIDTime < 2000) return; // Run PID at most every 200ms
  
  float currentRPM = rpmFiltered;
  float error = targetRPM_BACK - currentRPM;
  
  // Only use PID if we're reasonably close to target
  if (targetRPM_BACK > 100 && abs(error) > 50) {
    // Large error - use more aggressive approach
    float step = (error > 0 ? 4 : -4); // Lower step for large error
    float newDuty = currentDutyCycle + step;
   
    newDuty = constrain(newDuty, 0, 255);
    setFanSpeed(newDuty, group);
    integral = 0; // Reset integral
  } else {
    // ignore small errors
    if (abs(error) < 20) error = 0;
    // Normal PID control
    integral += error * dt;
    // Anti-windup: limit integral term
    integral = constrain(integral, -100, 100);
    float derivative = (error - lastError) / dt;
    float output = Kp * error + Ki * integral + Kd * derivative;
    // Limit output change rate
    float maxChange = 1.0; // Max 1% change per cycle
    output = constrain(output, -maxChange, maxChange);
    float newDuty = currentDutyCycle + output;
    newDuty = constrain(newDuty, 0, 255);

    setFanSpeed(newDuty, group);
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
  float currentRPM = readRPM_1();
  

  rpmFiltered = MovingAvgRPM(currentRPM);
  
if (targetRPM_BACK > 0) {
    // Wait for RPM to stabilize for at least 10 seconds before PID control
    if (!pidStableReady) {
      if (pidStableStart == 0) pidStableStart = millis();
      if (millis() - pidStableStart >= 10000) pidStableReady = true;
    }
    if (pidStableReady) {
      updatePID(BACK);
      pidStableReady = false; // Reset until next stabilization period
      pidStableStart = millis(); // Restart stabilization timer
    }
  } else {
    // Reset stabilization timer if PID is not active
    pidStableStart = 0;
    pidStableReady = false;
  }
  
  // Print status every second
  if (now - lastPrintMillis >= 1000) {
  
      Serial.print("RPM: ");
      Serial.print(rpmFiltered, 1);
      Serial.print(" | Duty: ");
      Serial.print(currentDutyCycle, 1);
      Serial.print("% | Target: ");
      Serial.print(targetRPM_BACK, 1);
      Serial.print(" | Error: ");
      Serial.print(targetRPM_BACK - rpmFiltered, 1);
      Serial.print(" | PID: ");
      Serial.print(Kp, 3);
      Serial.print(",");
      Serial.print(Ki, 3);
      Serial.print(",");
      Serial.print(Kd, 3);
      Serial.print(", ");
      
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

      // R#group#rpm (e.g. R1#900 or R2#1200)
      if (line.startsWith("R") && line.indexOf('#') > 0) {
        int group = line.substring(1, line.indexOf('#')).toInt();
        int rpm = line.substring(line.indexOf('#') + 1).toInt();
        int groupId = (group == 1) ? SIDE : BACK;
        float* targetRPM = (groupId == SIDE) ? &targetRPM_SIDE : &targetRPM_BACK;
        *targetRPM = constrain(rpm, 0, 3000);
        integral = 0; lastError = 0;
        int initialDuty = 0;
        for (int i = 0; i < sizeof(rpmPoints)/sizeof(rpmPoints[0]) - 1; i++) {
          if (*targetRPM >= rpmPoints[i] && *targetRPM <= rpmPoints[i+1]) {
            float ratio = (float)(*targetRPM - rpmPoints[i]) / (rpmPoints[i+1] - rpmPoints[i]);
            initialDuty = pwmPoints[i] + ratio * (pwmPoints[i+1] - pwmPoints[i]);
            break;
          }
        }
        currentDutyCycle = initialDuty;
        setFanSpeed(initialDuty, groupId);
        Serial.print("[CMD] Group ");
        Serial.print(group);
        Serial.print(" Target RPM set to: ");
        Serial.println(*targetRPM);
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
        targetRPM_BACK = 0; // Disable PID control
        setFanSpeed(constrain(input, 0, 255), SIDE);
        setFanSpeed(constrain(input, 0, 255), BACK);
      }
    }
  
  delay(50);
}