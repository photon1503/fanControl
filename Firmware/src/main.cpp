/*
Use serial commands to tune:

P0.2 - set Kp to 0.2
I0.05 - set Ki to 0.05  
D0.02 - set Kd to 0.02
T - start auto-tune calibration

Increase Kp for faster response (but may cause oscillation)
Increase Ki to eliminate steady-state error
Increase Kd to reduce overshoot

Key Auto-Tune Features:
Ziegler-Nichols Method: Uses the ultimate gain (Ku) and ultimate period (Tu) to calculate PID parameters

Oscillation Detection: Automatically detects when the system starts oscillating around the setpoint

Adaptive Testing: Tries different output levels if oscillations aren't detected initially

Safety Features:

30-second timeout

Minimum oscillation count requirement

Parameter limits to prevent unstable values

Real-time Feedback: Shows progress during the tuning process

Type T in serial monitor to start auto-tune

The system will oscillate the fan to find the ultimate gain and period

After completion, it will display and apply the calculated PID values

You can further fine-tune using the P, I, D commands
*/
#include <Arduino.h>

void tachISR();
float readRPM();
void setFanSpeed(float dutyCycle);
void updatePID();
float estimateAirflow(int rpm);
void startAutoTune();
void updateAutoTune();
void finishAutoTune();


// RPM filtering variables
// Use longer filter for slow fans
#define RPM_FILTER_SIZE 10
float rpmFilterBuffer[RPM_FILTER_SIZE] = {0};
int rpmFilterIndex = 0;
float rpmFiltered = 0;

// PID controller variables
float targetRPM = 0;
float integral = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;

// PID constants (tune these for your specific fan)
float Kp = 0.03;   // Even lower proportional gain
float Ki = 0.01;   // Lower integral gain
float Kd = 0.005;  // Lower derivative gain

// Auto-tune variables
bool autoTuneRunning = false;
unsigned long autoTuneStartTime = 0;
unsigned long lastAutoTuneUpdate = 0;
int autoTuneState = 0;
float autoTuneMaxRPM = 0;
float autoTuneMinRPM = 9999;
float autoTuneLastRPM = 0;
unsigned long autoTuneLastCrossTime = 0;
unsigned long autoTuneLastUpdate = 0;
int autoTuneCrossCount = 0;
float autoTunePeriodSum = 0;
float autoTuneAmplitudeSum = 0;
float autoTuneSetpoint = 0;
float autoTuneOutput = 0;
bool autoTuneOutputHigh = false;

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
const int rpmPoints[] = {0, 205, 350, 560, 980, 1370, 2300};
const float airflowPoints[] = {0, 4.1, 8.3, 12.4, 16.6, 24.9, 33.2, 38.2};

void setup(void) {
  Serial.begin(115200);
  Serial.println("Fan Controller - Improved Response");
  Serial.println("Commands: R<rpm>, P<value>, I<value>, D<value>, T (auto-tune)");

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

  // Calculate RPM every 1000ms for stability
  if (timeDiff < 1000) {
    return currentRPM; // Return last calculated RPM
  }

  noInterrupts();
  unsigned long pulses = pulseCount - lastPulseCount;
  lastPulseCount = pulseCount;
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
 


    analogWrite(PWM_PIN, (uint8_t)dutyCycle);
    currentDutyCycle = dutyCycle;

}

void updatePID() {
  unsigned long now = millis();
  float dt = (now - lastPIDTime) / 1000.0; // Convert to seconds
  if (now - lastPIDTime < 2000) return; // Run PID at most every 200ms
  
  float currentRPM = rpmFiltered;
  float error = targetRPM - currentRPM;
  
  // Only use PID if we're reasonably close to target
  if (targetRPM > 100 && abs(error) > 50) {
    // Large error - use more aggressive approach
    float step = (error > 0 ? 4 : -4); // Lower step for large error
    float newDuty = currentDutyCycle + step;
   
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
    float maxChange = 2.0; // Max 2% change per cycle
    output = constrain(output, -maxChange, maxChange);
    float newDuty = currentDutyCycle + output;
    newDuty = constrain(newDuty, 0, 255);
    
    setFanSpeed(newDuty);
    lastError = error;
  }
  
  lastPIDTime = now;
}

void startAutoTune() {
  if (autoTuneRunning) return;
  
  Serial.println("[AUTO-TUNE] Starting calibration...");
  Serial.println("[AUTO-TUNE] Make sure fan can spin freely!");
  
  autoTuneRunning = true;
  autoTuneStartTime = millis();
  autoTuneLastUpdate = millis();
  autoTuneState = 0;
  autoTuneMaxRPM = 0;
  autoTuneMinRPM = 9999;
  autoTuneLastRPM = 0;
  autoTuneLastCrossTime = 0;
  autoTuneCrossCount = 0;
  autoTunePeriodSum = 0;
  autoTuneAmplitudeSum = 0;
  autoTuneSetpoint = 900; // Target RPM for tuning
  autoTuneOutput = 128;     // Start with 50% duty cycle
  autoTuneOutputHigh = true;
  
  // Set initial output
  setFanSpeed(autoTuneOutput);
  targetRPM = 0; // Disable normal PID during auto-tune
}

void updateAutoTune() {
  if (!autoTuneRunning) return;
  
  unsigned long now = millis();
  float currentRPM = rpmFiltered;
  
  // Update min/max RPM tracking
  if (currentRPM > autoTuneMaxRPM) autoTuneMaxRPM = currentRPM;
  if (currentRPM < autoTuneMinRPM && currentRPM > 100) autoTuneMinRPM = currentRPM;
  
  // Detect zero crossings for oscillation analysis (use filtered RPM)
  if (autoTuneLastCrossTime > 0 && 
      ((autoTuneLastRPM < autoTuneSetpoint && currentRPM >= autoTuneSetpoint) ||
       (autoTuneLastRPM > autoTuneSetpoint && currentRPM <= autoTuneSetpoint))) {
    unsigned long period = now - autoTuneLastCrossTime;
    autoTuneLastCrossTime = now;
    autoTuneCrossCount++;
    if (autoTuneCrossCount >= 2) {
      float amplitude = (autoTuneMaxRPM - autoTuneMinRPM) / 2.0;
      autoTunePeriodSum += period;
      autoTuneAmplitudeSum += amplitude;
      // Reset min/max for next cycle
      autoTuneMaxRPM = currentRPM;
      autoTuneMinRPM = currentRPM;
    }
  } else if (autoTuneLastCrossTime == 0) {
    autoTuneLastCrossTime = now;
  }
  
  autoTuneLastRPM = currentRPM;
  
  // State machine for auto-tune process
  switch (autoTuneState) {
    case 0: // Initial ramp-up and oscillation detection
      if (now - autoTuneStartTime > 5000) { // Wait 5 seconds for oscillations to develop
        if (autoTuneCrossCount >= 4) {
          autoTuneState = 1;
          Serial.println("[AUTO-TUNE] Oscillations detected, analyzing...");
        } else {
          // No oscillations detected, try different output
          autoTuneOutput = autoTuneOutputHigh ? 179 : 77;
          autoTuneOutputHigh = !autoTuneOutputHigh;
          setFanSpeed(autoTuneOutput);
          autoTuneStartTime = now;
          autoTuneCrossCount = 0;
          Serial.println("[AUTO-TUNE] Adjusting output to induce oscillations...");
        }
      }
      break;
      
    case 1: // Analysis phase - collect data for several cycles
      if (autoTuneCrossCount >= 8) { // Collect data from several cycles
        finishAutoTune();
      }
      break;
  }
  
  // Safety timeout
  if (now - autoTuneStartTime > 30000) { // 30 second timeout
    Serial.println("[AUTO-TUNE] Timeout - could not find stable oscillations");
    autoTuneRunning = false;
    setFanSpeed(0);
  }
  
  // Print progress every 2 seconds
  if (now - lastAutoTuneUpdate > 2000) {
    Serial.print("[AUTO-TUNE] Progress: ");
    Serial.print(autoTuneCrossCount);
    Serial.print(" crosses, RPM: ");
    Serial.print(currentRPM);
    Serial.print(", Output: ");
    Serial.print(autoTuneOutput);
    Serial.print("%, Range: ");
    Serial.print(autoTuneMinRPM);
    Serial.print("-");
    Serial.println(autoTuneMaxRPM);
    lastAutoTuneUpdate = now;
  }
}

void finishAutoTune() {
  if (autoTuneCrossCount < 4) {
    Serial.println("[AUTO-TUNE] Insufficient data for tuning");
    autoTuneRunning = false;
    return;
  }
  
  // Calculate average period and amplitude
  float avgPeriod = autoTunePeriodSum / (autoTuneCrossCount - 1);
  float avgAmplitude = autoTuneAmplitudeSum / (autoTuneCrossCount - 1);
  
  // Convert period from ms to seconds
  float Tu = avgPeriod / 1000.0; // Ultimate period in seconds
  
  // Calculate ultimate gain (Ku) - Ziegler-Nichols method
  // Ku = 4 * d / (π * a) where d is output amplitude, a is process amplitude
  float outputAmplitude = 20.0; // We're switching between ±20% from center
  float Ku = (4.0 * outputAmplitude) / (3.14159 * avgAmplitude);
  
  // Ziegler-Nichols tuning rules for PID
  Kp = 0.6 * Ku;
  Ki = 1.2 * Ku / Tu;
  Kd = 0.075 * Ku * Tu;
  
  // Apply safety limits
  Kp = constrain(Kp, 0.1, 2.0);
  Ki = constrain(Ki, 0.01, 1.0);
  Kd = constrain(Kd, 0.001, 0.1);
  
  Serial.println("[AUTO-TUNE] Complete!");
  Serial.print("[AUTO-TUNE] Ku: ");
  Serial.print(Ku, 4);
  Serial.print(", Tu: ");
  Serial.print(Tu, 4);
  Serial.println("s");
  Serial.print("[AUTO-TUNE] New PID values - Kp: ");
  Serial.print(Kp, 4);
  Serial.print(", Ki: ");
  Serial.print(Ki, 4);
  Serial.print(", Kd: ");
  Serial.println(Kd, 4);
  Serial.println("[AUTO-TUNE] Use these values or fine-tune manually");
  
  autoTuneRunning = false;
  setFanSpeed(0);
  
  // Reset PID state
  integral = 0;
  lastError = 0;
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
  
  // Handle auto-tune or normal PID
  if (autoTuneRunning) {
    updateAutoTune();
  } else if (targetRPM > 0) {
    updatePID();
  }
  
  // Print status every second
  if (now - lastPrintMillis >= 1000) {
    if (autoTuneRunning) {
      Serial.print("[AUTO-TUNE] Running... Crosses: ");
      Serial.print(autoTuneCrossCount);
      Serial.print(", RPM: ");
      Serial.print(rpmFiltered, 1);
      Serial.print(", Output: ");
      Serial.print(autoTuneOutput);
      Serial.println("%");
    } else {
      Serial.print("RPM: ");
      Serial.print(rpmFiltered, 1);
      Serial.print(" | Duty: ");
      Serial.print(currentDutyCycle, 1);
      Serial.print("% | Target: ");
      Serial.print(targetRPM, 1);
      Serial.print(" | Error: ");
      Serial.print(targetRPM - rpmFiltered, 1);
      Serial.print(" | PID: ");
      Serial.print(Kp, 3);
      Serial.print(",");
      Serial.print(Ki, 3);
      Serial.print(",");
      Serial.print(Kd, 3);
      Serial.println(" rpm");
      
      float airflow = estimateAirflow((int)rpmFiltered);
      Serial.print("Airflow: ");
      Serial.print(airflow);
      Serial.println(" m³/h");
    }
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
      // Lookup initial duty cycle from table
      int initialDuty = 0;
      for (int i = 0; i < sizeof(rpmPoints)/sizeof(rpmPoints[0]) - 1; i++) {
        if (targetRPM >= rpmPoints[i] && targetRPM <= rpmPoints[i+1]) {
          float ratio = (float)(targetRPM - rpmPoints[i]) / (rpmPoints[i+1] - rpmPoints[i]);
          initialDuty = pwmPoints[i] + ratio * (pwmPoints[i+1] - pwmPoints[i]);
          break;
        }
      }
      currentDutyCycle = initialDuty;
      setFanSpeed(initialDuty);
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
    } else if (line.startsWith("T")) {
      startAutoTune();
    } else {
      int input = line.toInt();
      targetRPM = 0; // Disable PID control
      
      setFanSpeed(constrain(input, 0, 255));
    }
  }
  
  delay(50);
}