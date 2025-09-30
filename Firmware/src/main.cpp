#include <Arduino.h>

void setFanSpeed(byte dutyCycle);
void setPwmFrequency(int pin, int divisor);
void tachISR();

#define TACH_PIN 4
#define PWM_PIN 16 // Change to Pin 9 or 10 (Timer 1)

#define PWM_RESOLUTION  16      // 16-bit = 65536 steps
#define PWM_CHANNEL     0       // Use channel 0


unsigned int pwm_freq = 25;
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



const byte pwmPoints[] = {0,10, 20, 30, 50, 70, 100};
const int rpmPoints[]  = {0,225, 450, 900, 1800, 2000, 2200};
const float airflowPoints[] = {0,5.8, 11.5, 23, 39, 48, 55.5};

void setup(void) {
  Serial.begin(115200);
  Serial.println("Fan Controller - Correct Timing");
  
  // Setup PWM with high resolution
  ledcSetup(PWM_CHANNEL, pwm_freq, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  pinMode(TACH_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);
  lastRpmCalcTime = millis();
  lastPrintTime = millis();
}

void tachISR() {
  pulseCount++;
 
}


void setFanSpeed(float dutyCycle) {
   currentDutyCycle = dutyCycle;
  
  // Calculate PWM value based on resolution
  uint32_t pwmValue;
  
  if (PWM_RESOLUTION == 8) {
    pwmValue = (dutyCycle / 100.0) * 255;
  } 
  else if (PWM_RESOLUTION == 12) {
    pwmValue = (dutyCycle / 100.0) * 4095;  // 12-bit: 0-4095
  }
  else if (PWM_RESOLUTION == 16) {
    pwmValue = (dutyCycle / 100.0) * 65535; // 16-bit: 0-65535
  }
  
  ledcWrite(PWM_CHANNEL, pwmValue);
  
}

void setFanSpeedDither(float dutyCycle) {
  // Add small random variation to break up regular pulses
  float jitter = (random(-10, 10) / 100.0); // ±0.1% jitter
  float effectiveDuty = dutyCycle + jitter;
  
  uint32_t pwmValue = (effectiveDuty / 100.0) * 4095;
  ledcWrite(PWM_CHANNEL, pwmValue);
}

float readRPM() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastRpmCalcTime;
  
  // Only calculate RPM every 1 second
  if (elapsedTime < 1000) {
    return currentRPM;
  }
  
  // Get pulses and reset counter atomically
  noInterrupts();
  unsigned long pulses = pulseCount;
  pulseCount = 0;
  interrupts();
  
  if (pulses > 0) {
    // Calculate RPM: (pulses per second × 60) ÷ 2 pulses per revolution
    float pulsesPerSecond = (float)pulses / (elapsedTime / 1000.0);
    currentRPM = (pulsesPerSecond * 60.0) / 2.0;
    
    // Filter unrealistic values
    if (currentRPM < 100 || currentRPM > 3000) {
      currentRPM = 0;
    }
    
    Serial.print("Pulses: ");
    Serial.print(pulses);
    Serial.print(" | Time: ");
    Serial.print(elapsedTime);
    Serial.print("ms | RPM: ");
    Serial.println(currentRPM);
  } else {
    currentRPM = 0;
    Serial.println("No pulses detected");
  }
  
  lastRpmCalcTime = currentTime;
  return currentRPM;
}





int estimateRPM(byte duty) {
  if (duty <= pwmPoints[0]) return rpmPoints[0];
  for (int i = 0; i < sizeof(pwmPoints)/sizeof(pwmPoints[0])-1; i++) {
    if (duty >= pwmPoints[i] && duty <= pwmPoints[i+1]) {
      float ratio = (float)(duty - pwmPoints[i]) / (pwmPoints[i+1] - pwmPoints[i]);
      return rpmPoints[i] + ratio * (rpmPoints[i+1] - rpmPoints[i]);
    }
  }
  return rpmPoints[sizeof(rpmPoints)/sizeof(rpmPoints[0]) - 1];
}

float estimateAirflow(byte duty) {
  if (duty <= pwmPoints[0]) return airflowPoints[0];
  for (int i = 0; i < sizeof(pwmPoints)/sizeof(pwmPoints[0])-1; i++) {
    if (duty >= pwmPoints[i] && duty <= pwmPoints[i+1]) {
      float ratio = (float)(duty - pwmPoints[i]) / (pwmPoints[i+1] - pwmPoints[i]);
      return airflowPoints[i] + ratio * (airflowPoints[i+1] - airflowPoints[i]);
    }
  }
  return airflowPoints[sizeof(airflowPoints)/sizeof(airflowPoints[0]) - 1];
}


void loop(void) {
  //unsigned int rpms = calculateRPM();
  

 //setFanSpeedDither(currentDutyCycle);
  
  static unsigned long lastPrintTime = 0;
  if (millis() - lastPrintTime >= 1500) {
     unsigned int rpms = readRPM();
    Serial.print("RPM: ");
    Serial.print(rpms);
    Serial.print(" | Duty: ");
    Serial.print(currentDutyCycle);
    Serial.println("%");
    
    lastPrintTime = millis();
  
    
   //int rpm = estimateRPM(currentDutyCycle);
float airflow = estimateAirflow(currentDutyCycle);

  }
  
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');

    // number = set fan speed
    // P nnn = PWM frequency
    if (line.startsWith("p#")) {
      pwm_freq = line.substring(2).toInt();
      ledcSetup(PWM_CHANNEL, pwm_freq, PWM_RESOLUTION);
    } else {

    int input = line.toInt();
    float target = constrain(input, 0, 100);
    setFanSpeed(target);
    }
    
  }
  
  delay(50);
  setFanSpeedDither(currentDutyCycle);
}