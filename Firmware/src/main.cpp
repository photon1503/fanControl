#include <Arduino.h>

void setFanSpeed(byte dutyCycle);
void setPwmFrequency(int pin, int divisor);
void tachISR();

#define TACH_PIN 4
#define PWM_PIN 16 // Change to Pin 9 or 10 (Timer 1)
#define PWM_FREQ        30      // 30Hz for 3-pin fan
#define PWM_RESOLUTION  12      // 12-bit = 4096 steps
#define PWM_CHANNEL     0       // Use channel 0



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
// Analog tach variables
unsigned long analogPulseCount = 0;
int lastAnalogValue = 0;
bool lastPulseState = false;
int lowThreshold = 0;
int highThreshold = 0;
bool thresholdsCalculated = false;

const byte pwmPoints[] = {0,10, 20, 30, 50, 70, 100};
const int rpmPoints[]  = {0,225, 450, 900, 1800, 2000, 2200};
const float airflowPoints[] = {0,5.8, 11.5, 23, 39, 48, 55.5};

void setup(void) {
  Serial.begin(115200);
  Serial.println("Fan Controller - Correct Timing");
  
  // Setup PWM with high resolution
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  pinMode(TACH_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);
  lastRpmCalcTime = millis();
  lastPrintTime = millis();
}

void tachISR() {
 
  // Conservative 15ms debounce
  pulseCount++;
 
}


void setFanSpeed(byte dutyCycle) {
  currentDutyCycle = dutyCycle;
  uint32_t pwmValue = (dutyCycle / 100.0) * 4095;
  ledcWrite(PWM_CHANNEL, pwmValue);
}

float readRPM() {
  unsigned long startTime = millis();
  pulseCount = 0;
  
  // Count pulses for 1 second
  while (millis() - startTime < 1000) {
    delay(10);
  }
  
  unsigned long pulses = pulseCount;
  return (pulses * 60.0) / 2.0; // 2 pulses per revolution
}

unsigned int calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastRpmCalcTime;
  
  if (elapsedTime < 1000) {
    return currentRPM;
  }
  
  // Reset thresholds for new measurement (optional)
  //thresholdsCalculated = false;
  
  // Count pulses
  unsigned long pulseCount = 0;
  unsigned long sampleStart = millis();
  

  
  if (pulseCount > 0) {
    currentRPM = (pulseCount * 60) / 2;
    
    if (currentRPM < 100 || currentRPM > 3000) {
      currentRPM = 0;
    }
    
    Serial.print("Pulses: ");
    Serial.print(pulseCount);
    Serial.print(" | RPM: ");
    Serial.println(currentRPM);
  } else {
    currentRPM = 0;
    Serial.println("No pulses");
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
  
  if (millis() - lastPrintTime >= 1500) {

    unsigned long pulses = pulseCount;
     float rpm = (pulses * 60.0) / 1.5;
     pulseCount = 0;
    
    Serial.print("Pulses: "); Serial.print(pulses);
    Serial.print(" | RPM: "); Serial.println(rpm);

    Serial.print(" | Duty: ");
    Serial.print(currentDutyCycle);
    Serial.println("%");
    
   //int rpm = estimateRPM(currentDutyCycle);
float airflow = estimateAirflow(currentDutyCycle);


//Serial.print("%, Estimated RPM: "); Serial.print(rpm);
//Serial.print(", Airflow: "); Serial.print(airflow); Serial.println(" m³/h");

    lastPrintTime = millis();
  }
  
  if (Serial.available() > 0) {
    String line = Serial.readStringUntil('\n');
    int input = line.toInt();
    byte target = constrain(input, 0, 100);
    setFanSpeed(target);
    
  }
  
  delay(50);
}