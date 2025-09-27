#define TACH_PIN 3
#define PWM_PIN 9  // Change to Pin 9 or 10 (Timer 1)
#define PWM_MONITOR_PIN 2  // Separate pin to monitor PWM

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
  
  setPwmFrequency(PWM_PIN, 8);
  //setupPhaseCorrectPWM(3700); // 1 kHz PWM
  
  pinMode(PWM_PIN, OUTPUT);
 
  
  
    pinMode(PWM_MONITOR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(TACH_PIN), tachISR, FALLING);
  attachInterrupt(digitalPinToInterrupt(PWM_MONITOR_PIN), pwmMonitorISR, CHANGE);
  pinMode(TACH_PIN, INPUT);
  lastRpmCalcTime = millis();
  lastPrintTime = millis();
}





// Safe PWM frequency setting for Timer 1 (pins 9, 10)
void setPwmFrequency(int pin, int divisor) {
  if(pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: TCCR1B = (TCCR1B & 0b11111000) | 0x01; break; // 31.25 kHz
      case 8: TCCR1B = (TCCR1B & 0b11111000) | 0x02; break; // 3.91 kHz  
      
       case 256: TCCR1B = (TCCR1B & 0b11111000) | 0x04; break; // 122 Hz
      case 1024: TCCR1B = (TCCR1B & 0b11111000) | 0x05; break; // 30 Hz
      default: TCCR1B = (TCCR1B & 0b11111000) | 0x03; break; // 490 Hz
    }
  }
}

void pwmMonitorISR() {
  if (digitalRead(PWM_MONITOR_PIN) == LOW) {
    sampleWindowOpen = true;
  } else {
    sampleWindowOpen = false;
  }
}

void tachISR() {
  if (sampleWindowOpen) { // Only count pulses during PWM HIGH
    syncPulseCount++;
  }
}



 
unsigned int calculateRPM() {
  unsigned long currentTime = millis();
  unsigned long elapsedTime = currentTime - lastRpmCalcTime;
  
  if (elapsedTime < 1000) {
    return currentRPM;
  }
  
  // Calculate actual sampling time (micros for precision)
  unsigned long currentMicros = micros();
  unsigned long sampleDuration = currentMicros - lastSampleResetTime;
  
  // Prevent division by zero
  if (sampleDuration == 0) sampleDuration = 1;
  
  if (syncPulseCount > 0) {
    // Calculate pulses per second based on actual sampling time
    float pulsesPerSecond = (float)syncPulseCount / (sampleDuration / 1000000.0);
    
    // Critical: Adjust for duty cycle sampling
    // We only sample during PWM HIGH periods, so we need to extrapolate
    float samplingRatio = (float)currentDutyCycle / 100.0;
    
    // Avoid division by zero for very low duty cycles
    if (samplingRatio < 0.05) samplingRatio = 0.05; // Minimum 5%
    
    // Extrapolate to full second accounting for sampling ratio
    float fullSecondPulses = pulsesPerSecond / samplingRatio;
    
    // Calculate RPM (2 pulses per revolution for most fans)
    currentRPM = (fullSecondPulses * 60.0) / 2.0;
    
    // Filter unrealistic values
    if (currentRPM < 100 || currentRPM > 3000) {
      currentRPM = 0;
    }
    
    // Debug output
    Serial.print("Pulses: ");
    Serial.print(syncPulseCount);
    Serial.print(" | SampleTime: ");
    Serial.print(sampleDuration / 1000.0);
    Serial.print("ms | Duty: ");
    Serial.print(currentDutyCycle);
    Serial.print("% | RawPPS: ");
    Serial.print(pulsesPerSecond);
    Serial.print(" | AdjRPM: ");
    Serial.println(currentRPM);
    
  } else {
    currentRPM = 0;
    Serial.print("No pulses | Duty: ");
    Serial.print(currentDutyCycle);
    Serial.print("% | SampleTime: ");
    Serial.println(sampleDuration / 1000.0);
  }
  
  // Reset counters for next calculation
  syncPulseCount = 0;
  lastRpmCalcTime = currentTime;
  lastSampleResetTime = currentMicros;
  
  return currentRPM;
}
void setFanSpeed(byte dutyCycle) {
  currentDutyCycle = dutyCycle;
  
  Serial.print("Setting PWM: ");
  Serial.print(dutyCycle);
  Serial.println("%");
  
  if (dutyCycle == 0) {
    analogWrite(PWM_PIN, 0);
  } else {
    byte pwmValue = map(dutyCycle, 0, 100, 0, 255);
    analogWrite(PWM_PIN, pwmValue);
  }
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
  unsigned int rpms = calculateRPM();
  
  if (millis() - lastPrintTime >= 1500) {

    Serial.print("RPM: ");
    Serial.print(rpms);
    Serial.print(" | Duty: ");
    Serial.print(currentDutyCycle);
    Serial.println("%");
    
    int rpm = estimateRPM(currentDutyCycle);
float airflow = estimateAirflow(currentDutyCycle);


Serial.print("%, Estimated RPM: "); Serial.print(rpm);
Serial.print(", Airflow: "); Serial.print(airflow); Serial.println(" m³/h");

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