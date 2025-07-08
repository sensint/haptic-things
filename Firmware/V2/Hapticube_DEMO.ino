// Delta threshold sweetspots :  FSR > ~20/30 | Orientation > ~5/10 | Accelerometer > ~30/50
// Frequency sweetspots = 75 / 100 / 150 / 200 / 300

#include <Wire.h>
#include <Arduino.h>
#include <DaisyDuino.h>
#include <elapsedMillis.h>
#include <Adafruit_BNO08x.h>
#include <APA102.h>
#include <Adafruit_BNO08x.h>

// --- Serial Buffer for RX/TX COM --- //
#define SERIAL_BUFFER_SIZE 128
char serialBuffer[SERIAL_BUFFER_SIZE];
int serialBufferIndex = 0;

// --- FSR --- //
float smoothedPressureA, smoothedPressureB, smoothedPressureC = 0;
float alpha = 0.5;
int offset = 3;
int sensor0, sensor1, sensor2, sensor3, sensor4, sensor5;
float smoothBuffer0, smoothBuffer1, smoothBuffer2, smoothBuffer3, smoothBuffer4, smoothBuffer5, smoothBufferLin0, smoothBufferLin1, smoothBufferLin2, smoothBufferLin3, smoothBufferLin4, smoothBufferLin5, smoothBufferA, smoothBufferB, smoothBufferC;

// --- delta var --- //
long deltaCurrentA = 0.0;
long deltaCurrentB = 0.0;
long deltaCurrentC = 0.0;

long cumulativeDeltaA = 0;
long cumulativeDeltaB = 0;
long cumulativeDeltaC = 0;

double sensor_pressureA = 0.0;
double sensor_pressureB = 0.0;
double sensor_pressureC = 0.0;

double prev_sensor_pressureA = 0.0;
double prev_sensor_pressureB = 0.0;
double prev_sensor_pressureC = 0.0;

double delta_sensor_pressureA = 0.0;
double delta_sensor_pressureB = 0.0;
double delta_sensor_pressureC = 0.0;

double prev_imu_x = 0.0;
double prev_imu_y = 0.0;
double prev_imu_z = 0.0;

double delta_imu_x = 0.0;
double delta_imu_y = 0.0;
double delta_imu_z = 0.0;

// --- IMU --- //
#ifdef FAST_MODE
  // Top frequency is reported to be 1000Hz (but freq is somewhat variable)
  sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
  long reportIntervalUs = 2000;
#else
  // Top frequency is about 250Hz but this report is more accurate
  sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
  long reportIntervalUs = 5000;
#endif

#define BNO08X_RESET -1
Adafruit_BNO08x  bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

float imu_x, imu_y, imu_z;

// Variables to track angle for each axis
float prevAngle_x = 0, prevAngle_y = 0, prevAngle_z = 0;
float totalAngle_x = 0, totalAngle_y = 0, totalAngle_z = 0;
bool isFirstReading = true;  // Flag for the first reading

void quaternionToEuler(float qr, float qi, float qj, float qk, euler_t* ypr, bool degrees = false) {

    float sqr = sq(qr);
    float sqi = sq(qi);
    float sqj = sq(qj);
    float sqk = sq(qk);

    ypr->yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
    ypr->pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
    ypr->roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

    if (degrees) {
      ypr->yaw *= RAD_TO_DEG;
      ypr->pitch *= RAD_TO_DEG;
      ypr->roll *= RAD_TO_DEG;
    }
}

void quaternionToEulerRV(sh2_RotationVectorWAcc_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

// ============ Audio/Haptics Flow signal variables ============ //
elapsedMicros pulseTimeUS = 0;

bool isVibrating = false;
bool shouldVibrate = false;

float deltaTresh = 15.0;  // threshohold imu = 100,  fsr = ~10/30 
float ampValue = 0.0;
float newAmpValue = 0.70; // keep < 0.7 to prevent amp distortion
int freqValue = 150; // 70hz resonant freq for hap MKII - i prefer 100 or 150 or 200Hz, oher are too audible at my onion - base freq varifreq is  -/+ 20%
uint32_t signalDuration;

const int sampleRateHz = 100;
unsigned long sampleInterval = 1000 / sampleRateHz;  // Convert Hz to milliseconds
unsigned long lastSampleTime = 0;                    // Time of the last sample
elapsedMillis timer_imu_data = 0;
unsigned long prevTime = 0;
int variFreq = freqValue;

// ================ DaisyVariables ===============
DaisyHardware hw;
Svf filterLow;
Svf filterHigh;
size_t num_channels;
static Oscillator osc;

// ----------- SERIAL READ Variables ----------------- //
String inputString;
int input0 = 0, input1 = 0, input3 = 0, input4 = 0;
float input2;

// --------- Print control -------------- //
int printRefreshRate = 20 * 1000; // in microseconds - 20ms / ~50hz -
unsigned long previousPrintMicros = 0;
bool debugPrint = false;

// --- LEDS --- //
const uint8_t dataPin = 0;
const uint8_t clockPin = 1;
const uint16_t ledCount = 1;
float brightness = 0;
rgb_color colors[ledCount];

APA102<dataPin, clockPin> ledStrip;

// --- Mode management --- //
int modeValue = 1; // 0 -> OFF | 1 -> FSR | 2 -> Orientation | 3 -> Accelerometer | [...]
int axisValue = 0; //  0 = xyz | 1 = x | 2 = y | 3 = z | 4 = xy | 5 = yz | 6 = xz

// --- Callback --- //
void MyCallback(float** in, float** out, size_t size) {
  float sig;
  osc.SetAmp(ampValue);
  osc.SetFreq(freqValue);
  osc.SetWaveform(osc.WAVE_SIN);
  for (size_t i = 0; i < size; i++) {
    sig = osc.Process();
    out[0][i] = sig;
    out[1][i] = sig;
  }
}

// --- Setup --- //
void setup() {
  Serial.begin(115200); // debug serial
  Serial1.begin(9600);  // Use Serial1 for hardware UART
  Wire.begin();

  float sample_rate;
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  num_channels = hw.num_channels;
  sample_rate = DAISY.get_samplerate();
  osc.Init(sample_rate);
  osc.SetFreq(freqValue);
  osc.SetAmp(0.0);
  osc.SetWaveform(osc.WAVE_SIN);
  DAISY.begin(MyCallback);
  prevTime = millis();  // Initialize time
  delay(1000);

  Serial.println("Bonjour, Hola, Hello :-)");
  Serial.println("Adafruit BNO08x test!");
  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");
  setReports(reportType, reportIntervalUs);

  delay(1000);
  //setReports();
}

// --- Loop --- //

void loop() {

  unsigned long currentPrintMicros = micros();
  checkSerial();

  //////////////////
  //   OFF MODE   //
  //////////////////

  if (modeValue == 0) {
    checkSerial();
    // --- LED management --- // just blink red
    brightness = 15 ; // 
    for(uint16_t i = 0; i < ledCount; i++) {
        colors[i] = rgb_color(255,0,0);
        ledStrip.write(colors, ledCount, brightness);
      }
    delay(100);
    brightness = 0;
    for(uint16_t i = 0; i < ledCount; i++) {
        colors[i] = rgb_color(255,0,0);
        ledStrip.write(colors, ledCount, brightness);
      }
    delay(100);
  }

  //////////////////
  //   FSR MODE   //
  //////////////////

  if (modeValue == 1) {
    checkSerial();
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 0); // disable bno reports, prevent grounding issues

    // --- LED management --- //
    brightness = 5 + ((sensor_pressureA + sensor_pressureB + sensor_pressureC) / 150) ; // adjust brightness level depending of pressure
    for(uint16_t i = 0; i < ledCount; i++) {
        colors[i] = rgb_color(sensor_pressureA / 5, sensor_pressureB / 5, sensor_pressureC / 5); // adjust color depending on group pressed
        ledStrip.write(colors, ledCount, brightness);
      }

    // --- READ FSR VALUES --- //
    sensor0 = smooth(analogRead(A0), &smoothBuffer0, 0.6); // subtle smoothing before linearization, 0.6 is enough
    sensor1 = smooth(analogRead(A1), &smoothBuffer1, 0.6);
    sensor2 = smooth(analogRead(A2), &smoothBuffer2, 0.6);
    sensor3 = smooth(analogRead(A3), &smoothBuffer3, 0.6);
    sensor4 = smooth(analogRead(A4), &smoothBuffer4, 0.6);
    sensor5 = smooth(analogRead(A5), &smoothBuffer5, 0.6);
    
    // --- Linearize FSR response --- //
    double linSensor0 = smooth(linearize(sensor0, 4), &smoothBufferLin0, 0.5); // exponent 2-4 give good results, alpha of 0.4 - 0.5
    double linSensor1 = smooth(linearize(sensor1, 4), &smoothBufferLin1, 0.5);
    double linSensor2 = smooth(linearize(sensor2, 4), &smoothBufferLin2, 0.5);
    double linSensor3 = smooth(linearize(sensor3, 4), &smoothBufferLin3, 0.5);
    double linSensor4 = smooth(linearize(sensor4, 4), &smoothBufferLin4, 0.5);
    double linSensor5 = smooth(linearize(sensor5, 4), &smoothBufferLin5, 0.5);

    // --- Group FSR --- //

    float groupA; // Top/Bot      || A0/A5
    float groupB; // East/West    || A2/A4
    float groupC; // North/South  || A1/A3

    if (axisValue == 0) {
    groupA = abs((linSensor0 + linSensor5) /2 - offset); // Top/Bot      || A0/A5
    groupB = abs((linSensor2 + linSensor4) /2 - offset); // East/West    || A2/A4
    groupC = abs((linSensor3 + linSensor1) /2 - offset); // North/South  || A1/A3
    }

    if (axisValue == 1) {
    groupA = abs((linSensor0 + linSensor5) /2 - offset);
    groupB = 0; 
    groupC = 0; 
    }

    if (axisValue == 2) {
    groupA = 0; 
    groupB = abs((linSensor2 + linSensor4) /2 - offset); 
    groupC = 0; 
    }

    if (axisValue == 3) {
    groupA = 0;
    groupB = 0;
    groupC = abs((linSensor3 + linSensor1) /2 - offset);
    }

    if (axisValue == 4) {
    groupA = abs((linSensor0 + linSensor5) /2 - offset);
    groupB = abs((linSensor2 + linSensor4) /2 - offset);
    groupC = 0;
    }

    if (axisValue == 5) {
    groupA = 0;
    groupB = abs((linSensor2 + linSensor4) /2 - offset);
    groupC = abs((linSensor3 + linSensor1) /2 - offset);
    }

    if (axisValue == 6) {
    groupA = abs((linSensor0 + linSensor5) /2 - offset);
    groupB = 0;
    groupC = abs((linSensor3 + linSensor1) /2 - offset);
    }

    if(groupA < 3){groupA = 3;} // ignore base noise
    if(groupB < 3){groupB = 3;}
    if(groupC < 3){groupC = 3;}
      
    sensor_pressureA = int(adaptiveSmooth(groupA, &smoothBufferA, 0.4)); // some additional adaptive smoothing here, 0.4 - 0.6
    sensor_pressureB = int(adaptiveSmooth(groupB, &smoothBufferB, 0.4));
    sensor_pressureC = int(adaptiveSmooth(groupC, &smoothBufferC, 0.4));

    // --- Delta Calculations --- //
    delta_sensor_pressureA = abs(sensor_pressureA - prev_sensor_pressureA);
    delta_sensor_pressureB = abs(sensor_pressureB - prev_sensor_pressureB);
    delta_sensor_pressureC = abs(sensor_pressureC - prev_sensor_pressureC);

    deltaCurrentA = delta_sensor_pressureA;
    deltaCurrentB = delta_sensor_pressureB;
    deltaCurrentC = delta_sensor_pressureC;

    cumulativeDeltaA += deltaCurrentA;
    cumulativeDeltaB += deltaCurrentB;
    cumulativeDeltaC += deltaCurrentC;

    prev_sensor_pressureA = sensor_pressureA;
    prev_sensor_pressureB = sensor_pressureB;
    prev_sensor_pressureC = sensor_pressureC;

    // --- Haptic triggers --- //
    generateTrigger(&cumulativeDeltaA, deltaTresh);
    generateTrigger(&cumulativeDeltaB, deltaTresh); 
    generateTrigger(&cumulativeDeltaC, deltaTresh); 

    GenerateGrain(freqValue, &shouldVibrate, &ampValue, newAmpValue);

    // --- Print --- //
    if (currentPrintMicros - previousPrintMicros >= printRefreshRate && debugPrint > 0) {
      Serial.print(modeValue);
      Serial.print(" ");
      Serial.print(axisValue);
      Serial.print(" ");
      Serial.print(freqValue);
      Serial.print(" ");
      Serial.print(newAmpValue);
      Serial.print(" ");
      Serial.print(deltaTresh);
      Serial.print(" ");

      // Serial.print(sensor0);
      // Serial.print(" ");
      // Serial.print(sensor1);
      // Serial.print(" ");
      // Serial.print(sensor2);
      // Serial.print(" ");
      // Serial.print(sensor3);
      // Serial.print(" ");
      // Serial.print(sensor4);
      // Serial.print(" ");
      // Serial.print(sensor5);
      // Serial.print(" ");

      // Serial.print(linSensor0);
      // Serial.print(" ");
      // Serial.print(linSensor1);
      // Serial.print(" ");
      // Serial.print(linSensor2);
      // Serial.print(" ");
      // Serial.print(linSensor3);
      // Serial.print(" ");
      // Serial.print(linSensor4);
      // Serial.print(" ");
      // Serial.print(linSensor5);
      // Serial.print(" ");

      Serial.print((sensor0 + sensor5) / 2);
      Serial.print(" ");
      Serial.print((sensor2 + sensor4) / 2);
      Serial.print(" ");
      Serial.print((sensor1 + sensor3) / 2);
      Serial.print(" ");

      Serial.print(sensor_pressureA);
      Serial.print(" ");
      Serial.print(sensor_pressureB);
      Serial.print(" ");
      Serial.print(sensor_pressureC);
      Serial.print(" ");

      // Serial.print(cumulativeDeltaA);
      // Serial.print(" ");
      // Serial.print(cumulativeDeltaB);
      // Serial.print(" ");
      // Serial.print(cumulativeDeltaC);
      Serial.println();
      previousPrintMicros = currentPrintMicros;
    }

    delay(5); // smoll delay to limit jitter in FSR
  }

  //////////////////////////
  //   ORIENTATION MODE   //
  //////////////////////////

  if (modeValue == 2) {
    checkSerial();

    // --- LED management --- //
    brightness = 15 ; //
    for(uint16_t i = 0; i < ledCount; i++) {
        colors[i] = rgb_color(abs(imu_x) * 1.4,abs(imu_y) * 1.4, abs(imu_z) * 1.4); // adjust color depending on angle
        ledStrip.write(colors, ledCount, brightness);
      }  

    // --- imu readings --- //
    bno08x.enableReport(SH2_ACCELEROMETER, 0); // block bno accel reports 
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, reportIntervalUs); // enable bno orientation reports

    if (bno08x.getSensorEvent(&sensorValue)) {
        
      quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);

      imu_x = ypr.yaw;
      imu_y = ypr.pitch;
      imu_z = ypr.roll;

      imu_y = mapAngle((imu_y + 90) / 180); // everyon goes to [-180,180] range
      updateTotalAngles(imu_x, imu_y, imu_z); // unwrapping the angles
    }

    // --- Axis mode --- // 

    if (axisValue == 0) {
    }

    if (axisValue == 1) {
      imu_y = 0;
      imu_z = 0;
    }

    if (axisValue == 2) {
      imu_x = 0;
      imu_z = 0;
    }

    if (axisValue == 3) {
    imu_x = 0;
    imu_y = 0;
    }

    if (axisValue == 4) {
    imu_z = 0;
    }

    if (axisValue == 5) {
    imu_x = 0;
    }

    if (axisValue == 6) {
    imu_y = 0;
    }

    // --- Delta Calculations --- //
    delta_imu_x = abs(imu_x - prev_imu_x);
    delta_imu_y = abs(imu_y - prev_imu_y);
    delta_imu_z = abs(imu_z - prev_imu_z);

    deltaCurrentA = delta_imu_x;
    deltaCurrentB = delta_imu_y;
    deltaCurrentC = delta_imu_z;

    cumulativeDeltaA += deltaCurrentA;
    cumulativeDeltaB += deltaCurrentB;
    cumulativeDeltaC += deltaCurrentC;

    prev_imu_x = imu_x;
    prev_imu_y = imu_y;
    prev_imu_z = imu_z;

    // --- Serial print --- //
    if (currentPrintMicros - previousPrintMicros >= printRefreshRate && debugPrint > 0) {
      Serial.print(freqValue);
      Serial.print(" ");
      Serial.print(newAmpValue);
      Serial.print(" ");
      Serial.print(deltaTresh);
      Serial.print(" ");

      Serial.print(imu_x);
      Serial.print(" ");
      Serial.print(imu_y);
      Serial.print(" ");
      Serial.print(imu_z);
      Serial.print(" ");

      Serial.println();
      previousPrintMicros = currentPrintMicros;
    }

    // --- Haptic triggers --- //

    generateTrigger(&cumulativeDeltaA, deltaTresh);
    generateTrigger(&cumulativeDeltaB, deltaTresh); 
    generateTrigger(&cumulativeDeltaC, deltaTresh); 

    GenerateGrain(freqValue, &shouldVibrate, &ampValue, newAmpValue);
    delay(5);
  }

  ////////////////////////////
  //   ACCELEROMETER MODE   //
  ////////////////////////////

  if (modeValue == 3) {
    checkSerial();

    // --- LED management --- //
    brightness = 15 ; //
    for(uint16_t i = 0; i < ledCount; i++) {
        colors[i] = rgb_color(abs(imu_x) * 2,abs(imu_y) * 2, abs(imu_z) * 2); // adjust color depending on angle
        ledStrip.write(colors, ledCount, brightness);
    } 

    // --- imu readings --- //
    bno08x.enableReport(SH2_ARVR_STABILIZED_RV, 0); // disable bno orient reports
    bno08x.enableReport(SH2_ACCELEROMETER, reportIntervalUs); // enable bno accel reports 

    if (bno08x.getSensorEvent(&sensorValue)) {
  
      if (sensorValue.sensorId == SH2_ACCELEROMETER) {
      imu_x = sensorValue.un.accelerometer.x * 5; // *10 to make everythin more sensitive
      imu_y = sensorValue.un.accelerometer.y * 5;
      imu_z = sensorValue.un.accelerometer.z * 5;

      }
    }

    // --- Axis mode --- // 

    if (axisValue == 0) {
    }

    if (axisValue == 1) {
      imu_y = 0;
      imu_z = 0;
    }

    if (axisValue == 2) {
      imu_x = 0;
      imu_z = 0;
    }

    if (axisValue == 3) {
    imu_x = 0;
    imu_y = 0;
    }

    if (axisValue == 4) {
    imu_z = 0;
    }

    if (axisValue == 5) {
    imu_x = 0;
    }

    if (axisValue == 6) {
    imu_y = 0;
    }

    // --- Delta Calculations --- // 
    delta_imu_x = abs(imu_x - prev_imu_x);
    delta_imu_y = abs(imu_y - prev_imu_y);
    delta_imu_z = abs(imu_z - prev_imu_z);

    deltaCurrentA = delta_imu_x;
    deltaCurrentB = delta_imu_y;
    deltaCurrentC = delta_imu_z;

    cumulativeDeltaA += deltaCurrentA;
    cumulativeDeltaB += deltaCurrentB;
    cumulativeDeltaC += deltaCurrentC;

    prev_imu_x = imu_x;
    prev_imu_y = imu_y;
    prev_imu_z = imu_z;

    // --- Serial print --- //
    if (currentPrintMicros - previousPrintMicros >= printRefreshRate && debugPrint > 0) {
      Serial.print(freqValue);
      Serial.print(" ");
      Serial.print(newAmpValue);
      Serial.print(" ");
      Serial.print(deltaTresh);
      Serial.print(" ");

      Serial.print(imu_x);
      Serial.print(" ");
      Serial.print(imu_y);
      Serial.print(" ");
      Serial.print(imu_z);
      Serial.print(" ");

      Serial.print(cumulativeDeltaA);
      Serial.print(" ");
      Serial.print(cumulativeDeltaB);
      Serial.print(" ");
      Serial.print(cumulativeDeltaC);

      Serial.println();
      previousPrintMicros = currentPrintMicros;
    }

    // --- Haptic triggers --- //
    
    generateTrigger(&cumulativeDeltaA, deltaTresh);
    generateTrigger(&cumulativeDeltaB, deltaTresh); 
    generateTrigger(&cumulativeDeltaC, deltaTresh); 

    GenerateGrain(freqValue, &shouldVibrate, &ampValue, newAmpValue);
    delay(5);
  }
}

/////////////////////////////////////////
//     FUNCTIONS FOR GRAIN FEEDBACK    //
/////////////////////////////////////////

void GenerateGrain(float freq, bool* vibrateFlag, float* amplitude_value, float newAmpValue) {
  signalDuration = 1000 * 1000 / freqValue;  // in µs

  if (isVibrating && (pulseTimeUS >= signalDuration)) {
    StopPulse(amplitude_value);
  }

  if (*vibrateFlag == true) {
    if (isVibrating && (pulseTimeUS >= signalDuration)) {
      StopPulse(amplitude_value);
    }
    if(pulseTimeUS >=  signalDuration) {//random(signalDuration / 2, signalDuration * 2)) { // minimum of 1 cycle space between grains to prevent grain overlap (maybe can add a random factor here too)
      //variFreq = random(freq * 0.8, freq * 1.2);
      StartPulse(freqValue, amplitude_value, newAmpValue);
    *vibrateFlag = false;
    }
  } 
}

void StartPulse(float freq, float* ampValue, float newAmpVal) {
  *ampValue = float(random((newAmpVal*10) / 2 ,newAmpVal * 10) / 10.0); // adding a small random here to make grains feel more 'natural'
  //Serial.print("amp value = "); Serial.println(*ampValue);
  osc.Reset();
  pulseTimeUS = 0;
  isVibrating = true;
}

void StopPulse(float* ampValue) {
  osc.Reset();
  *ampValue = 0.0;
  pulseTimeUS = 0;
  isVibrating = false;
}

void generateTrigger(long* cumulativeDelta, float deltaTreshold) {
  if (*cumulativeDelta >= deltaTreshold) {
    shouldVibrate = true;
    *cumulativeDelta = 0.0;
  } 
}

//////////////////////////////////////////
//      FUNCTIONS FOR FSR SENSING       //
//////////////////////////////////////////

float adaptiveSmooth(float inputValue, float* buffer, float smoothCurve) {
  float smoothingFactor= (smoothCurve + 0.01) - (smoothCurve * (float(inputValue) / 1023));
  *buffer = (smoothingFactor * inputValue) + ((1 - smoothingFactor) * *buffer);
  //Serial.print("Adaptive smooth = ");Serial.println(smoothingFactor);
  return *buffer;
}

float smooth(float inputValue, float* buffer, float alphaValue){
  *buffer = alphaValue * inputValue + (1 - alphaValue) * *buffer;
  return *buffer;
}

double linearize(float inputValue, int exponent){
  double linearizedValue = pow(11, pow(inputValue / 1023.0, exponent));
  return (linearizedValue - 1) * 102.3;
}


//////////////////////////////////////////
//      FUNCTIONS FOR SERIAL READ       //
//////////////////////////////////////////

// Check if a string is a valid int
bool isNumber(String s) {
  s.trim();
  if (s.length() == 0) return false;
  for (int i = 0; i < s.length(); i++) {
    if (!isDigit(s[i]) && !(i == 0 && s[i] == '-')) return false;
  }
  return true;
}

// Check if a string is a valid float
bool isFloat(String s) {
  s.trim();
  if (s.length() == 0) return false;
  bool dotSeen = false;
  for (int i = 0; i < s.length(); i++) {
    if (s[i] == '.') {
      if (dotSeen) return false;
      dotSeen = true;
    } else if (!isDigit(s[i]) && !(i == 0 && s[i] == '-')) {
      return false;
    }
  }
  return true;
}

void processCommand(char* cmd) {
  // Check it's a SET command
  if (strncmp(cmd, "SET,", 4) != 0) return;

  // Create a local copy for safe tokenization
  char localCopy[SERIAL_BUFFER_SIZE];
  strncpy(localCopy, cmd + 4, SERIAL_BUFFER_SIZE - 1);
  localCopy[SERIAL_BUFFER_SIZE - 1] = '\0';

  // Tokenize and store
  char* tokens[5] = {nullptr};
  char* token = strtok(localCopy, ",");
  int i = 0;

  while (token != nullptr && i < 5) {
    tokens[i++] = token;
    token = strtok(nullptr, ",");
  }

  // Check we got exactly 5 parameters
  if (i != 5) {
    Serial1.println("ERR");
    Serial.println("ERR sent");
    return;
  }

  // Parse and assign values
  modeValue = atoi(tokens[0]);
  axisValue = atoi(tokens[1]);
  freqValue = atoi(tokens[2]);
  newAmpValue = atof(tokens[3]);
  if (newAmpValue < 0 || newAmpValue > 0.7) newAmpValue = 0.7;
  deltaTresh = atoi(tokens[4]);

  // Acknowledge only when all is OK
  Serial1.println("ACK");
  Serial.println("ACK sent");
  return;
}


void checkSerial() {
  while (Serial1.available()) {
    char c = Serial1.read();
    Serial.print(c);

    if (c == '\r' || c == '\n') {
      continue;
    }

    if (c != ';' && serialBufferIndex < SERIAL_BUFFER_SIZE - 1) {
      serialBuffer[serialBufferIndex++] = c;
    } else if (c == ';') {
      serialBuffer[serialBufferIndex] = '\0';
      processCommand(serialBuffer);
      serialBufferIndex = 0;
    } else if (serialBufferIndex >= SERIAL_BUFFER_SIZE - 1) {
      // Prevent buffer overflow from junk
      serialBufferIndex = 0;
    }
  }
}


///////////////////////////////
//     FUNCTION FOR IMU       //
///////////////////////////////

void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (! bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
    quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

float mapAngle(float input) {
  return fmap(input, -180.0, 180.0);
}

float toRad(float inputAngle) {
  return inputAngle * (PI / 180.0);
}

// Function to update all three axes
void updateTotalAngles(float current_x, float current_y, float current_z) {
  // Skip the first reading comparison, just initialize
  if (isFirstReading) {
    prevAngle_x = current_x;
    prevAngle_y = current_y;
    prevAngle_z = current_z;
    isFirstReading = false;
    return;
  }
  
  // Update X-axis
  float diff_x = current_x - prevAngle_x;
  if (diff_x > 180) diff_x -= 360;
  else if (diff_x < -180) diff_x += 360;
  totalAngle_x += diff_x;
  prevAngle_x = current_x;
  
  // Update Y-axis
  float diff_y = current_y - prevAngle_y;
  if (diff_y > 180) diff_y -= 360;
  else if (diff_y < -180) diff_y += 360;
  totalAngle_y += diff_y;
  prevAngle_y = current_y;
  
  // Update Z-axis
  float diff_z = current_z - prevAngle_z;
  if (diff_z > 180) diff_z -= 360;
  else if (diff_z < -180) diff_z += 360;
  totalAngle_z += diff_z;
  prevAngle_z = current_z;
  
  // Convert to radians if needed 
  float totalXRad = toRad(totalAngle_x);
  float totalYRad = toRad(totalAngle_y);
  float totalZRad = toRad(totalAngle_z);
}


//   s e e   u   c y b e r s p a c e   c o w b o y   //
