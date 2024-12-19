#include <DaisyDuino.h>
#include <math.h>
#include <Wire.h>
#include <elapsedMillis.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <APA102.h>
#include <Arduino.h>
#include <Adafruit_BNO08x.h>

//          /----------------------------------> Calculate Speed ----------------------------------\                                                        
// Sensor -/---> Calculate Position \---> Generate Triggers --> Generate Grain ---/---> Filter --> Compressor --> Amplitude Modulation
//                     (optional)    \--> Generate Triggers --> Generate Grain --/

// ======= DaisyVariables =========
DaisyHardware hw;
Svf filterLow;
Svf filterHigh;
size_t num_channels;
static Oscillator osc;

// ====== Audio/Haptics Flow signal variables ======
elapsedMicros pulseTimeUS = 0;
bool isVibrating = false;
bool shouldVibrate = false;

float deltaSensor = 1.0;
long deltaCurrent = 0.0;
long cumulativeDelta = 0;

float ampValue;
float newAmpValue = 0.7;
int freqValue = 200;
uint32_t signalDuration;

const float amplitudeValues[] = { 0.9, 0.8, 0.5 };
const float frequencyValues[] = { 50.0, 40.0, 60.0 };
const float deltaValues[] = { 40.0, 20.0, 10.0 };

// ======= IMU ========
// #define FAST_MODE

// For SPI mode, we also need a RESET
//#define BNO08X_RESET 5
// but not for I2C or UART
#define BNO08X_RESET -1

struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

#ifdef FAST_MODE
// Top frequency is reported to be 1000Hz (but freq is somewhat variable)
sh2_SensorId_t reportType = SH2_GYRO_INTEGRATED_RV;
long reportIntervalUs = 2000;
#else
// Top frequency is about 250Hz but this report is more accurate
sh2_SensorId_t reportType = SH2_ARVR_STABILIZED_RV;
long reportIntervalUs = 5000;
#endif
void setReports(sh2_SensorId_t reportType, long report_interval) {
  Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    Serial.println("Could not enable stabilized remote vector");
  }
}

const int sampleRateHz = 100;
unsigned long sampleInterval = 1000 / sampleRateHz;  // Convert Hz to milliseconds
unsigned long lastSampleTime = 0;                    // Time of the last sample
elapsedMillis timer_imu_data = 0;
double m_x, m_y, m_z = 0.0;
double prev_mx, prev_my, prev_mz = 0.0;
double delta_x, delta_y, delta_z = 0.0;

double velocityX = 0;
double positionX = 0;
double prev_positionX = 0;
double velocityY = 0;
double positionY = 0;
double velocityZ = 0;
double positionZ = 0;
unsigned long prevTime = 0;

// ====== LED ======
const uint8_t dataPin = 10;
const uint8_t clockPin = 8;
APA102<dataPin, clockPin> ledStrip;
// Set the number of LEDs to control.
const uint16_t ledCount = 3;
rgb_color colors[ledCount];
const uint8_t brightness = 1;

// ==== Mean value speed ======
const int bufferSize = 50;  // Number of values to store
long buffer[bufferSize];    // Array to store values
int indexX = 0;             // Current indexX in the buffer
long sum = 0.0;             // Sum of all values in the buffer
bool bufferFull = false;    // Flag to indicate if the buffer is full

// ===== Serial communication + LED =====
bool dataRequested = false;
uint8_t r, g, b = 255;


void MyCallback(float** in, float** out, size_t size) {
  float sig;

  osc.SetAmp(ampValue);
  osc.SetFreq(freqValue);
  osc.SetWaveform(osc.WAVE_SIN);
  for (size_t i = 0; i < size; i++) {
    sig = osc.Process();
    // filterLow.Process(sig);
    // filterHigh.Process(filterLow.Low());

    // out[0][i] = filterHigh.High();
    // out[1][i] = filterHigh.High();

    out[0][i] = sig;
    out[1][i] = sig;
  }
}

//////////////////////////////////////////
// START - FUNCTIONS FOR GRAIN FEEDBACK //
//////////////////////////////////////////

// this function triggers a grain
void generateTrigger() {
  if (cumulativeDelta >= deltaSensor) {  //deltasensor should be a parameter, so we can modulate it
    shouldVibrate = true;
    cumulativeDelta = 0.0;

  } else {
    shouldVibrate = false;
  }
}

// this function plays the grain
void GenerateGrain(float freq, bool* vibrateFlag, float* amplitude_value, float newAmpValue) {

  // stop the OSC if the current vibration reached it cycle
  signalDuration = 1000 * 1000 / freq;  // 1 cicle in ms
  if (isVibrating && (pulseTimeUS >= signalDuration)) {
    StopPulse(amplitude_value);
  }

  // if there is a new bin
  //check if a grain is playing,
  //stop it if it is,
  //start a puls

  if (*vibrateFlag == true) {
    // stop the current vibration and play the next one.
    if (isVibrating) {  // This loop is for the case when we want to stop the ongoing vibration and start the next one.
      StopPulse(amplitude_value);
    }

    StartPulse(freq, amplitude_value, newAmpValue);
    *vibrateFlag = false;
  }
}

void StartPulse(float freq, float* ampValue, float newAmpVal) {
  *ampValue = newAmpVal;
  osc.Reset();  // phase to 0.0
  pulseTimeUS = 0;
  isVibrating = true;
}

void StopPulse(float* ampValue) {
  //osc.Reset();  // phase to 0.0
  *ampValue = 0.0;
  isVibrating = false;
}

void mix() {
  //if using multiple oscillators
}

void filter() {
  //seperatly set lowpass and highpass (probably with low resonance)
  //cutoff values should be passed as parameters, so they can be used for modulation
}

void compressor() {
  //gentle compression
}

void modulateAmplitude() {
  // think VCA
  //change the amplitude based on movement speed
  //either takes the amplitude as parameter, or takes a sensor value that then changes the amplitude.
  //for example, speed could be the envelop
}

//////////////////////////////////////////
// END - FUNCTIONS FOR GRAIN FEEDBACK   //
//////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens


  // IMU init
  Serial.println("Adafruit BNO08x test!");

  // Try to initialize!
  if (!bno08x.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  Serial.println("BNO08x Found!");


  setReports(reportType, reportIntervalUs);

  Serial.println("Reading events");
  delay(100);

  // Initialize for Daisy pod at 48kHz
  float sample_rate;
  hw = DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  num_channels = hw.num_channels;
  sample_rate = DAISY.get_samplerate();
  osc.Init(sample_rate);

  // Oscilator starting variable
  osc.SetFreq(freqValue);
  osc.SetAmp(0.1);
  osc.SetWaveform(osc.WAVE_SIN);

  // Initialize Filters
  filterLow.Init(sample_rate);
  filterLow.SetFreq(500.0);
  filterLow.SetRes(0.85);
  filterLow.SetDrive(0.8);

  filterHigh.Init(sample_rate);
  filterHigh.SetFreq(50.0);
  filterHigh.SetRes(0.85);
  filterHigh.SetDrive(0.8);

  // Daisy
  DAISY.begin(MyCallback);

  // Initialize the buffer with zeros
  for (int i = 0; i < bufferSize; i++) {
    buffer[i] = 0;
  }

  prevTime = millis(); // Initialize time
}

void loop() {
  // ===== Read data from IMU =======
  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports(reportType, reportIntervalUs);
  }

  //if (timer_imu_data >= sampleInterval) {
  // timer_imu_data = 0;  // Update the last sample time

  if (bno08x.getSensorEvent(&sensorValue)) {
    // in this demo only one report type will be received depending on FAST_MODE define (above)


    // Get accelarion values
    m_x = sensorValue.un.linearAcceleration.x;
    m_y = sensorValue.un.linearAcceleration.y;
    m_z = sensorValue.un.linearAcceleration.z;

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert ms to seconds
    prevTime = currentTime;

    // delta_x = fabs(m_x - prev_mx) * 10.0;
    // delta_y = fabs(m_y - prev_my) * 10.0;
    // delta_z = fabs(m_z - prev_mz) * 10.0;

    velocityX += m_x * deltaTime;
    velocityY += m_y * deltaTime;
    velocityZ += m_z * deltaTime;

    positionX += velocityX * deltaTime;
    positionY += velocityY * deltaTime;
    positionZ += velocityZ * deltaTime;

    delta_x = fabs(positionX - prev_positionX);
    deltaCurrent = delta_x;
    cumulativeDelta += deltaCurrent;
    prev_positionX = positionX;

    // Print results
  Serial.print("Acceleration (X): ");
  Serial.print(m_x);
  Serial.print(" m/s^2, Velocity (X): ");
  Serial.print(velocityX);
  Serial.print(" m/s, Position (X): ");
  Serial.print(positionX);
  Serial.println(" m");
  }

  // ======= LED LOGIC =========
  // for (uint16_t i = 0; i < ledCount; i++) {
  //   colors[i] = rgb_color(r, g, b);
  // }
  // ledStrip.write(colors, ledCount, brightness);

  // ===== AVERAGE SPEED =======
  // Update the circular buffer
  // sum -= buffer[indexX];               // Subtract the value being replace from the sum
  // buffer[indexX] = deltaCurrent;       // Store the new value in the buffer
  // sum += deltaCurrent;                 // Add the new value to the sum
  // indexX = (indexX + 1) % bufferSize;  // Increment indexX with wrap-around

  // // Check if the buffer is full
  // if (!bufferFull && indexX == 0) {
  //   bufferFull = true;
  // }

  // //Calculate the average
  // long average = bufferFull ? sum / bufferSize : sum / indexX;

  // ==== Communication logic =====
  // if (dataRequested) {
  //   dataRequested = false;

  //   imu::Vector<3> accel = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

  //   double vector[8] = { m_x, m_y, m_z, delta_x, delta_y, delta_z, accel[0] + accel[1] + accel[2], deltaCurrent };
  //   for (int i = 0; i < 8; i++) {
  //     Serial.print(vector[i]);
  //     if (i < 7) Serial.print(',');  // Separate elements with commas
  //   }
  //   Serial.println();  // End the transmission with a newline
  // }
  //}

  // ===== Serial communication with processing ========
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.charAt(0) == 'R') {
      dataRequested = true;

    } else if (input.charAt(0) == 'L') {
      input = input.substring(2);  // Skip "L,"
      int values[3];
      int index = 0;

      while (input.length() > 0 && index < 3) {
        int commaIndex = input.indexOf(',');
        if (commaIndex == -1) {
          // No more commas; take the remaining value
          values[index] = input.toInt();
          break;
        } else {
          // Extract value before the comma
          values[index] = input.substring(0, commaIndex).toInt();
          input = input.substring(commaIndex + 1);  // Remove processed value
        }
        index++;
      }

      r = values[0];
      g = values[1];
      b = values[2];

    } else if (input.charAt(0) == 'H') {
      input = input.substring(2);  // Skip "L,"
      int values[3];
      int index = 0;

      while (input.length() > 0 && index < 3) {
        int commaIndex = input.indexOf(',');
        if (commaIndex == -1) {
          // No more commas; take the remaining value
          values[index] = input.toInt();
          break;
        } else {
          // Extract value before the comma
          values[index] = input.substring(0, commaIndex).toInt();
          input = input.substring(commaIndex + 1);  // Remove processed value
        }
        index++;
      }

      // amp 0-255, freq 20 - 500, delta 10 - 250
      newAmpValue = ((float)map(values[0], 0, 255, 0, 100)) / 100;
      freqValue = values[1];           // already int
      deltaSensor = (float)values[2];  // delta is float so convert from int to float;
    }
  }


  generateTrigger();  //only can trigger if the sensor values are updated
  GenerateGrain(freqValue, &shouldVibrate, &ampValue, newAmpValue);

  //if there are multiple oscillators we also need a mixer();
  //filter();      //will need parametrs
  //compressor();  //will need parametrs
  //modulateAmplitude();
}
