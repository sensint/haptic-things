#include <DaisyDuino.h>
#include <Wire.h>
#include <elapsedMillis.h>

#include <Adafruit_BNO08x.h>

//          /----------------------------------> Calculate Speed ----------------------------------\                                                        
// Sensor -/---> Calculate Position \---> Generate Triggers --> Generate Grain ---/---> Filter --> Compressor --> Amplitude Modulation
//                     (optional)    \--> Generate Triggers --> Generate Grain --/

// New IMU
#define BNO08X_RESET -1
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

float deltaSensor = 10.0;
long deltaCurrent = 0.0;
long cumulativeDelta = 0;

float ampValue;
float newAmpValue = 0.7;
int freqValue = 60;
uint32_t signalDuration;

const float amplitudeValues[] = { 0.9, 0.8, 0.5 };
const float frequencyValues[] = { 50.0, 40.0, 60.0 };
const float deltaValues[] = { 40.0, 20.0, 10.0 };

// ======= IMU ========

const int sampleRateHz = 100;
unsigned long sampleInterval = 1000 / sampleRateHz;  // Convert Hz to milliseconds
unsigned long lastSampleTime = 0;                    // Time of the last sample
elapsedMillis timer_imu_data = 0;
double accelerationX, m_y, m_z = 0.0;
double prev_mx, prev_my, prev_mz = 0.0;
double delta_x, delta_y, delta_z = 0.0;

double velocityX = 0;
double positionX = 0;
double prev_positionX = 0;
double prev_velocityX = 0;
double velocityY = 0;
double positionY = 0;
double velocityZ = 0;
double positionZ = 0;
unsigned long prevTime = 0;

float filteredAccelerationX = 0.0;  // Filtered acceleration
float alpha = 0.1;                  // Smoothing factor for the filter (adjust as needed)

uint16_t BNO055_SAMPLERATE_DELAY_MS = 2.5;  //how often to read data from the board
uint16_t PRINT_DELAY_MS = 500;              // how often to print the data
uint16_t printCount = 0;                    //counter to avoid printing every 10MS sample

//velocity = accel*dt (dt in seconds)
//position = 0.5*accel*dt^2
double ACCEL_VEL_TRANSITION = (double)(BNO055_SAMPLERATE_DELAY_MS) / 1000.0;
double ACCEL_POS_TRANSITION = 0.5 * ACCEL_VEL_TRANSITION * ACCEL_VEL_TRANSITION;


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

// this function triggers a grain
void generateTrigger() {
  if (cumulativeDelta >= deltaSensor) {  //deltasensor should be a parameter, so we can modulate it
    shouldVibrate = true;
    cumulativeDelta = 0.0;

  } else {
    shouldVibrate = false;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens


  // IMU init
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

  prevTime = millis();  // Initialize time

  setReports();
}

void setReports(void) {

  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    Serial.println("Could not enable linear acceleration");
  }
}

void loop() {
  // ===== Read data from IMU =======


  if (bno08x.wasReset()) {
    Serial.print("sensor was reset ");
    setReports();
  }

  if (!bno08x.getSensorEvent(&sensorValue)) {
    return;
  }

  switch (sensorValue.sensorId) {

    case SH2_LINEAR_ACCELERATION:
      accelerationX = sensorValue.un.linearAcceleration.x;
      m_y = sensorValue.un.linearAcceleration.y;
      m_z = sensorValue.un.linearAcceleration.z;
      break;
  }

  filteredAccelerationX = alpha * accelerationX + (1 - alpha) * filteredAccelerationX;


  unsigned long currentTime = millis();
  float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert ms to seconds
  prevTime = currentTime;

  // delta_x = fabs(accelerationX - prev_mx) * 10.0;
  // delta_y = fabs(m_y - prev_my) * 10.0;
  // delta_z = fabs(m_z - prev_mz) * 10.0;

  velocityX += accelerationX * deltaTime;
  // velocityY += m_y * deltaTime;
  // velocityZ += m_z * deltaTime;

  //velocityX = prev_velocityX + accelerationX * deltaTime;


  //positionX += velocityX * deltaTime;
  // positionY += velocityY * deltaTime;
  // positionZ += velocityZ * deltaTime;

  //positionX = prev_positionX + velocityX * deltaTime;
  positionX = positionX * deltaTime * deltaTime * accelerationX;
  delta_x = fabs(velocityX - prev_velocityX) * 100;

  //delta_x = fabs(positionX - prev_positionX) * 100;

  deltaCurrent = delta_x;
  cumulativeDelta += deltaCurrent;

  prev_positionX = positionX;
  prev_velocityX = velocityX;

  // Print results
  Serial.print(20);
  Serial.print(",");
  Serial.print(accelerationX);
  Serial.print(",");
  Serial.print(velocityX);
  Serial.print(",");
  Serial.println(positionX);



  // ===== Serial communication with processing ========
  //generateTrigger();  //only can trigger if the sensor values are updated
  //GenerateGrain(freqValue, &shouldVibrate, &ampValue, newAmpValue);
}