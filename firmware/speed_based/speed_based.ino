#include <DaisyDuino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <elapsedMillis.h>

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

float deltaSensor = 5.0;
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
Adafruit_BNO055 myIMU = Adafruit_BNO055();
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
  myIMU.begin();
  delay(100);
  //myIMU.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P8);
  //myIMU.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P2);
  myIMU.setExtCrystalUse(true);

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
}

void loop() {
  // ===== Read data from IMU =======


  if (timer_imu_data >= sampleInterval) {
    timer_imu_data = 0;  // Update the last sample time

    imu::Vector<3> linearAccel = myIMU.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    // Get accelarion values
    accelerationX = linearAccel.x();
    m_y = linearAccel.y();
    m_z = linearAccel.z();

    unsigned long currentTime = millis();
    float deltaTime = (currentTime - prevTime) / 1000.0;  // Convert ms to seconds
    prevTime = currentTime;

    // delta_x = fabs(accelerationX - prev_mx) * 10.0;
    // delta_y = fabs(m_y - prev_my) * 10.0;
    // delta_z = fabs(m_z - prev_mz) * 10.0;

    // velocityX += accelerationX * deltaTime;
    // velocityY += m_y * deltaTime;
    // velocityZ += m_z * deltaTime;

    velocityX = prev_velocityX + accelerationX * deltaTime;


    // positionX += velocityX * deltaTime;
    // positionY += velocityY * deltaTime;
    // positionZ += velocityZ * deltaTime;

    positionX = prev_positionX + velocityX * deltaTime;

    delta_x = fabs(velocityX - prev_velocityX) * 100;
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
  }

  // ===== Serial communication with processing ========
  generateTrigger();  //only can trigger if the sensor values are updated
  GenerateGrain(freqValue, &shouldVibrate, &ampValue, newAmpValue);
}