#include <DaisyDuino.h>
#include <Wire.h>
#include <elapsedMillis.h>
#include <Adafruit_BNO08x.h>
#include <APA102.h>
#include "LSM6DSOXSensor.h"

// ======= Fast IMU SetUp ========
LSM6DSOXSensor lsm6dsoxSensor = LSM6DSOXSensor(&Wire, LSM6DSOX_I2C_ADD_L);
int32_t _sensor_tilt_x, sensor_tilt_x, prev_sensor_tilt_x = 0;
double delta_sensor_tilt_x = 0.0;
double filteredAccelerationX = 0.0;  // Filtered acceleration
float alpha = 0.3;                   // Smoothing factor for the filter (adjust as needed)

// ====== Bluetooth control values ===========
bool mode_fsr_activated = true;
bool mode_imu_activated = false;
bool mode_imu_activated_x, mode_imu_activated_y, mode_imu_activated_z = false;
bool mode_tilt_activated = false;
int bt_parameter_frecuency = 9;
int bt_parameter_delta_sensor = 40;

// ============ Audio/Haptics Flow signal variables ============
elapsedMicros pulseTimeUS = 0;
bool isVibrating = false;
bool shouldVibrate = false;

float deltaSensor = 50.0;  // for normail imu = 100, for fsr = 70
float default_deltaSensor = 100.0;
long deltaCurrent = 0.0;
long cumulativeDelta = 0;

float ampValue;
float newAmpValue = 0.4;
int freqValue = 100;
uint32_t signalDuration;

// ====== LED ======
const uint8_t dataPin = 0;
const uint8_t clockPin = 1;
APA102<dataPin, clockPin> ledStrip;
// Set the number of LEDs to control.
const uint16_t ledCount = 1;
rgb_color colors[ledCount];
const uint8_t brightness = 1;

// ========================= BNO085 ==========================================
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
  //Serial.println("Setting desired reports");
  if (!bno08x.enableReport(reportType, report_interval)) {
    //Serial.println("Could not enable stabilized remote vector");
  }
}

// ================ DaisyVariables ===============
DaisyHardware hw;
Svf filterLow;
Svf filterHigh;
size_t num_channels;
static Oscillator osc;

// ============ IMU ==============
struct euler_t {
  float yaw;
  float pitch;
  float roll;
} ypr;

const int sampleRateHz = 100;
unsigned long sampleInterval = 1000 / sampleRateHz;  // Convert Hz to milliseconds
unsigned long lastSampleTime = 0;                    // Time of the last sample
elapsedMillis timer_imu_data = 0;

double sensor_imu_x, sensor_imu_y, sensor_imu_z, sensor_pressure = 0.0;
double prev_sensor_imu_x, prev_sensor_imu_y, prev_sensor_imu_z, prev_sensor_pressure = 0.0;

double delta_sensor_imu_x, delta_sensor_imu_y, delta_sensor_imu_z, delta_sensor_pressure = 0.0;

unsigned long prevTime = 0;

// ===== Filter some signal? ======


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

// this function triggers a grain
void generateTrigger() {
  if (cumulativeDelta >= deltaSensor) {  //deltasensor should be a parameter, so we can modulate it
    shouldVibrate = true;
    cumulativeDelta = 0.0;
    //Serial.println("--- NEW GRAIN ----- ");
  } else {
    shouldVibrate = false;
  }
}

void setReports(void) {
  if (!bno08x.enableReport(SH2_LINEAR_ACCELERATION)) {
    //Serial.println("Could not enable linear acceleration");
  }
}
//////////////////////////////////////////
// END - FUNCTIONS FOR GRAIN FEEDBACK   //
//////////////////////////////////////////

void setup() {
  // ====== Setting communication with XIAO! ======
  Serial1.begin(115200);
  // while (!Serial1) delay(10);  // will pause Zero, Leonardo, etc until serial console opens
 //Serial.begin(9600);
  //Serial.println("HOLA");

  // ========== Fast IMU =========
  Wire.begin();
  Wire.setClock(400000);
  lsm6dsoxSensor.begin();

  // Enable accelerometer and gyroscope, and check success
  if (lsm6dsoxSensor.Enable_X() == LSM6DSOX_OK && lsm6dsoxSensor.Enable_G() == LSM6DSOX_OK) {
    //Serial.println("Success enabling accelero and gyro");
  } else {
    //Serial.println("Error enabling accelero and gyro");
  }

  // Read ID of device and check that it is correct
  uint8_t id;
  lsm6dsoxSensor.ReadID(&id);
  if (id != LSM6DSOX_ID) {
    //Serial.println("Wrong ID for LSM6DSOX sensor. Check that device is plugged");
  } else {
    //Serial.println("Receviced correct ID for LSM6DSOX sensor");
  }

  // Set accelerometer scale at +- 2G. Available values are +- 2, 4, 8, 16 G
  lsm6dsoxSensor.Set_X_FS(2);
  // Set gyroscope scale at +- 125 degres per second. Available values are +- 125, 250, 500, 1000, 2000 dps
  lsm6dsoxSensor.Set_G_FS(125);
  // Set Accelerometer sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_X_ODR(6667.0f);
  // Set Gyroscope sample rate to 208 Hz. Available values are +- 12.0, 26.0, 52.0, 104.0, 208.0, 416.0, 833.0, 1667.0, 3333.0, 6667.0 Hz
  lsm6dsoxSensor.Set_G_ODR(208.0f);

  // ======= IMU init =======
  if (!bno08x.begin_I2C()) {
    //Serial.println("Failed to find BNO08x chip");
    while (1) { delay(10); }
  }
  setReports(reportType, reportIntervalUs);
  delay(100);

  // ======= Initialize for Daisy pod at 48kHz =======
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

  prevTime = millis();  // Initialize time
  setReports();

  // ======= LED LOGIC =========
  //   for (uint16_t i = 0; i < ledCount; i++) {
  //     colors[i] = rgb_color(0, 0, 255);
  //   }
  //   ledStrip.write(colors, ledCount, brightness);
}

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

void quaternionToEulerGI(sh2_GyroIntegratedRV_t* rotational_vector, euler_t* ypr, bool degrees = false) {
  quaternionToEuler(rotational_vector->real, rotational_vector->i, rotational_vector->j, rotational_vector->k, ypr, degrees);
}

void loop() {


  // ========= READ BLUETOOTH DATA ==========
  static String receivedData = "";

  while (Serial1.available() > 0) {
    String receivedData = Serial1.readStringUntil(';');
    receivedData.remove(receivedData.length() - 1);

    int btvalues[7];  // Array to store the parsed integers
    int index = 0;

    while (receivedData.length() > 0) {
      int commaIndex = receivedData.indexOf(',');  // Find the next comma

      if (commaIndex == -1) {  // No more commas, get the last value
        btvalues[index++] = receivedData.toInt();
        break;
      }

      // Extract the number before the comma
      String numStr = receivedData.substring(0, commaIndex);
      btvalues[index++] = numStr.toInt();

      // Remove the parsed number and comma
      receivedData = receivedData.substring(commaIndex + 1);
    }

    // Assign the values
    bt_parameter_delta_sensor = btvalues[0];
    freqValue = btvalues[1];
    mode_fsr_activated = btvalues[2];
    mode_imu_activated_x = btvalues[3];
    mode_imu_activated_y = btvalues[4];
    mode_imu_activated_z = btvalues[5];
    mode_tilt_activated = btvalues[6];
    bt_parameter_frecuency = btvalues[7];

    if (mode_imu_activated_x || mode_imu_activated_y || mode_imu_activated_z) {
      mode_imu_activated = 1;
    } else {
      mode_imu_activated = 0;
    }

    deltaSensor = float(bt_parameter_delta_sensor);

    lsm6dsoxSensor.Set_X_ODR(float(bt_parameter_frecuency));
  }

  if (mode_fsr_activated) {

    // ======= LED LOGIC =========
    for (uint16_t i = 0; i < ledCount; i++) {
      colors[i] = rgb_color(255, 0, 0);
    }
    ledStrip.write(colors, ledCount, brightness);

    // ======= READ FSR VALUES =======
    int total = 0;
    int numPins = 6;
    float average = 0.0;
    for (int pin = A0; pin <= A5; pin++) {
      total += analogRead(pin);
    }
    average = ((total / (float)numPins));
    sensor_pressure = average;
    //Serial.println(sensor_pressure);
    //  if (sensor_pressure > 20) {
    delta_sensor_pressure = abs(sensor_pressure - prev_sensor_pressure);
    deltaCurrent = delta_sensor_pressure;
    // }

    delay(10);
  }

  if (mode_imu_activated) {
    // ======= LED LOGIC =========
    for (uint16_t i = 0; i < ledCount; i++) {
      colors[i] = rgb_color(0, 0, 255);
    }
    ledStrip.write(colors, ledCount, brightness);

    // ============= READ IMU DATA =============
    if (bno08x.wasReset()) {
      //Serial.print("sensor was reset ");
      setReports();
    }

    if (bno08x.getSensorEvent(&sensorValue)) {
      //Serial.println("imu algo");
      switch (sensorValue.sensorId) {
        case SH2_ARVR_STABILIZED_RV:
          quaternionToEulerRV(&sensorValue.un.arvrStabilizedRV, &ypr, true);
        case SH2_GYRO_INTEGRATED_RV:
          // faster (more noise?)
          quaternionToEulerGI(&sensorValue.un.gyroIntegratedRV, &ypr, true);
          break;
      }

      sensor_imu_x = ypr.yaw;
      sensor_imu_y = ypr.pitch;
      sensor_imu_z = ypr.roll;

      //Serial.println(sensor_imu_x);
    }

    // ======== GRAIN LOGIC ===========
    delta_sensor_imu_x = fabs(sensor_imu_x - prev_sensor_imu_x) * 10.0;
    delta_sensor_imu_y = fabs(sensor_imu_y - prev_sensor_imu_y) * 10.0;
    delta_sensor_imu_z = fabs(sensor_imu_z - prev_sensor_imu_z) * 10.0;


    // deltaCurrent = delta_sensor_imu_x + delta_sensor_imu_y + delta_sensor_imu_z;
    deltaCurrent = 0;
    if (mode_imu_activated_x) {
      deltaCurrent += delta_sensor_imu_x;
      //Serial.println("x");
    }
    if (mode_imu_activated_y) {
      deltaCurrent += delta_sensor_imu_y;
      //Serial.println("y");
    }
    if (mode_imu_activated_z) {
      deltaCurrent += delta_sensor_imu_z;
      //Serial.println("z");
    }
  }

  if (mode_tilt_activated) {
    // ======= LED LOGIC =========
    for (uint16_t i = 0; i < ledCount; i++) {
      colors[i] = rgb_color(0, 255, 0);
    }
    ledStrip.write(colors, ledCount, brightness);

    // Read accelerometer
    uint8_t acceleroStatus;
    lsm6dsoxSensor.Get_X_DRDY_Status(&acceleroStatus);
    if (acceleroStatus == 1) {  // Status == 1 means a new data is available
      int32_t acceleration[3];
      lsm6dsoxSensor.Get_X_Axes(acceleration);
      // Plot data for each axis in mg
      _sensor_tilt_x = acceleration[0];
      filteredAccelerationX = alpha * _sensor_tilt_x + (1 - alpha) * filteredAccelerationX;
      sensor_tilt_x = filteredAccelerationX;
      if (sensor_tilt_x < 50 && sensor_tilt_x > -50) {
        sensor_tilt_x = 0;
      }
      //Serial.println(sensor_tilt_x);
    }

    //Serial.println(sensor_tilt_x);

    delta_sensor_tilt_x = fabs(sensor_tilt_x - prev_sensor_tilt_x);
    deltaCurrent = delta_sensor_tilt_x;

    delay(10);
  }

  cumulativeDelta += deltaCurrent;
  //Serial.println(cumulativeDelta);
  // ========= SET PREVIOUS VALUES ==========
  prev_sensor_imu_x = sensor_imu_x;
  prev_sensor_imu_y = sensor_imu_y;
  prev_sensor_imu_z = sensor_imu_z;
  prev_sensor_pressure = sensor_pressure;
  prev_sensor_tilt_x = sensor_tilt_x;

  generateTrigger();  //only can trigger if the sensor values are updated
  GenerateGrain(freqValue, &shouldVibrate, &ampValue, newAmpValue);
}