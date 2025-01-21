#include <Wire.h>

#define INC_ADDRESS 0x69

#define ACC_CONF  0x20  //Page 91

#define GYR_CONF  0x21  //Page 93

#define CMD       0x7E  //Page 65

int16_t  x, y, z;
int16_t  gyro_x, gyro_y, gyro_z;
float accelX_m_s2 , accelY_m_s2, accelZ_m_s2;

void setup(void) { 
  Serial.begin(115200);
  //Accelerometer
  Wire.begin(); 
  Wire.setClock(400000);      // I2C Fast Mode (400kHz) 
  softReset(); 
  /*

Acc_Conf P.91
mode: 0x7000 -> High
average: 0x0000 -> No
filtering: 0x0080 -> ODR/4
range: 0x0000 -> 2G
ODR: 0x000B -> 800Hz
Total: 0x708B / writeRegister16(ACC_CONF,0x753D);//Setting accelerometer /
Gyr_Conf P.93
mode: 0x7000 -> High
average: 0x0000 -> No
filtering: 0x0080 -> ODR/4
range: 0x0000 -> 125kdps
ODR: 0x000B -> 800Hz
Total: 0x708B */ writeRegister16(GYR_CONF,0x758D);//Setting gyroscope
}
void softReset(){ 
  writeRegister16(CMD, 0xDEAF);
  delay(50);
}

//Write data in 16 bits
void writeRegister16(uint16_t reg, uint16_t value) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  //Low
  Wire.write((uint16_t)value & 0xff);
  //High
  Wire.write((uint16_t)value >> 8);
  Wire.endTransmission();
}

//Read data in 16 bits
uint16_t readRegister16(uint8_t reg) {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(reg);
  Wire.endTransmission(false);
  int n = Wire.requestFrom(INC_ADDRESS, 4); 
  uint16_t data[4];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  } 
  return (data[3]   | data[2] << 8);
}

//Read all axis
void readAllAccel() {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(INC_ADDRESS, 14);
  int16_t data[14];
  int i =0;
  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  }

  //Offset = 2 because the 2 first bytes are dummy (useless)
  int offset = 2; 
  x =             (data[offset + 0]        | (int16_t )data[offset + 1] << 8);  //0x03
  y =             (data[offset + 2]        | (int16_t )data[offset + 3] << 8);  //0x04
  z =             (data[offset + 4]        | (int16_t )data[offset + 5] << 8);  //0x05
  gyro_x =        (data[offset + 6]        | (int16_t )data[offset + 7] << 8);  //0x03
  gyro_y =        (data[offset + 8]        | (int16_t )data[offset + 9] << 8);  //0x04
  gyro_z =        (data[offset + 10]       | (int16_t )data[offset + 11] << 8);  //0x05
 
  accelX_m_s2 = lsbToM2S(x);
  accelY_m_s2 = lsbToM2S(y);
  accelZ_m_s2 = lsbToM2S(z);
}
float lsbToM2S(int16_t rawData) {
    // Sensitivity for the ±8g range is typically 4096 LSB/g
    const float sensitivity = 2048.0;
    // Convert g to m/s² using the standard Earth gravitational acceleration 9.81 m/s²
    const float gToM2S = 9.80665;

    // Calculate and return the acceleration in m/s²
    return (rawData / sensitivity) * gToM2S;
}

void loop() {

  if(readRegister16(0x02) == 0x00) {
    //Read ChipID
    readAllAccel();             // read all accelerometer/gyroscope/temperature data
    Serial.print("X: ");
    Serial.print(accelX_m_s2);
    Serial.print("Y: ");
    Serial.print(accelY_m_s2);
    Serial.print("Z: ");
    Serial.println(accelZ_m_s2);
  }
}