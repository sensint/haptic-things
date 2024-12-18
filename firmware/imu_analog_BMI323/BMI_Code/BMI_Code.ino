#include <Wire.h>

#define INC_ADDRESS 0x69
#define ACC_CONF  0x20  //Page 91
#define GYR_CONF  0x21  //Page 93
#define CMD       0x7E  //Page 65

int16_t  x, y, z;
int16_t  gyro_x, gyro_y, gyro_z;


void setup(void) {
  Serial.begin(115200);
  //Accelerometer
  Wire.begin();
  Wire.setClock(400000);      // I2C Fast Mode (400kHz)
  softReset();
  /*
     Acc_Conf P.91
     mode:        0x7000  -> High
     average:     0x0000  -> No
     filtering:   0x0080  -> ODR/4
     range:       0x0000  -> 2G
     ODR:         0x000B  -> 800Hz
     Total:       0x708B
  */
  writeRegister16(ACC_CONF, 0x708E); //Setting accelerometer
  /*
     Gyr_Conf P.93
     mode:        0x7000  -> High
     average:     0x0000  -> No
     filtering:   0x0080  -> ODR/4
     range:       0x0000  -> 125kdps
     ODR:         0x000B  -> 800Hz
     Total:       0x708B
  */
  writeRegister16(GYR_CONF, 0x708E); //Setting gyroscope
}


void softReset() {
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
  int i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }
  return (data[3] | data[2] << 8);
}


//Read all axis
void readAllAccel() {
  Wire.beginTransmission(INC_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();
  Wire.requestFrom(INC_ADDRESS, 14);
  int16_t data[14];
  int i = 0;
  while (Wire.available()) {
    data[i] = Wire.read();
    i++;
  }

  //Offset = 2 because the 2 first bytes are dummy (useless)
  static int offset = 2;
  x = (data[offset + 0]        | (int16_t )data[offset +  1] << 8);  //0x03
  y = (data[offset + 2]        | (int16_t )data[offset +  3] << 8);  //0x04
  z = (data[offset + 4]        | (int16_t )data[offset +  5] << 8);  //0x05
  gyro_x = (data[offset + 6]   | (int16_t )data[offset +  7] << 8);  //0x03
  gyro_y = (data[offset + 8]   | (int16_t )data[offset +  9] << 8);  //0x04
  gyro_z = (data[offset + 10]  | (int16_t )data[offset + 11] << 8);  //0x05
}


void loop() {
  static int i = 0;
  static int available_count = 0;
  static int not_available_count = 0;
  if (i < 10) {
    unsigned long t1 = millis();
    for (int j = 0; j < 1000; j++) {
      if (readRegister16(0x02) == 0x00) {
        readAllAccel();
        available_count++;
      } else {
        not_available_count++;
      }
    }
    unsigned long t2 = millis();
    Serial.printf("Elapsed ms: %i  - attempts: %i, available: %i, not available: %i", (int)t2-t1, available_count, not_available_count);
//    Serial.print("Elapsed ms:");
//    Serial.println(t2-t1);
//    Serial.print("Available:");
//    Serial.println(available_count);
//    Serial.print("Not Available:");
//    Serial.println(not_available_count);
    i++;
    available_count = 0;
    not_available_count = 0;
  }
}
