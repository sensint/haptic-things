#include <ArduinoBLE.h>

BLEService customService("19B10000-0001-537E-4F6C-D104768A1214");  // Replace with your service UUID
// BLEByteCharacteristic customCharacteristic("19B10001-0001-537E-4F6C-D104768A1214", BLERead | BLEWrite);
BLEStringCharacteristic customCharacteristic("19B10001-0001-537E-4F6C-D104768A1214", BLERead | BLEWrite, 512);

void setup() {
  if (!BLE.begin()) {
    while (1)
      ;
  }

  BLE.setLocalName("Haptic-Thing-2");
  BLE.setAdvertisedService(customService);
  customService.addCharacteristic(customCharacteristic);
  BLE.addService(customService);

  customCharacteristic.writeValue("hello");

  BLE.advertise();

  Serial1.begin(115200);
}

void loop() {
  BLEDevice central = BLE.central();

  if (central) {

    while (central.connected()) {
      if (customCharacteristic.written()) {
        // Use a buffer to read and convert the received data
        String receivedData = customCharacteristic.value();

        // Parse the string
        String dataString = String(receivedData);  // Convert to String for easier parsing
        Serial1.print(dataString);
      }
    }
  }
}

