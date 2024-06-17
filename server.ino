#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "BLE2902.h"

// GPIO pin to read from and write to
const int GPIO_PIN = 10;

class SimpleBLEServer : public BLEServerCallbacks {
private:
  BLEServer* pServer;
  BLEService* pService;
  BLECharacteristic* pCharacteristic;
  BLEAdvertising* pAdvertising;
  std::string deviceName;
  std::string deviceManufacturer;
  bool connected = false;
  BLEAddress connectedAddress;

public:
  SimpleBLEServer(std::string name = "ESP32C3 Super Mini", std::string manufacturer = "Babyyoda777")
      : deviceName(name), deviceManufacturer(manufacturer), connectedAddress(BLEAddress((uint8_t*)"\0\0\0\0\0\0")) {}

  void begin() {
    BLEDevice::init(deviceName);
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(this);

    pService = pServer->createService(BLEUUID((uint16_t)0x181A));  // HID Service UUID
    pCharacteristic = pService->createCharacteristic(
        BLEUUID((uint16_t)0x2A4D),  // Report Characteristic UUID
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_NOTIFY
    );

    pCharacteristic->addDescriptor(new BLE2902());
    pService->start();

    pAdvertising = pServer->getAdvertising();
    pAdvertising->addServiceUUID(pService->getUUID());
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    Serial.println("Advertising started!");

    pinMode(GPIO_PIN, INPUT); // Set GPIO pin 10 as input initially
  }

  void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) override {
    connected = true;
    connectedAddress = BLEAddress(param->connect.remote_bda);
    Serial.println("Client connected");
    Serial.print("Connected device address: ");
    Serial.println(connectedAddress.toString().c_str());
  }

  void onDisconnect(BLEServer* pServer) override {
    connected = false;
    Serial.println("Client disconnected");
    pAdvertising->start();  // Restart advertising
  }

  bool isConnected() {
    return connected;
  }

  // Function to update characteristic with GPIO pin state
  void updateCharacteristic() {
    if (connected) {
      // Read GPIO pin value
      int gpioValue = digitalRead(GPIO_PIN);

      // Update characteristic with the GPIO pin value
      pCharacteristic->setValue((uint8_t*)&gpioValue, sizeof(gpioValue));
      pCharacteristic->notify();
    }
  }

  // Public method to get characteristic by UUID
  BLECharacteristic* getCharacteristicByUUID(uint16_t uuid) {
    return pService->getCharacteristic(BLEUUID(uuid));
  }

  // Function to handle writing to GPIO pin from Bluetooth
  void writeCharacteristic(BLECharacteristic *pCharacteristic) {
    std::string value = pCharacteristic->getValue();

    if (value.length() == sizeof(int)) {
      int newValue;
      memcpy(&newValue, value.data(), sizeof(int));

      // Write newValue to GPIO pin
      digitalWrite(GPIO_PIN, newValue);
    }
  }
};

SimpleBLEServer myBLEServer;

void setup() {
  Serial.begin(115200);
  myBLEServer.begin();
}

void loop() {
  myBLEServer.updateCharacteristic();

  // Handle Bluetooth characteristic write
  BLECharacteristic *pCharacteristic = myBLEServer.getCharacteristicByUUID((uint16_t)0x2A4D);
  if (pCharacteristic && pCharacteristic->getValue().length() > 0) {
    myBLEServer.writeCharacteristic(pCharacteristic);
  }

  delay(1000);  // Update characteristic and handle Bluetooth interaction every second
}
