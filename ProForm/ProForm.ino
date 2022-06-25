// Reference: https://teaandtechtime.com/arduino-ble-cycling-power-service/
const int lowestPin = 2;
const int highestPin = 33;
#include <ESP32Servo.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "ProForm.h"

char str[128] = "";

int8_t set_position = 0, current_position = 0;
volatile bool reed_flag = 0, switch_flag = 0;
uint8_t motor_position = MOTOR_DEFAULT_POSITION;
uint32_t reed_debounce = 0, switch_debounce = 0;

Servo servo;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
#ifdef DEBUG
    sprintf(str, "%lu Device connected!: Core ID: %d\n", millis(), xPortGetCoreID());
    Serial.print(str);
#endif
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = true;
#ifdef DEBUG
    sprintf(str, "%lu Device disconected!: Core ID: %d\n", millis(), xPortGetCoreID());
    Serial.print(str);
#endif
  }
};

void IRAM_ATTR ISR_SWITCH()
{
  switch_flag = 1;
}

void IRAM_ATTR ISR_REED()
{
  if (digitalRead(REED_SWITCH_PIN) == HIGH)
    reed_flag = 1;
}

void setup()
{
#ifdef DEBUG
  Serial.begin(SERIAL_BAUD_RATE);
#endif
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  servo.setPeriodHertz(50);
  // Setup hardware
  servo.attach(MOTOR_PIN);

  // Switches
  pinMode(R_PLUS_PIN, INPUT_PULLUP);
  pinMode(R_MINUS_PIN, INPUT_PULLUP);
  pinMode(L_PLUS_PIN, INPUT_PULLUP);
  pinMode(L_MINUS_PIN, INPUT_PULLUP);

  // Reed switch
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);

  // Create the BLE Device
  BLEDevice::init("ProForm");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  service_dev_info = pServer->createService(SERVICE_DEVICE_INFORMATION);
  service_battery = pServer->createService(SERVICE_BATTERY);
  service_power = pServer->createService(SERVICE_CYCLING_POWER);

  // Battery characteristics
  c_SERVICE_BATTERY_LEVEL = service_battery->createCharacteristic(
      SERVICE_BATTERY_LEVEL,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  c_SERVICE_BATTERY_LEVEL->addDescriptor(new BLE2902());

  // Heart rate characteristics
  c_SERVICE_CYCLING_POWER_FEATURE = service_power->createCharacteristic(
      SERVICE_CYCLING_POWER_FEATURE,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_CYCLING_POWER_MEASUREMENT = service_power->createCharacteristic(
      SERVICE_CYCLING_POWER_MEASUREMENT,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  c_SERVICE_CYCLING_POWER_MEASUREMENT->addDescriptor(new BLE2902());

  c_SERVICE_CYCLING_POWER_SENSOR_LOCATION = service_power->createCharacteristic(
      SERVICE_CYCLING_POWER_SENSOR_LOCATION,
      BLECharacteristic::PROPERTY_READ);

  // Device information characteristics

  c_SERVICE_DEVICE_INFROMATION_SYSTEM_ID = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_SYSTEM_ID,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_DEVICE_INFROMATION_MODEL_NUMBER_STRING = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_MODEL_NUMBER_STRING,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_DEVICE_INFROMATION_SERIAL_NUMBER_STRING = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_SERIAL_NUMBER_STRING,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_DEVICE_INFROMATION_FIRMWARE_REVISION_STRING = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_FIRMWARE_REVISION_STRING,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_DEVICE_INFROMATION_HARDWARE_REVISION_STRING = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_HARDWARE_REVISION_STRING,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_DEVICE_INFROMATION_SOFTWARE_REVISION_STRING = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_SOFTWARE_REVISION_STRING,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_DEVICE_INFROMATION_MANUFACTURER_NAME_STRING = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_MANUFACTURER_NAME_STRING,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_DEVICE_INFROMATION_IEEE11073 = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_IEEE11073,
      BLECharacteristic::PROPERTY_READ);

  c_SERVICE_DEVICE_INFROMATION_PNP_ID = service_dev_info->createCharacteristic(
      SERVICE_DEVICE_INFROMATION_PNP_ID,
      BLECharacteristic::PROPERTY_READ);

  // Start the service
  service_dev_info->start();
  service_battery->start();
  service_power->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();

  pAdvertising->addServiceUUID(SERVICE_DEVICE_INFORMATION);
  pAdvertising->addServiceUUID(SERVICE_CYCLING_POWER);
  pAdvertising->addServiceUUID(SERVICE_BATTERY);

  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

#ifdef DEBUG
  Serial.println("Waiting a client connection to notify");
#endif

  // Set device properties
  c_SERVICE_DEVICE_INFROMATION_SYSTEM_ID->setValue((uint8_t *)&sys_id, 8);
  c_SERVICE_DEVICE_INFROMATION_SERIAL_NUMBER_STRING->setValue((uint8_t *)string, 16);
  c_SERVICE_DEVICE_INFROMATION_IEEE11073->setValue((uint8_t *)&iee, 8);
  c_SERVICE_DEVICE_INFROMATION_PNP_ID->setValue((uint8_t *)&pnp, 7);
  c_SERVICE_DEVICE_INFROMATION_FIRMWARE_REVISION_STRING->setValue((uint8_t *)string, 4);

  // Power meter:
  c_SERVICE_CYCLING_POWER_SENSOR_LOCATION->setValue((uint8_t *)&power_meter_location, 1);

  // Battery - tell its 100%:
  c_SERVICE_BATTERY_LEVEL->setValue((uint8_t *)&battery_level, 1);

  attachInterrupt(R_PLUS_PIN, ISR_SWITCH, FALLING);
  attachInterrupt(L_PLUS_PIN, ISR_SWITCH, FALLING);

  attachInterrupt(R_MINUS_PIN, ISR_SWITCH, FALLING);
  attachInterrupt(L_MINUS_PIN, ISR_SWITCH, FALLING);

  attachInterrupt(REED_SWITCH_PIN, ISR_REED, RISING);

  servo.write(motor_position);
#ifdef DEBUG
  sprintf(str, "Setup completed Core ID: %d   %u\n", xPortGetCoreID(), millis());
  Serial.print(str);
#endif
}

void loop()
{
  if (switch_flag && (millis() - switch_debounce) > SWITCH_DEBOUNCE_TIME)
  {
    if (digitalRead(R_PLUS_PIN) == LOW)
      motor_position += 5;
    else if (digitalRead(L_PLUS_PIN) == LOW)
      motor_position += 5;
    else if (digitalRead(R_MINUS_PIN) == LOW)
      motor_position -= 5;
    else if (digitalRead(L_MINUS_PIN) == LOW)
      motor_position -= 5;

    if (motor_position < MOTOR_MIN_POSITION)
      motor_position = MOTOR_MIN_POSITION;

    if (motor_position > MOTOR_MAX_POSITION)
      motor_position = MOTOR_MAX_POSITION;

    switch_debounce = millis();
    switch_flag = 0;
#ifdef DEBUG
    sprintf(str, "New position: %d\n", motor_position);
    Serial.print(str);
#endif
    servo.write(motor_position);
  }

  if (reed_flag)
  {
#ifdef DEBUG
    static uint16_t cnt = 0;
    sprintf(str, "Reed! Device connected: %s , Timestamp: %u CNT: %d\n", deviceConnected ? "Yes" : "No", reed_debounce, ++cnt);
    Serial.print(str);
#endif
    if (deviceConnected && (millis() - reed_debounce) >= REED_DEBOUNCE_TIME)
    {
      static uint16_t time = (millis() - reed_debounce);
      float cadence = 60000.f / (float)time;

      power = Aconst * (float)motor_position * cadence + Bconst;

      timestamp += (uint16_t)time;
      revolutions += 1;

      bleBuffer[0] = flags & 0xff;
      bleBuffer[1] = (flags >> 8) & 0xff;
      bleBuffer[2] = power & 0xff;
      bleBuffer[3] = (power >> 8) & 0xff;
      bleBuffer[4] = revolutions & 0xff;
      bleBuffer[5] = (revolutions >> 8) & 0xff;
      bleBuffer[6] = timestamp & 0xff;
      bleBuffer[7] = (timestamp >> 8) & 0xff;

      fBuffer[0] = 0x00;
      fBuffer[1] = 0x00;
      fBuffer[2] = 0x00;
      fBuffer[3] = 0x08;

      c_SERVICE_CYCLING_POWER_FEATURE->setValue((uint8_t *)fBuffer, 4);
      c_SERVICE_CYCLING_POWER_MEASUREMENT->setValue((uint8_t *)bleBuffer, 8);
      c_SERVICE_CYCLING_POWER_MEASUREMENT->notify();
    }
    reed_debounce = millis();
    reed_flag = 0;
  }

  // Bluetooth
  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    vTaskDelay(500);             // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    oldDeviceConnected = deviceConnected;
#ifdef DEBUG
    Serial.println("BT: Start advertising");
#endif
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    oldDeviceConnected = deviceConnected;
#ifdef DEBUG
    Serial.println("BT: Start notyfing");
#endif
  }
}
