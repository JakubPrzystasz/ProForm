//Reference: https://teaandtechtime.com/arduino-ble-cycling-power-service/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BTS7960.h"
#include "power.h"

//Hardware
BTS7960 motorController(BTS_EN_PIN, BTS_L_PWM_PIN, BTS_R_PWM_PIN);

//Software
static SemaphoreHandle_t bt_mutex;

TaskHandle_t bluetooth_task;

bool deviceConnected = false, oldDeviceConnected = false;

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];

short power = 0;
unsigned short revolutions = 0;
unsigned short timestamp = 0;
unsigned short flags = 0x20;

BLEServer *pServer = NULL;
BLEService *service_dev_info = NULL, *service_battery = NULL, *service_power = NULL;

BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_SYSTEM_ID = NULL;
BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_MODEL_NUMBER_STRING = NULL;
BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_SERIAL_NUMBER_STRING = NULL;
BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_FIRMWARE_REVISION_STRING = NULL;
BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_HARDWARE_REVISION_STRING = NULL;
BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_SOFTWARE_REVISION_STRING = NULL;
BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_MANUFACTURER_NAME_STRING = NULL;
BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_IEEE11073 = NULL;
BLECharacteristic *c_SERVICE_DEVICE_INFROMATION_PNP_ID = NULL;

BLECharacteristic *c_SERVICE_BATTERY_LEVEL = NULL;

BLECharacteristic *c_SERVICE_CYCLING_POWER_FEATURE = NULL;
BLECharacteristic *c_SERVICE_CYCLING_POWER_MEASUREMENT = NULL;
BLECharacteristic *c_SERVICE_CYCLING_POWER_SENSOR_LOCATION = NULL;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
#ifdef DEBUG
    char str[45];
    sprintf(str, "%lu Device connected!: Core ID: %d\n", millis(), xPortGetCoreID());
    Serial.print(str);
#endif
  };

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;
#ifdef DEBUG
    char str[45];
    sprintf(str, "%lu Device disconected!: Core ID: %d\n", millis(), xPortGetCoreID());
    Serial.print(str);
#endif
  }
};

void bluetooth_task_function(void *params)
{
  for (;;)
  {
    if (deviceConnected)
    {
#ifdef DEBUG
      char str[40];
      sprintf(str, "%lu BT: Notifying values\n", millis());
      Serial.print(str);
#endif
      timestamp += (unsigned short)(10 * (1024 / 16));
      revolutions += 1;

      //pack data, and send them into device
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
    vTaskDelay(500);
  }
}

void setup()
{

  //Setup hardware
  //Potentiometer
  pinMode(POT_PIN, INPUT);

  //Switches
  pinMode(R_PLUS_PIN, INPUT_PULLUP);
  pinMode(R_MINUS_PIN, INPUT_PULLUP);
  pinMode(L_PLUS_PIN, INPUT_PULLUP);
  pinMode(L_MINUS_PIN, INPUT_PULLUP);

  //Reed switch
  pinMode(REED_SWITCH_PIN, INPUT_PULLUP);

#ifdef DEBUG
  Serial.begin(115200);
#endif

  // Create the BLE Device
  BLEDevice::init("ProForm");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  service_dev_info = pServer->createService(SERVICE_DEVICE_INFORMATION);
  service_battery = pServer->createService(SERVICE_BATTERY);
  service_power = pServer->createService(SERVICE_CYCLING_POWER);

  //Battery characteristics
  c_SERVICE_BATTERY_LEVEL = service_battery->createCharacteristic(
      SERVICE_BATTERY_LEVEL,
      BLECharacteristic::PROPERTY_READ |
          BLECharacteristic::PROPERTY_NOTIFY);
  c_SERVICE_BATTERY_LEVEL->addDescriptor(new BLE2902());

  //Heart rate characteristics
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

  //Device information characteristics

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

  //Start the service
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

  //Set device properties
  c_SERVICE_DEVICE_INFROMATION_SYSTEM_ID->setValue((uint8_t *)&sys_id, 8);
  c_SERVICE_DEVICE_INFROMATION_SERIAL_NUMBER_STRING->setValue((uint8_t *)string, 16);
  c_SERVICE_DEVICE_INFROMATION_IEEE11073->setValue((uint8_t *)&iee, 8);
  c_SERVICE_DEVICE_INFROMATION_PNP_ID->setValue((uint8_t *)&pnp, 7);
  c_SERVICE_DEVICE_INFROMATION_FIRMWARE_REVISION_STRING->setValue((uint8_t *)string, 4);

  //Power meter:
  c_SERVICE_CYCLING_POWER_SENSOR_LOCATION->setValue((uint8_t *)power_meter_location, 1);

  //Battery:
  c_SERVICE_BATTERY_LEVEL->setValue((uint8_t *)battery_level, 1);

#ifdef DEBUG
  Serial.println("Starting tasks.");
#endif

  //Setup tasks
  xTaskCreatePinnedToCore(
      bluetooth_task_function, /* Task function. */
      "bt_task",               /* name of task. */
      10000,                   /* Stack size of task */
      NULL,                    /* parameter of the task */
      1,                       /* priority of the task */
      &bluetooth_task,         /* Task handle to keep track of created task */
      0);                      /* pin task to core 0 */
  delay(500);
}

void loop()
{
#ifdef DEBUG
  char str[25];
  sprintf(str, "%lu Core ID: %d\n", millis(), xPortGetCoreID());
  Serial.print(str);
  vTaskDelay(10000);
#endif
  //Examine all inputs

  int value = READ_POT;
  Serial.println(value);
  delay(150);
}
