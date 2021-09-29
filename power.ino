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
static SemaphoreHandle_t switch_mutex;

TaskHandle_t switch_task_t;
TaskHandle_t motor_task_t;

uint8_t motor_position = BTS_DEFAULT;
uint32_t reed_time = 0;

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

void switch_task(void *param)
{
  uint32_t sw_time[4] = {0, 0, 0, 0};

  for (;;)
  {

    if ((digitalRead(R_PLUS_PIN) == LOW && xTaskGetTickCount() - sw_time[0] >= 250) ||
        (digitalRead(L_PLUS_PIN) == LOW && xTaskGetTickCount() - sw_time[1] >= 250))
    {
      xSemaphoreTake(switch_mutex, portMAX_DELAY);
      if (motor_position + 5 <= BTS_MAX_VALUE)
      {
        motor_position += 5;
      }
      xSemaphoreGive(switch_mutex);
      sw_time[0] = xTaskGetTickCount();
      sw_time[1] = xTaskGetTickCount();
    }

    if ((digitalRead(R_MINUS_PIN) == LOW && xTaskGetTickCount() - sw_time[2] >= 250) ||
        (digitalRead(L_MINUS_PIN) == LOW && xTaskGetTickCount() - sw_time[3] >= 250))
    {
      xSemaphoreTake(switch_mutex, portMAX_DELAY);
      if (motor_position - 5 >= BTS_MIN_VALUE)
      {
        motor_position -= 5;
      }
      xSemaphoreGive(switch_mutex);
      sw_time[2] = xTaskGetTickCount();
      sw_time[3] = xTaskGetTickCount();
    }
  }
}

void motor_task(void *param)
{
  uint32_t last_time = 0;
  uint32_t current_position = 0;
  uint32_t set_position = 0;
  for (;;)
  {
    xSemaphoreTake(switch_mutex, portMAX_DELAY);
    set_position = motor_position;
    xSemaphoreGive(switch_mutex);

    //Read Position
    current_position = map(analogRead(POT_PIN), 0, 4095, 0, 100);

    if (current_position + 1 < set_position)
    {
      motorController.Enable();

      motorController.TurnRight(255);
      vTaskDelay(50 / portTICK_PERIOD_MS);

      motorController.Stop();
      motorController.Disable();
    }

    if (current_position - 1 > set_position)
    {
      motorController.Enable();

      motorController.TurnRight(255);
      vTaskDelay(50 / portTICK_PERIOD_MS);

      motorController.Stop();
      motorController.Disable();
    }

    vTaskDelay(25 / portTICK_PERIOD_MS);
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
  switch_mutex = xSemaphoreCreateMutex();

  xTaskCreate(
      switch_task,      // Function that should be called
      "Switch_Task",    // Name of the task (for debugging)
      1000,             // Stack size (bytes)
      NULL,             // Parameter to pass
      tskIDLE_PRIORITY, // Task priority
      NULL              // Task handle
  );

  xTaskCreate(
      motor_task,       // Function that should be called
      "Motor_Task",     // Name of the task (for debugging)
      1000,             // Stack size (bytes)
      NULL,             // Parameter to pass
      tskIDLE_PRIORITY, // Task priority
      NULL              // Task handle
  );
}

void loop()
{
  if (digitalRead(REED_SWITCH_PIN) == LOW && xTaskGetTickCount() - reed_time >= 150)
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
#ifdef DEBUG
  char str[25];
  sprintf(str, "%lu Core ID: %d\n", millis(), xPortGetCoreID());
  Serial.print(str);
#endif

  vTaskDelay(5 / portTICK_PERIOD_MS);
}
