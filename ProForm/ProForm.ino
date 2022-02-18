//Reference: https://teaandtechtime.com/arduino-ble-cycling-power-service/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "BTS7960.h"
#include "power.h"
//Hardware
BTS7960 motorController(BTS_EN_L_PIN, BTS_EN_R_PIN, BTS_L_PWM_PIN, BTS_R_PWM_PIN);

//Software
static SemaphoreHandle_t switch_mutex;
static SemaphoreHandle_t reed_mutex;
static BaseType_t xHigher = pdFALSE;

static bool rev = false;

static char str[128] = "";

TaskHandle_t switch_task_t;

uint32_t tmp_time = 0;

int8_t motor_position = BTS_DEFAULT;
int8_t set_position = 0, current_position = 0;
uint32_t reed_time = 0;
uint32_t diff = 0;
float cadence = 0;
float resistance = 0;

static uint32_t switch_debounce = 0;
static uint32_t reed_debounce = 0;

class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    if (xSemaphoreTake(reed_mutex, 10))
    {
      deviceConnected = true;
      xSemaphoreGive(reed_mutex);
    }
#ifdef DEBUG
    sprintf(str, "%lu Device connected!: Core ID: %d\n", millis(), xPortGetCoreID());
    Serial.print(str);
#endif
  };

  void onDisconnect(BLEServer *pServer)
  {
    if (xSemaphoreTake(reed_mutex, 10))
    {
      deviceConnected = true;
      xSemaphoreGive(reed_mutex);
    }
#ifdef DEBUG
    sprintf(str, "%lu Device disconected!: Core ID: %d\n", millis(), xPortGetCoreID());
    Serial.print(str);
#endif
  }
};

void IRAM_ATTR ISR_SWITCH()
{
  if (xSemaphoreTakeFromISR(switch_mutex, &xHigher) == pdTRUE)
  {
    if (xTaskGetTickCountFromISR() - switch_debounce > 250)
    {
      if (digitalRead(R_PLUS_PIN) == LOW)
      {
        motor_position += 5;
      }
      else if (digitalRead(L_PLUS_PIN) == LOW)
      {
        motor_position += 5;
      }
      else if (digitalRead(R_MINUS_PIN) == LOW)
      {
        motor_position -= 5;
      }
      else if (digitalRead(L_MINUS_PIN) == LOW)
      {
        motor_position -= 5;
      }
      switch_debounce = xTaskGetTickCountFromISR();

      if (motor_position < BTS_MIN_VALUE)
        motor_position = BTS_MIN_VALUE;

      if (motor_position > BTS_MAX_VALUE)
        motor_position = BTS_MAX_VALUE;

#ifdef DEBUG
      sprintf(str, "New position: %d\n", motor_position);
      Serial.print(str);
#endif
    }
    xSemaphoreGiveFromISR(switch_mutex, &xHigher);
  }
}

void IRAM_ATTR ISR_REED()
{
  if (xSemaphoreTakeFromISR(reed_mutex, &xHigher) == pdTRUE)
  {
    if (xTaskGetTickCountFromISR() - reed_debounce > 150)
    {
      if (digitalRead(REED_SWITCH_PIN) == LOW)
      {
        reed_debounce = xTaskGetTickCountFromISR();
        rev = !rev;
#ifdef DEBUG
        sprintf(str, "Reed! Device connected: %d  %u\n", deviceConnected, reed_debounce);
        Serial.print(str);
#endif

        if (deviceConnected == true && rev == true)
        {
#ifdef DEBUG
          sprintf(str, "BT: Notifying values %u\n", reed_debounce);
          Serial.print(str);
#endif
          //diff = (float)((float)reed_debounce - (float)reed_time);

          //calculate cadence:
          //cadence = ((1000.0f / diff) * 60.0f);

          // if (xSemaphoreTakeFromISR(switch_mutex, &xHigher) == pdTRUE)
          // {
          //   resistance = map(motor_position, BTS_MIN_VALUE, BTS_MAX_VALUE, 0, 100);
          //   xSemaphoreGiveFromISR(switch_mutex, &xHigher);
          // }

          // power = Aconst * resistance * cadence + Bconst;

          timestamp += (unsigned short)(reed_debounce - reed_time);
          reed_time = reed_debounce;
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
    }
    xSemaphoreGiveFromISR(reed_mutex, &xHigher);
  }
}

void setup()
{
#ifdef DEBUG
  Serial.begin(115200);
#endif

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

  //Setup tasks
  switch_mutex = xSemaphoreCreateMutex();
  reed_mutex = xSemaphoreCreateMutex();
  xSemaphoreGive(switch_mutex);
  xSemaphoreGive(reed_mutex);

  attachInterrupt(R_PLUS_PIN, ISR_SWITCH, FALLING);
  attachInterrupt(L_PLUS_PIN, ISR_SWITCH, FALLING);

  attachInterrupt(R_MINUS_PIN, ISR_SWITCH, FALLING);
  attachInterrupt(L_MINUS_PIN, ISR_SWITCH, FALLING);

  attachInterrupt(REED_SWITCH_PIN, ISR_REED, RISING);

#ifdef DEBUG
  sprintf(str, "SETUP MOTOR POSITION Core ID: %d   %u\n", xPortGetCoreID(), xTaskGetTickCount());
  Serial.print(str);
#endif

  set_position = BTS_DEFAULT;

repeat:
  current_position = READ_POT;

  if (current_position < set_position)
  {
    motorController.Enable();
    motorController.TurnRight(BTS_PWM);
    vTaskDelay(BTS_DELAY / portTICK_PERIOD_MS);
    motorController.Stop();
    motorController.Disable();
  }
  else if (current_position > set_position)
  {
    motorController.Enable();
    motorController.TurnLeft(BTS_PWM);
    vTaskDelay(BTS_DELAY / portTICK_PERIOD_MS);
    motorController.Stop();
    motorController.Disable();
  }

  if (current_position == set_position)
    goto finish;
  goto repeat;

finish:

#ifdef DEBUG
  sprintf(str, "Setup completed Core ID: %d   %u\n", xPortGetCoreID(), xTaskGetTickCount());
  Serial.print(str);
#endif
}

void loop()
{

  if (xTaskGetTickCount() - tmp_time > 2500)
  {
    tmp_time = xTaskGetTickCount();
#ifdef DEBUG
    sprintf(str, "Act/Set: %d/%d  %u\n", current_position, set_position, xTaskGetTickCount());
    Serial.print(str);
#endif
  }

  if (xSemaphoreTake(switch_mutex, portMAX_DELAY))
  {
    set_position = motor_position;
    xSemaphoreGive(switch_mutex);
  }
repeat:
  current_position = READ_POT;

  if (current_position < set_position)
  {
#ifdef DEBUG
    sprintf(str, "Act/Set: %d/%d  R %u\n", current_position, set_position, xTaskGetTickCount());
    Serial.print(str);
#endif
    motorController.Enable();
    motorController.TurnRight(BTS_PWM);
    vTaskDelay(BTS_DELAY / portTICK_PERIOD_MS);
    motorController.Stop();
    motorController.Disable();
  }
  else if (current_position > set_position)
  {
#ifdef DEBUG
    sprintf(str, "Act/Set: %d/%d  L %u\n", current_position, set_position, xTaskGetTickCount());
    Serial.print(str);
#endif
    motorController.Enable();
    motorController.TurnLeft(BTS_PWM);
    vTaskDelay(BTS_DELAY / portTICK_PERIOD_MS);
    motorController.Stop();
    motorController.Disable();
  }

  if (current_position == set_position)
    goto finish;
  goto repeat;

finish:

  // disconnecting
  if (!deviceConnected && oldDeviceConnected)
  {
    vTaskDelay(500);             // give the bluetooth stack the chance to get things ready
    pServer->startAdvertising(); // restart advertising
    if (xSemaphoreTake(reed_mutex, 10))
    {
      oldDeviceConnected = deviceConnected;
      xSemaphoreGive(reed_mutex);
    }
#ifdef DEBUG
    Serial.println("BT: Start advertising");
#endif
  }
  // connecting
  if (deviceConnected && !oldDeviceConnected)
  {
    if (xSemaphoreTake(reed_mutex, 10))
    {
      oldDeviceConnected = deviceConnected;
      xSemaphoreGive(reed_mutex);
    }
#ifdef DEBUG
    Serial.println("BT: Start notyfing");
#endif
  }

  vTaskDelay(5 / portTICK_PERIOD_MS);
}
