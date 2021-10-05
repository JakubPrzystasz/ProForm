
//Debug flag:
#define DEBUG

//Services
#define SERVICE_DEVICE_INFORMATION "180A"
#define SERVICE_CYCLING_POWER "1818"
#define SERVICE_BATTERY "180F"

//Device information serivce characteristics
#define SERVICE_DEVICE_INFROMATION_SYSTEM_ID "2A23"
#define SERVICE_DEVICE_INFROMATION_MODEL_NUMBER_STRING "2A24"
#define SERVICE_DEVICE_INFROMATION_SERIAL_NUMBER_STRING "2A25"
#define SERVICE_DEVICE_INFROMATION_FIRMWARE_REVISION_STRING "2A26"
#define SERVICE_DEVICE_INFROMATION_HARDWARE_REVISION_STRING "2A27"
#define SERVICE_DEVICE_INFROMATION_SOFTWARE_REVISION_STRING "2A28"
#define SERVICE_DEVICE_INFROMATION_MANUFACTURER_NAME_STRING "2A29"
#define SERVICE_DEVICE_INFROMATION_IEEE11073 "2A2A"
#define SERVICE_DEVICE_INFROMATION_PNP_ID "2A50"

//Battery service characteristics
#define SERVICE_BATTERY_LEVEL "2A19"

//Heart rate service characteristics
#define SERVICE_CYCLING_POWER_FEATURE "2A65"
#define SERVICE_CYCLING_POWER_MEASUREMENT "2A63"
#define SERVICE_CYCLING_POWER_SENSOR_LOCATION "2A5D"

//Device properties
const char *string = "ProForm";
const unsigned long long sys_id = 0x4E2F4100;
const unsigned long long iee = 0x4E2F4100;
const unsigned long long pnp = 0x4E2F41;

const unsigned char power_meter_location[1] = {0x0D & 0xff};

const char battery_level[1] = {100};


bool deviceConnected = false, oldDeviceConnected = false;

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];

short power = 150;
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

//Hardware

//BTS MOTOR DRIVER:
#define BTS_EN_R_PIN 23
#define BTS_EN_L_PIN 23

#define BTS_L_PWM_PIN 18
#define BTS_R_PWM_PIN 19
#define BTS_DELAY 3
#define BTS_PWM 128

#define BTS_MAX_VALUE 95
#define BTS_MIN_VALUE 5
#define BTS_DEFAULT 50

//Potentiometer
#define POT_PIN 39

//Switches:
#define REED_SWITCH_PIN 36

#define R_PLUS_PIN 15
#define R_MINUS_PIN 2

#define L_PLUS_PIN 0
#define L_MINUS_PIN 4

#define Aconst 1.0f
#define Bconst 0.0f

#define READ_POT map(analogRead(POT_PIN),0,4095,0,100)