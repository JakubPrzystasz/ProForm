
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

//Hardware

//BTS MOTOR DRIVER:
#define BTS_EN_PIN 12
#define BTS_L_PWM_PIN 14
#define BTS_R_PWM_PIN 27

#define BTS_MAX_VALUE

//Potentiometer
#define POT_PIN 3

//Switches:
#define REED_SWITCH_PIN 34

#define R_PLUS_PIN 4
#define R_MINUS_PIN 15

#define L_PLUS_PIN 0
#define L_MINUS_PIN 2

#define READ_POT map(analogRead(POT_PIN),0,4095,0,100)