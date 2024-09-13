
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 9600          // serial port baudrate

#define VERSION "1.0.0"

//#define DEBUG_MODE
// #define TX D10
// #define RX D8

const int BUFFER_SIZE = 64;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// QueueHandle_t queue_data_to_BLE = xQueueCreate(8, sizeof(char[BUFFER_SIZE]));
SemaphoreHandle_t xserialReadBufferMutex = NULL; // Mutex for TC4数据输出时写入队列的数据
QueueHandle_t queueCMD_BLE = xQueueCreate(8, sizeof(uint8_t[BUFFER_SIZE]));
static TaskHandle_t xTASK_BLE_CMD_handle = NULL;
static TaskHandle_t xTask_TIMER = NULL;


#endif