
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define VERSION "1.1.5"

#define DEBUG_MODE
#define TX 17
#define RX 16

const int BUFFER_SIZE = 64;
#define BLE_BUFFER_SIZE 64


static TaskHandle_t xTask_TIMER = NULL;
static TaskHandle_t xTASK_BLE_CMD_handle = NULL;

SemaphoreHandle_t xserialReadBufferMutex = NULL; // Mutex for TC4数据输出时写入队列的数据
QueueHandle_t queueCMD_BLE = xQueueCreate(8, sizeof(char[BLE_BUFFER_SIZE]));

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


#endif