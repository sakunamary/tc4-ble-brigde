
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define VERSION "1.1.2"

//#define DEBUG_MODE
#define TX 17
#define RX 16

const int BUFFER_SIZE = 128;

static TaskHandle_t xTask_TC4_data2Modbus = NULL;

SemaphoreHandle_t xSerailDataMutex = NULL;


QueueHandle_t queue_data = xQueueCreate(15, sizeof(uint8_t[BUFFER_SIZE])); // 发送到TC4的命令队列
QueueHandle_t queueCMD = xQueueCreate(15, sizeof(uint8_t[BUFFER_SIZE]));          // 发送到TC4的命令队列




#endif