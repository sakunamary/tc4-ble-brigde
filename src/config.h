
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 9600          // serial port baudrate

#define VERSION "1.1.7"

// #define DEBUG_MODE
#define TX 17
#define RX 16

const int BUFFER_SIZE = 48;

static TaskHandle_t xTask_TC4_data2Modbus_handle = NULL;
static TaskHandle_t xTask_Modbus_CMD2TC4_handle = NULL; 
static TaskHandle_t xTASK_Send_READ_CMDtoTC4_handle = NULL;
static TaskHandle_t xTASK_ReadBtTask_handle = NULL;
static TaskHandle_t xTASK_ReadSerialTask_handle = NULL;



SemaphoreHandle_t xDataMutex = NULL;
SemaphoreHandle_t xserialReadBufferMutex = NULL;

HardwareSerial Serial_in(2); // D16 RX_drumer  D17 TX_drumer

#endif