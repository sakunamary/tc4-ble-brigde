/**
 * Bluetooth LE Serial Bridge Example
 *
 * Creates a bridge between the default serial port and a Bluetooth LE Serial port.
 * Data received from BLE is transferred to the serial port and
 * data receivedfrom serial port is transferred to BLE.
 *
 * Avinab Malla
 * 28 December 2022
 **/

#include <Arduino.h>
#include "config.h"
#include <BleSerial.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"
#include <HardwareSerial.h>
#include <WiFi.h>

BleSerial SerialBT;
HardwareSerial Serial_in(2); // D16 RX_drumer  D17 TX_drumer

uint8_t unitMACAddress[6]; // Use MAC address in BT broadcast and display
char deviceName[20];       // The serial string that is broadcast.

SemaphoreHandle_t xserialReadBufferMutex = NULL; // Mutex for TC4数据输出时写入队列的数据
uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];

void startBluetooth()
{
    // Get unit MAC address
    WiFi.macAddress(unitMACAddress);
    // Serial_debug.println("WiFi.mode(AP):");
    // WiFi.mode(WIFI_AP);
    sprintf(deviceName, "MATCHBOX_%02X%02X%02X", unitMACAddress[3], unitMACAddress[4], unitMACAddress[5]);
    // Init BLE Serial
    SerialBT.begin(deviceName);
    SerialBT.setTimeout(10);
}

// Task for reading Serial Port
void ReadSerialTask(void *e)
{
    (void)e;
    const TickType_t xIntervel = 250 / portTICK_PERIOD_MS;
    char BLE_Send_out[BUFFER_SIZE];
    uint8_t serialReadBuffer_clean_OUT[BUFFER_SIZE];
    // String cmd_check;
    int j = 0;
    while (true)
    {
        if (Serial_in.available())
        {
            if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
            {
                auto count = Serial_in.readBytes(serialReadBuffer, BUFFER_SIZE);
                // cmd_check = String((char *)serialReadBuffer);
                // Serial.println(cmd_check);
                if (serialReadBuffer[0] != 0x23) // 不等于# ，剔除其他无关数据
                {
                    while (j < sizeof(serialReadBuffer) && sizeof(serialReadBuffer) > 0)
                    {
                        if (serialReadBuffer[j] == '\n' || serialReadBuffer[j] == '\0')
                        {
                            j = 0;                                               // clearing
                            break; // 跳出循环
                        }
                        else
                        {
                            serialReadBuffer_clean_OUT[j] = serialReadBuffer[j]; // copy value
                            j++;
                        }
                    }
                    sprintf(BLE_Send_out, "#%s;\n", serialReadBuffer_clean_OUT);
                    Serial.printf(BLE_Send_out);
                    SerialBT.printf(BLE_Send_out);
                }
                xSemaphoreGive(xserialReadBufferMutex);
            }

            delay(50);
        }
    }
}

// Task for reading BLE Serial
void ReadBtTask(void *e)
{
    (void)e;
    const TickType_t xIntervel = 250 / portTICK_PERIOD_MS;
    while (true)
    {
        if (SerialBT.available())
        {
            if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
            {
                auto count = SerialBT.readBytes(bleReadBuffer, BUFFER_SIZE);
                Serial_in.write(bleReadBuffer, count);
                // Serial.write(bleReadBuffer, count);
                xSemaphoreGive(xserialReadBufferMutex);
            }
            delay(50);
        }
    }
}
// Task for keep sending READ 指令写入queueCMD 传递给 TASK_SendCMDtoTC4
void TASK_Send_READ_CMDtoTC4(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 1500 / portTICK_PERIOD_MS;
    String cmd;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
        {
            Serial_in.printf("READ\n");
            xSemaphoreGive(xserialReadBufferMutex);
        }
    }
}

void setup()
{

    xserialReadBufferMutex = xSemaphoreCreateMutex();
    // Start Serial
     Serial_in.setRxBufferSize(BUFFER_SIZE);
    Serial.begin(BAUDRATE);
    // Serial_in.setTimeout(10);
    Serial_in.begin(BAUDRATE, SERIAL_8N1, RX, TX);

    // Start BLE
    startBluetooth();

    // Disable watchdog timers
    disableCore0WDT();
    disableCore1WDT();
    disableLoopWDT();
    esp_task_wdt_delete(NULL);
    rtc_wdt_protect_off();
    rtc_wdt_disable();
    Serial.printf("Disable watchdog timers\n");
    // Start tasks

    xTaskCreate(ReadSerialTask, "ReadSerialTask", 10240, NULL, 1, NULL);
    Serial.printf("Start ReadSerialTask\n");
    xTaskCreate(ReadBtTask, "ReadBtTask", 10240, NULL, 1, NULL);
    Serial.printf("Start ReadSerialTask\n");
    xTaskCreate(TASK_Send_READ_CMDtoTC4, "Send_READ_Task", 10240, NULL, 1, NULL);
    Serial.printf("Start Send_READ_Task\n");
}

void loop()
{
    // This task is not used
    vTaskDelete(NULL);
}
