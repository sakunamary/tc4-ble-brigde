// mian.cpp

// TC4 配套使用的模块，将TC4指令转成modbus-TCP发送到artisan和转成BLE数据接口与小程序交互
// 硬件:ESP32/ESP32-s芯片，
// IO :D16  与TC4交互的串口RXD
//     D17  与TC4交互的串口TXD
//
// *** BSD License ***
// ------------------------------------------------------------------------------------------
// Copyright (c) 2011, MLG Properties, LLC
// All rights reserved.
//
// Contributor:  Jim Gallt
//
// Redistribution and use in source and binary forms, with or without modification, are
// permitted provided that the following conditions are met:
//
//   Redistributions of source code must retain the above copyright notice, this list of
//   conditions and the following disclaimer.
//
//   Redistributions in binary form must reproduce the above copyright notice, this list
//   of conditions and the following disclaimer in the documentation and/or other materials
//   provided with the distribution.
//
//   Neither the name of the copyright holder(s) nor the names of its contributors may be
//   used to endorse or promote products derived from this software without specific prior
//   written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS
// OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
// THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// ------------------------------------------------------------------------------------------

// Revision history:
// 20240209 version 1.1.0 : 完成第一个固定milestone

#include <Arduino.h>
#include "config.h"

#include <BleSerial.h>
#include <HardwareSerial.h>
#include <WiFi.h>
// #include <StringTokenizer.h>
// #include <cmndreader.h>

// #include <ModbusIP_ESP8266.h>

HardwareSerial Serial_in(2); // D16 RX_drumer  D17 TX_drumer

SemaphoreHandle_t xserialReadBufferMutex = NULL;                           // Mutex for TC4数据输出时写入队列的数据
QueueHandle_t queueCMD = xQueueCreate(40, sizeof(char[BUFFER_SIZE]));      // 发送到TC4的命令队列
QueueHandle_t queueTC4_data = xQueueCreate(40, sizeof(char[BUFFER_SIZE])); // 接受TC4发出的反馈数据队列
// String local_IP;

// // Modbus Registers Offsets
// const uint16_t BT_HREG = 3001;
// const uint16_t ET_HREG = 3002;
// const uint16_t HEAT_HREG = 3003;
// const uint16_t FAN_HREG = 3004;
// const uint16_t SV_HREG = 3005;
// const uint16_t RESET_HREG = 3006;
// const uint16_t PID_HREG = 3007;

char ap_name[16];
uint8_t macAddr[6];
// double Data[6]; // TC4输出的温度数据：ambient,chan1,chan2,heater duty, fan duty, SV。共6路数据

// bool init_status = true;
// bool pid_on_status = false;

// uint16_t last_SV;
// uint16_t last_FAN;
// uint16_t last_PWR;

BleSerial SerialBT;
// ModbusIP object
// ModbusIP mb;

// CmndInterp ci(DELIM); // command interpreter object

// Task for reading Serial Port  模块发送 READ 指令后，读取Serial的数据 ，写入QueueTC4_data 传递给 TASK_Modbus_Send_DATA
void TASK_GetDataFormTC4(void *pvParameters) // task1
{
    uint8_t serialReadBuffer[BUFFER_SIZE];
    uint8_t ReadBuffer_clean_OUT[BUFFER_SIZE];
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
    int j = 0;
    int count = 0;
    // String CMD_String;
    String task1_TC4_data_String;

    for (;;)
    {
        if (Serial_in.available())
        {
            if (xSemaphoreTake(xserialReadBufferMutex, timeOut) == pdPASS) // lock mutex
            {
                auto count = Serial_in.readBytesUntil('\n', serialReadBuffer, BUFFER_SIZE); // 从串口读取数据进来。
                task1_TC4_data_String = String((char *)serialReadBuffer);                   // 转字到处理符号

                if (serialReadBuffer[0] != 0x23) // 不等于# ，剔除其他无关数据
                {
                    while (j < sizeof(serialReadBuffer) && sizeof(serialReadBuffer) > 0)
                    {
                        if (serialReadBuffer[j] == '\n' || serialReadBuffer[j] == '\0')
                        {
                            ReadBuffer_clean_OUT[j] = serialReadBuffer[j]; // copy value
                            j = 0;                                         // clearing

                            break; // 跳出循环
                        }
                        else
                        {
                            ReadBuffer_clean_OUT[j] = serialReadBuffer[j]; // copy value
                            j++;
                        }
                    }

#if defined(DEBUG_MODE)
                    // Serial.print("TC4 data:");
                    // Serial.printf("%s\n", serialReadBuffer);
#endif

                    // memcpy(BLEReadBuffer_clean, &CMD_String, CMD_String.length());  // 把干净数据添加到输出字符串数组中
                    xQueueSend(queueTC4_data, &ReadBuffer_clean_OUT, timeOut); // 发送到队列
                }
                memset(ReadBuffer_clean_OUT, '\0', sizeof(ReadBuffer_clean_OUT)); // cleaning
                memset(serialReadBuffer, '\0', sizeof(serialReadBuffer));         // cleaning
                task1_TC4_data_String = "";                                       // cleaning
                xSemaphoreGive(xserialReadBufferMutex);
            }
        }
        vTaskDelay(50);
    }
}

void TASK_SendDataTo_BLE(void *pvParameters) // task3
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 300 / portTICK_PERIOD_MS;
    uint8_t BLE_Send_Buffer[BUFFER_SIZE];
    uint8_t BLE_Send_Buffer_clean[BUFFER_SIZE];
    char BLE_Send_out[BUFFER_SIZE];
    String BLE_data_String;
    int j = 0;

    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        // vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xQueueReceive(queueTC4_data, &BLE_Send_Buffer, timeOut) == pdPASS)
        {
            if (xSemaphoreTake(xserialReadBufferMutex, timeOut) == pdPASS)
            {
                sprintf(BLE_Send_out, "#%s;\n", BLE_Send_Buffer);

                // BLE_data_String = String((char *)BLE_Send_Buffer); // 转字到处理符号
                for (j = 0; j < sizeof(BLE_Send_out); j++)
                {
                    BLE_Send_Buffer_clean[j] = BLE_Send_out[j]; // copy value
                }

#if defined(DEBUG_MODE)
                // Serial.print("BLE send in :");
                // Serial.printf("%s\n", BLE_Send_Buffer);
                // Serial.print("BLE send data:");
                Serial.print(BLE_Send_out);
#endif
                // SerialBT.print(BLE_Send_out);
                SerialBT.write(BLE_Send_Buffer_clean, sizeof(BLE_Send_Buffer_clean));
                memset(BLE_Send_Buffer_clean, '\0', sizeof(BLE_Send_Buffer_clean));
                memset(BLE_Send_Buffer, '\0', sizeof(BLE_Send_Buffer));
                memset(BLE_Send_out, '\0', sizeof(BLE_Send_out));
                j = 0;
                BLE_data_String = "";
                xSemaphoreGive(xserialReadBufferMutex);
            }
            vTaskDelay(50);
        }
    }
}
// Task  for Reading BLE 模块读取SerialBT的数据 queueCMD 传递给 TASK_SendCMDtoTC4，实现小程序通过蓝牙发送TC4指令到TC4功能
void TASK_CMD_From_BLE(void *pvParameters) // task2
{
    uint8_t task2_bleReadBuffer[BUFFER_SIZE];
    uint8_t task2_BLEReadBuffer_clean[BUFFER_SIZE];
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
    int j = 0;
    String TASK2_CMD_String;
    String task2_BLE_data_String;
    for (;;)
    {
        if (SerialBT.available())
        {
            if (xSemaphoreTake(xserialReadBufferMutex, timeOut) == pdPASS)
            {
                auto count = SerialBT.readBytes(task2_bleReadBuffer, BUFFER_SIZE);
                task2_BLE_data_String = String((char *)task2_bleReadBuffer);
#if defined(DEBUG_MODE)
                // Serial.print("CMD From BLE:");
                // Serial.println(task2_BLE_data_String);
#endif

                while (j < task2_BLE_data_String.length() && task2_BLE_data_String.length() > 0)
                {

                    if (task2_BLE_data_String[j] == '\n')
                    {
                        TASK2_CMD_String += task2_BLE_data_String[j]; // copy value

                        j = 0; // clearing
                        break; // 跳出循环
                    }
                    else
                    {
                        TASK2_CMD_String += task2_BLE_data_String[j]; // copy value
                        j++;
                    }
                }
                TASK2_CMD_String.trim();
                TASK2_CMD_String.toUpperCase();
                TASK2_CMD_String.concat('\n'); // 增加换行结束符号
                memcpy(task2_BLEReadBuffer_clean, &TASK2_CMD_String, TASK2_CMD_String.length());
                xQueueSend(queueCMD, &task2_BLEReadBuffer_clean, timeOut);
                memset(task2_BLEReadBuffer_clean, '\0', sizeof(task2_BLEReadBuffer_clean));
                memset(task2_bleReadBuffer, '\0', sizeof(task2_bleReadBuffer));
                TASK2_CMD_String = "";
                task2_BLE_data_String = "";
                xSemaphoreGive(xserialReadBufferMutex);
            }
        }
        vTaskDelay(20);
    }
}

// Task for keep sending READ 指令写入queueCMD 传递给 TASK_SendCMDtoTC4
void TASK_Send_READ_CMDtoTC4(void *pvParameters) // task4
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 1500 / portTICK_PERIOD_MS;
    uint8_t TASK4_CMDBuffer[BUFFER_SIZE] = "READ\n";
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
        {
            xQueueSend(queueCMD, &TASK4_CMDBuffer, xIntervel);
            xSemaphoreGive(xserialReadBufferMutex);
        }
    }
}

// Task for keep sending 指令到TC4
void TASK_SendCMDtoTC4(void *pvParameters) // TASK5
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    uint8_t TASK5_CMDBuffer[BUFFER_SIZE];
    char TASK5_CMD_String[BUFFER_SIZE];
    String TASK5_BLE_data_String;
    const TickType_t xIntervel = 300 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
         vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xQueueReceive(queueCMD, &TASK5_CMDBuffer, xIntervel) == pdPASS)
        { // 从接收QueueCMD 接收指令
            if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
            {
                TASK5_BLE_data_String = String((char *)TASK5_CMDBuffer);
#if defined(DEBUG_MODE)
                Serial.print("To TC4 CMD:");
                Serial.print(TASK5_BLE_data_String);
#endif
                // while (j < BLE_data_String.length() && BLE_data_String.length() > 0)
                // {

                //     if (BLE_data_String[j] == '\n')
                //     {
                //         CMD_String += BLE_data_String[j]; // copy value
                //         BLE_data_String = "";
                //         j = 0; // clearing
                //         break; // 跳出循环
                //     }
                //     else
                //     {
                //         CMD_String += BLE_data_String[j]; // copy value
                //         j++;
                //     }
                // }

                sprintf(TASK5_CMD_String, "%s\n", TASK5_CMDBuffer);
                Serial_in.print(TASK5_CMD_String); // 输出指令
                memset(TASK5_CMD_String, '\0', sizeof(TASK5_CMD_String));
                TASK5_BLE_data_String = "";
                xSemaphoreGive(xserialReadBufferMutex);
            }
            vTaskDelay(50);
        }
    }
}

// void TASK_Modbus_Send_DATA(void *pvParameters)
// {
//     // 接收TASK_ReadSerial 传递 QueueTC4_data 数据serialReadBuffer
//     // 将数据serialReadBuffer 转换为字符串
//     // tokenizer 将数据写入Data[]数组
//     // Data[]数组 赋值给Hreg
//     (void)pvParameters;
//     const TickType_t xIntervel = 1500 / portTICK_PERIOD_MS;
//     int i = 0;
//     uint8_t serialReadBuffer[BUFFER_SIZE];
//     String TC4_data_String;

//     for (;;) // A Task shall never return or exit.
//     {        // for loop
//         if (xQueueReceive(queueTC4_data, &serialReadBuffer, xIntervel) == pdPASS)
//         {
//             if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
//             {
//                 TC4_data_String = String((char *)serialReadBuffer);
//             }
//             xSemaphoreGive(xserialReadBufferMutex);
//             if (!TC4_data_String.startsWith("#"))
//             { //
//                 StringTokenizer TC4_Data(TC4_data_String, ",");
//                 while (TC4_Data.hasNext())
//                 {
//                     Data[i] = TC4_Data.nextToken().toDouble(); // prints the next token in the string
//                     i++;
//                 }
//                 mb.Hreg(BT_HREG, Data[1] * 100); //
//                 mb.Hreg(ET_HREG, Data[2] * 100); //
//                 // PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
//                 if ((mb.Hreg(PID_HREG) == 1) && (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS))
//                 {
//                     mb.Hreg(HEAT_HREG, Data[3]); // 获取赋值
//                 }
//                 xSemaphoreGive(xserialReadBufferMutex);
//                 //
//                 i = 0;
//             }
//             else
//             {
//                 TC4_data_String.replace("#DATA_OUT,", "");
//                 ci.checkCmnd(TC4_data_String);
//             }
//         }
//         vTaskDelay(50);
//     }
// }
// // 从PC artisan 获取命令后转为TC4格式命令再发送到TC4
// void TASK_Modbus_From_CMD(void *pvParameters)
// {

//     (void)pvParameters;
//     TickType_t xLastWakeTime;
//     const TickType_t xIntervel = 250 / portTICK_PERIOD_MS;
//     xLastWakeTime = xTaskGetTickCount();

//     for (;;)
//     {
//         vTaskDelayUntil(&xLastWakeTime, xIntervel);
//         if (init_status)
//         {
//             last_FAN = mb.Hreg(FAN_HREG);
//             last_PWR = mb.Hreg(HEAT_HREG);
//             mb.Hreg(PID_HREG, 0);
//             mb.Hreg(FAN_HREG, 0);
//             init_status = false;
//             pid_on_status == false;
//         }
//         else
//         { // RESET timer和风门时随时手动控制
//             if (mb.Hreg(RESET_HREG) != 0)
//             {
//                 Serial_in.printf("RESET\n");
//                 mb.Hreg(RESET_HREG, 0);
//             }

//             if (last_FAN != mb.Hreg(FAN_HREG))
//             {
//                 Serial_in.printf("IO3,%d\n", mb.Hreg(FAN_HREG));
//                 last_FAN = mb.Hreg(FAN_HREG);
//             }

//             if (mb.Hreg(PID_HREG) == 1)
//             {                               // PID ON
//                 if (pid_on_status == false) // 状态：mb.Hreg(PID_HREG) == 1 and pid_on_status == false
//                 {
// #if defined(DEBUG_MODE)
//                     Serial.printf("\n 4:PID_HREG:%d,pid_on_status:%d:init:%d", mb.Hreg(PID_HREG), pid_on_status, init_status); // PID ON 当前状态是关
// #endif
//                     pid_on_status = !pid_on_status;
//                     Serial_in.printf("PID,SV,%d\n", mb.Hreg(SV_HREG) / 10);
//                     vTaskDelay(50);
//                     Serial_in.printf("PID,ON\n"); // 发送指令
//                 }
//                 else
//                 { // 状态：mb.Hreg(PID_HREG) == 1 and pid_on_status == true
// #if defined(DEBUG_MODE)
//                     Serial.printf("\n 6:PID_HREG:%d,pid_on_status:%d:init:%d", mb.Hreg(PID_HREG), pid_on_status, init_status);
// #endif
//                     // 持续发送sv数据，TC4输出：#DATA_OUT，PID，OUT，温度，火力
//                     Serial_in.printf("PID,SV,%d\n", mb.Hreg(SV_HREG) / 10);
//                 }
//             }
//             else // PID OFF
//             {
//                 if (pid_on_status == true)
//                 {
//                     // 状态：mb.Hreg(PID_HREG) == 0 and pid_on_status == true

// #if defined(DEBUG_MODE)
//                     Serial.printf("\n 1:PID_HREG:%d,pid_on_status:%d:init:%d", mb.Hreg(PID_HREG), pid_on_status, init_status);
// #endif
//                     Serial_in.printf("PID,OFF\n"); // 发送指令
//                     mb.Hreg(PID_HREG, 0);
//                     mb.Hreg(HEAT_HREG, last_PWR); // 回读PID ON之前的OT1数据
//                     Serial_in.printf("OT1,%d\n", mb.Hreg(HEAT_HREG));
//                     mb.Hreg(SV_HREG, 0); // PID SV 归零
//                     Serial_in.printf("PID,SV,0\n");
//                     pid_on_status = !pid_on_status; // 同步状态量
//                 }
//                 else
//                 {
// #if defined(DEBUG_MODE)
//                     Serial.printf("\n 2:PID_HREG:%d,pid_on_status:%d:init:%d", mb.Hreg(PID_HREG), pid_on_status, init_status);
// #endif
//                     if (last_PWR != mb.Hreg(HEAT_HREG))
//                     {
//                         Serial_in.printf("OT1,%d\n", mb.Hreg(HEAT_HREG));
//                         last_PWR = mb.Hreg(HEAT_HREG);
//                     }
//                 }
//             }
//         }
//     }
// }

void setup()
{
    xserialReadBufferMutex = xSemaphoreCreateMutex();

#if defined(DEBUG_MODE)
    Serial.begin(BAUDRATE);
    Serial.printf("\nSerial Started\n");
#endif
    Serial_in.begin(BAUDRATE, SERIAL_8N1, RX, TX);

    // 初始化网络服务
    WiFi.macAddress(macAddr);
    // Serial_debug.println("WiFi.mode(AP):");
    // WiFi.mode(WIFI_AP);
    sprintf(ap_name, "MATCHBOX_%02X%02X%02X", macAddr[3], macAddr[4], macAddr[5]);
    //     if (WiFi.softAP(ap_name, "12345678"))
    //     { // defualt IP address :192.168.4.1 password min 8 digis
    // #if defined(DEBUG_MODE)
    //         Serial.printf("\nWiFi AP: %s Started\n", ap_name);
    // #endif
    //     }
    //     else
    //     {
    // #if defined(DEBUG_MODE)
    //         Serial.printf("\nWiFi AP NOT OK YET...\n");
    // #endif
    //         vTaskDelay(500);
    //     }

    // Init BLE Serial
    SerialBT.begin(ap_name, true, 2);
    SerialBT.setTimeout(30);
#if defined(DEBUG_MODE)
    Serial.printf("\nSerial_BT setup OK\n");
#endif

    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TASK_GetDataFormTC4, "DataFormTC4" // 测量电池电源数据，每分钟测量一次
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=1:TASK_GetDataFormTC4 OK\n");
#endif

    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TASK_CMD_From_BLE, "CMD_From_BLE" // 测量电池电源数据，每分钟测量一次
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=2:CMD_From_BLE OK\n");
#endif
    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TASK_SendDataTo_BLE, "TASK_SendDataTo_BLE" // 测量电池电源数据，每分钟测量一次
        ,
        1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=3:TASK_SendDataTo_BLE OK\n");
#endif

    //     // Setup tasks to run independently.
    //     xTaskCreatePinnedToCore(
    //         TASK_Modbus_Send_DATA, "ModbusSendTask" // 测量电池电源数据，每分钟测量一次
    //         ,
    //         1024 * 6 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );
    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK=3:ModbusSendTask OK\n");
    // #endif

    // Setup tasks to run independently.
    xTaskCreate(
        TASK_Send_READ_CMDtoTC4, "READ_CMDtoTC4" // 测量电池电源数据，每分钟测量一次
        ,
        1024 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );

#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=4:READ_CMDtoTC4 OK \n");
#endif

    // Setup tasks to run independently.
    xTaskCreate(
        TASK_SendCMDtoTC4, "SendCMDtoTC4" // 测量电池电源数据，每分钟测量一次
        ,
        1024 * 8 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 2 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );

#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=5:SendCMDtoTC4 OK \n");
#endif

    //     // Setup tasks to run independently.
    //     xTaskCreatePinnedToCore(
    //         TASK_Modbus_From_CMD, "TASK_Modbus_From_CMD" // 测量电池电源数据，每分钟测量一次
    //         ,
    //         1024 * 10 // This stack size can be checked & adjusted by reading the Stack Highwater
    //         ,
    //         NULL, 3 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    //         ,
    //         NULL, 1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    //     );

    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nTASK=6:TASK_Modbus_From_CMD OK \n");
    // #endif

    // Init Modbus-TCP
    // #if defined(DEBUG_MODE)
    //     Serial.printf("\nStart Modbus-TCP  service OK\n");
    // #endif

    //     mb.server(502); // Start Modbus IP //default port :502
    //     // Add SENSOR_IREG register - Use addIreg() for analog Inputs
    //     mb.addHreg(BT_HREG);
    //     mb.addHreg(ET_HREG);
    //     mb.addHreg(HEAT_HREG);
    //     mb.addHreg(FAN_HREG);
    //     mb.addHreg(SV_HREG);
    //     mb.addHreg(RESET_HREG);
    //     mb.addHreg(PID_HREG);

    //     mb.Hreg(BT_HREG, 0);    // 初始化赋值
    //     mb.Hreg(ET_HREG, 0);    // 初始化赋值
    //     mb.Hreg(HEAT_HREG, 0);  // 初始化赋值
    //     mb.Hreg(FAN_HREG, 0);   // 初始化赋值
    //     mb.Hreg(SV_HREG, 0);    // 初始化赋值
    //     mb.Hreg(RESET_HREG, 0); // 初始化赋值
    //     mb.Hreg(PID_HREG, 0);   // 初始化赋值

    //     ////////////////////////////////////////////////////////////////

    //     ci.addCommand(&pid);
    //     ci.addCommand(&io3);
    //     ci.addCommand(&ot1);
}

void loop()
{
    // mb.task();
}