#ifndef __TASK_MODBUS_HANDLE_H__
#define __TASK_MODBUS_HANDLE_H__

#include <Arduino.h>
#include <Wire.h>
#include <ModbusIP_ESP8266.h>

ModbusIP mb; // declear object

extern String CMD_Data[8];

// Modbus Registers Offsets
const uint16_t AMB_TEMP_HREG = 3001;
const uint16_t BT_HREG = 3002;
const uint16_t ET_HREG = 3003;
const uint16_t HEAT_HREG = 3004;
const uint16_t FAN_HREG = 3005;
const uint16_t PID_SV_HREG = 3006;
const uint16_t RESET_HREG = 3007;
const uint16_t PID_ON_HREG = 3008;
const uint16_t PID_STATUS_HREG = 3009;

extern double BT_TEMP;
extern double ET_TEMP;
extern double AMB_TEMP;
extern int levelOT1;
extern int levelIO3;
double pid_sv = 0;

bool init_status = true;

bool pid_on_status = false;

bool PID_output;

// void TASK_TC4_data2Modbus(void *pvParameters)
// { // function
//     (void)pvParameters;
//     TickType_t xLastWakeTime;
//     const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
//     const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
//     xLastWakeTime = xTaskGetTickCount();
//     uint8_t TEMP_DATA_Buffer[BUFFER_SIZE];

//     while (1)
//     {
//         vTaskDelayUntil(&xLastWakeTime, xIntervel);
//         if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
//         {

//             // const uint16_t AMB_TEMP_HREG = 3001;
//             // const uint16_t BT_HREG = 3002;
//             // const uint16_t ET_HREG = 3003;
//             // const uint16_t HEAT_HREG = 3004;
//             // const uint16_t FAN_HREG = 3005;
//             // const uint16_t PID_SV_HREG = 3006;
//             // const uint16_t RESET_HREG = 3007;
//             // const uint16_t PID_ON_HREG = 3008;
//             // const uint16_t PID_STATUS_HREG = 3009;

//             mb.Hreg(BT_HREG, int(round(BT_TEMP * 10)));
//             mb.Hreg(ET_HREG, int(round(ET_TEMP * 10)));

//             xSemaphoreGive(xserialReadBufferMutex);
//         }
//     }
// }

void TASK_Modbus_CMD2TC4(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 150 / portTICK_PERIOD_MS;
    const TickType_t xIntervel = 300 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();

    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        if (xSemaphoreTake(xserialReadBufferMutex, timeOut) == pdPASS)
        {
            if (mb.Hreg(RESET_HREG) != 0)
            {
                Serial_in.printf("RESET\n");
                mb.Hreg(RESET_HREG, 0);
            }

            if (levelIO3 != mb.Hreg(FAN_HREG))
            {
                levelIO3 = mb.Hreg(FAN_HREG);
                Serial_in.printf("IO3,%d\n", mb.Hreg(FAN_HREG));
#if defined(DEBUG_MODE)
                Serial.printf("FAN HREG IO3 :%d , %d \n",mb.Hreg(FAN_HREG),levelIO3);
#endif
                
            }

            // RESET timer和风门时随时手动控制
            if (mb.Hreg(PID_ON_HREG) == 1)
            {                               // PID ON
                if (pid_on_status == false) // 状态：mb.Hreg(PID_HREG) == 1 and pid_on_status == false
                {
#if defined(DEBUG_MODE)
                    //   Serial.printf("4:PID_HREG:%d,pid_on_status:%d:init:%d \n", mb.Hreg(PID_ON_HREG), pid_on_status, init_status); // PID ON 当前状态是关
#endif
                    pid_on_status = !pid_on_status;
                    Serial_in.printf("PID,SV,%d\n", mb.Hreg(PID_SV_HREG) / 10);
                    vTaskDelay(50);
                    Serial_in.printf("PID,ON\n"); // 发送指令
                }
                else
                { // 状态：mb.Hreg(PID_HREG) == 1 and pid_on_status == true
#if defined(DEBUG_MODE)
                  //  Serial.printf("6:PID_HREG:%d,pid_on_status:%d:init:%d \n", mb.Hreg(PID_ON_HREG), pid_on_status, init_status);
#endif
                  // 持续发送sv数据，TC4输出：#DATA_OUT，PID，OUT，温度，火力
                    Serial_in.printf("PID,SV,%d\n", mb.Hreg(PID_SV_HREG) / 10);
                }
            }
            else // PID OFF
            {
                if (pid_on_status == true)
                {
                    // 状态：mb.Hreg(PID_HREG) == 0 and pid_on_status == true

#if defined(DEBUG_MODE)
                    // Serial.printf("1:PID_HREG:%d,pid_on_status:%d:init:%d \n", mb.Hreg(PID_ON_HREG), pid_on_status, init_status);
#endif
                    Serial_in.printf("PID,OFF\n"); // 发送指令
                    mb.Hreg(PID_ON_HREG, 0);
                    mb.Hreg(HEAT_HREG, last_PWR); // 回读PID ON之前的OT1数据
                    Serial_in.printf("OT1,%d\n", mb.Hreg(HEAT_HREG));
                    mb.Hreg(PID_SV_HREG, 0); // PID SV 归零
                    Serial_in.printf("PID,SV,0\n");
                    pid_on_status = !pid_on_status; // 同步状态量
                }
                else
                {
#if defined(DEBUG_MODE)
                    // Serial.printf("2:PID_HREG:%d,pid_on_status:%d:init:%d \n", mb.Hreg(PID_ON_HREG), pid_on_status, init_status);
#endif
                    if (levelOT1 != mb.Hreg(HEAT_HREG))
                    {
                        Serial_in.printf("OT1,%d\n", mb.Hreg(HEAT_HREG));
                        levelOT1 = mb.Hreg(HEAT_HREG);
                    }
                }
                //     }
            }
            xSemaphoreGive(xserialReadBufferMutex);
        }
    }
}

#endif