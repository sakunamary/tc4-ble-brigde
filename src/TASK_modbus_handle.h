#ifndef __TASK_MODBUS_HANDLE_H__
#define __TASK_MODBUS_HANDLE_H__

#include <Arduino.h>
#include <Wire.h>
#include <ModbusIP_ESP8266.h>

ModbusIP mb; // declear object

// Modbus Registers Offsets
const uint16_t AMB_TEMP_HREG = 3001;
const uint16_t AMB_RH_HREG = 3002;
const uint16_t BT_HREG = 3003;
const uint16_t ET_HREG = 3004;
const uint16_t HEAT_HREG = 3005;
const uint16_t FAN_HREG = 3006;
const uint16_t PID_SV_HREG = 3007;
const uint16_t RESET_HREG = 3008;
const uint16_t PID_ON_HREG = 3009;
const uint16_t PID_STATUS_HREG = 3010;
const uint16_t PID_P_HREG = 3011;
const uint16_t PID_I_HREG = 3012;
const uint16_t PID_D_HREG = 3010;

uint16_t last_FAN;
uint16_t last_PWR;
bool pid_status = false;

bool PID_output;
double pid_sv = 0;

void TASK_TC4_data2Modbus(void *pvParameters)
{ // function
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    uint8_t TEMP_DATA_Buffer[BUFFER_SIZE];

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xSerailDataMutex, xIntervel) == pdPASS)
        {

            xSemaphoreGive(xSerailDataMutex);
        }
    }
}

void TASK_Modbus_CMD2TC4(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t timeOut = 500 / portTICK_PERIOD_MS;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xSerailDataMutex, xIntervel) == pdPASS)
        {

            xSemaphoreGive(xSerailDataMutex);
        }
    }
}

#endif
