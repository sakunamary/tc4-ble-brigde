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
#include "TASK_modbus_handle.h"
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"
#include <HardwareSerial.h>
#include <BleSerial.h>

#include <WiFi.h>
#include <WiFiClient.h>
// #include <WebServer.h>
// #include <ElegantOTA.h>

#include <StringTokenizer.h>

BleSerial SerialBT;
String CMD_Data[8];

String local_IP;
HardwareSerial Serial_in(2); // D16 RX_drumer  D17 TX_drumer

// WebServer server(80);

uint8_t unitMACAddress[6]; // Use MAC address in BT broadcast and display
char deviceName[30];       // The serial string that is broadcast.

uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];

String IpAddressToString(const IPAddress &ipAddress)
{
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

void startBluetooth()
{
    byte tries = 0;
    // Get unit MAC address
    WiFi.macAddress(unitMACAddress);
    sprintf(deviceName, "MATCHBOX_%02X%02X%02X", unitMACAddress[3], unitMACAddress[4], unitMACAddress[5]);

    while (WiFi.status() != WL_CONNECTED)
    {

        delay(1000);
        Serial.println("wifi not ready");

        if (tries++ > 5)
        {
            // init wifi
            Serial.println("WiFi.mode(AP):");
            WiFi.mode(WIFI_AP);
            WiFi.softAP(deviceName, "matchbox8888"); // defualt IP address :192.168.4.1 password min 8 digis
            break;
        }
    }
    // show AP's IP
    Serial.printf("IP:");
    if (WiFi.getMode() == 2) // 1:STA mode 2:AP mode
    {
        Serial.println(IpAddressToString(WiFi.softAPIP()));
        local_IP = IpAddressToString(WiFi.softAPIP());
    }
    else
    {
        Serial.println(IpAddressToString(WiFi.localIP()));
        local_IP = IpAddressToString(WiFi.localIP());
    }
}

// Task for reading Serial Port
void ReadSerialTask(void *e)
{
    (void)e;
    const TickType_t xIntervel = 300 / portTICK_PERIOD_MS;
    char BLE_Send_out[BUFFER_SIZE];
    uint8_t serialReadBuffer_clean_OUT[BUFFER_SIZE];
    int j = 0;
    int i = 0;

    String CMD_String;
#if defined(DEBUG_MODE)
    String cmd_check;
#endif
    while (true)
    {
        if (Serial_in.available())
        {
            if (xSemaphoreTake(xSerailDataMutex, xIntervel) == pdPASS)
            {
                auto count = Serial_in.readBytes(serialReadBuffer, BUFFER_SIZE);
                CMD_String = String((char *)serialReadBuffer);
#if defined(DEBUG_MODE)
                // cmd_check = String((char *)serialReadBuffer);
                // Serial.println(cmd_check);
#endif
                if (serialReadBuffer[0] != 0x23) // 不等于# ，剔除其他无关数据
                {
                    while (j < sizeof(serialReadBuffer) && sizeof(serialReadBuffer) > 0)
                    {
                        if (serialReadBuffer[j] == '\n' || serialReadBuffer[j] == '\r')
                        {

                            j = 0;                             // clearing
                            break;                             // 跳出循环
                        }
                        else
                        {
                            serialReadBuffer_clean_OUT[j] = serialReadBuffer[j]; // copy value

                            j++;
                        }
                    }
                }

                // Serial.println(cmd_check);
                CMD_String.trim();

                StringTokenizer BLE_CMD(CMD_String, ",");

                while (BLE_CMD.hasNext())
                {
                    CMD_Data[i] = BLE_CMD.nextToken(); // prints the next token in the string
                                                       // Serial.println(CMD_Data[i]);
                    i++;
                }
                i = 0;
                CMD_String = "";

                sprintf(BLE_Send_out, "#%s,%s,%s,%s;\r\n", CMD_Data[1], CMD_Data[2], CMD_Data[3], CMD_Data[4]);
#if defined(DEBUG_MODE)
                 Serial.printf(BLE_Send_out);
#endif
                SerialBT.printf(BLE_Send_out);
                xSemaphoreGive(xserialReadBufferMutex);
                delay(50);
            }
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
        if (xSemaphoreTake(xSerailDataMutex, xIntervel) == pdPASS)
        {
            Serial_in.printf("READ\n");
            xSemaphoreGive(xSerailDataMutex);
        }
    }
}




void setup()
{

    // Disable watchdog timers
    disableCore0WDT();
    disableCore1WDT();
    disableLoopWDT();
    esp_task_wdt_delete(NULL);
    rtc_wdt_protect_off();
    rtc_wdt_disable();
    Serial.printf("Disable watchdog timers\n");

    xSerailDataMutex = xSemaphoreCreateMutex();
    // Start Serial
    Serial_in.setRxBufferSize(BUFFER_SIZE);
    Serial.begin(BAUDRATE);
    Serial_in.begin(BAUDRATE, SERIAL_8N1, RX, TX);

    // Start BLE
    startBluetooth();

    // Start tasks
    xTaskCreate(ReadSerialTask, "ReadSerialTask", 10240, NULL, 1, NULL);
    Serial.printf("Start ReadSerialTask\n");
    xTaskCreate(TASK_Send_READ_CMDtoTC4, "Send_READ_Task", 10240, NULL, 2, NULL);
    Serial.printf("Start Send_READ_Task\n");
    xTaskCreate(TASK_TC4_data2Modbus, "TC4_data2Modbus", 10240, NULL, 1, &xTask_TC4_data2Modbus);
    Serial.printf("Start TC4_data2Modbus\n");
    xTaskCreate(TASK_Modbus_CMD2TC4, "Modbus_CMD2TC4", 10240, NULL, 1, NULL);
    Serial.printf("Start Modbus_CMD2TC4\n");

    // INIT MODBUS
    mb.server(502); // Start Modbus IP //default port :502
#if defined(DEBUG_MODE)
    Serial.printf("\nStart Modbus-TCP  service OK\n");
#endif
    // Add SENSOR_IREG register - Use addIreg() for analog Inputs
    // PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
    // AMB_TEMP,ET_HREG,BT_HREG,HEAT_HREG,FAN_HREG,PID_SV_HREG
    // const uint16_t AMB_TEMP_HREG = 3001; //AMB_TEMP
    // const uint16_t AMB_RH_HREG = 3002;
    // const uint16_t BT_HREG = 3003;
    // const uint16_t ET_HREG = 3004;
    // const uint16_t HEAT_HREG = 3005; //OT1
    // const uint16_t FAN_HREG = 3006; //IO3
    // const uint16_t PID_SV_HREG = 3007; //PID_SV
    // const uint16_t RESET_HREG = 3008;
    // const uint16_t PID_ON_HREG = 3009;
    // const uint16_t PID_P_HREG = 3010;
    // const uint16_t PID_I_HREG = 3011;
    // const uint16_t PID_D_HREG = 3012;

    mb.addHreg(AMB_TEMP_HREG);

    mb.addHreg(BT_HREG);
    mb.addHreg(ET_HREG);

    mb.addHreg(HEAT_HREG);
    mb.addHreg(FAN_HREG);
    mb.addHreg(RESET_HREG);

    mb.addHreg(PID_SV_HREG);
    mb.addHreg(PID_STATUS_HREG);

    // INIT MODBUS HREG VALUE
    mb.Hreg(AMB_TEMP_HREG, 0); // 初始化赋值
    mb.Hreg(BT_HREG, 0);       // 初始化赋值
    mb.Hreg(ET_HREG, 0);       // 初始化赋值

    mb.Hreg(HEAT_HREG, 0); // 初始化赋值
    mb.Hreg(FAN_HREG, 30); // 初始化赋值

    mb.Hreg(PID_SV_HREG, 0);     // 初始化赋值
    mb.Hreg(PID_STATUS_HREG, 0); // 初始化赋值


    // server.begin();
    // Serial.println("HTTP server started");
}
void loop()
{
    mb.task();

}
