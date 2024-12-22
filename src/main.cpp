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

#include <WiFiClient.h>
#include <StringTokenizer.h>

BleSerial SerialBT;
String CMD_Data[8];

String local_IP;

double BT_TEMP;
double ET_TEMP;
double AMB_TEMP;
int levelOT1;
int levelIO3;
extern double pid_sv;
extern uint16_t last_FAN;
extern uint16_t last_PWR;
extern uint16_t last_SV;

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

    // Init BLE Serial
    SerialBT.begin(deviceName);
    SerialBT.setTimeout(10);
    Serial.println("BT is ready");

    while (WiFi.status() != WL_CONNECTED)
    {

        delay(1000);
        Serial.println("wifi not ready");

        if (tries++ > 1)
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
            if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
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
                            // CMD_String += serialReadBuffer[j]; // copy value
                            j = 0; // clearing
                            break; // 跳出循环
                        }
                        else
                        {
                            serialReadBuffer_clean_OUT[j] = serialReadBuffer[j]; // copy value
                            // CMD_String += serialReadBuffer[j];                   // copy value
                            j++;
                        }
                    }
                }

                // Serial.println(cmd_check);
                CMD_String.trim();
                // Serial.println(CMD_String);
                //  CMD_String.toUpperCase();
                //  cmd from BLE cleaning
                StringTokenizer BLE_CMD(CMD_String, ",");

                while (BLE_CMD.hasNext())
                {
                    CMD_Data[i] = BLE_CMD.nextToken(); // prints the next token in the string
                                                       // Serial.println(CMD_Data[i]);
                    i++;
                }
                i = 0;
                CMD_String = "";

                // AMB_TEMP, ET_TEMP, BT_TEMP, levelOT1, levelIO3);
                //  CMD_Data[0],CMD_Data[1], CMD_Data[2], CMD_Data[3], CMD_Data[4]
                // AMB_TEMP = CMD_Data[0].toDouble();
                ET_TEMP = CMD_Data[1].toDouble();
                BT_TEMP = CMD_Data[2].toDouble();
                levelOT1 = CMD_Data[3].toInt();
                levelIO3 = CMD_Data[4].toInt();
                pid_sv = CMD_Data[5].toDouble();

                mb.Hreg(BT_HREG, int(round(BT_TEMP * 10)));
                mb.Hreg(ET_HREG, int(round(ET_TEMP * 10)));
                mb.Hreg(HEAT_HREG, levelOT1);
                mb.Hreg(FAN_HREG, levelIO3);
                mb.Hreg(PID_SV_HREG, int(round(pid_sv * 10))); // 初始化赋值

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

// Task for reading BLE Serial
void ReadBtTask(void *e)
{
    (void)e;
    const TickType_t xIntervel = 300 / portTICK_PERIOD_MS;
    while (true)
    {
        if (SerialBT.available())
        {
            if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
            {
                auto count = SerialBT.readBytes(bleReadBuffer, BUFFER_SIZE);
                Serial_in.write(bleReadBuffer, count);
#if defined(DEBUG_MODE)
                Serial.write(bleReadBuffer, count);
#endif
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
    const TickType_t xTimeOut = 300 / portTICK_PERIOD_MS;
    String cmd;
    xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        if (xSemaphoreTake(xserialReadBufferMutex, xTimeOut) == pdPASS)
        {
            Serial_in.printf("READ\n");
            xSemaphoreGive(xserialReadBufferMutex);
        }
    }
}

void setup()
{

    // Disable watchdog timers
    // disableCore0WDT();
    // disableCore1WDT();
    disableLoopWDT();
    esp_task_wdt_delete(NULL);
    rtc_wdt_protect_off();
    rtc_wdt_disable();

    xserialReadBufferMutex = xSemaphoreCreateMutex();
    // Start Serial
    Serial_in.setRxBufferSize(BUFFER_SIZE);
    Serial.begin(BAUDRATE);
    Serial_in.begin(BAUDRATE, SERIAL_8N1, RX, TX);
    Serial.printf("Serial is ready\n");
    // Start BLE
    startBluetooth();

    delay(2000);
    // Start tasks
    xTaskCreatePinnedToCore(TASK_Send_READ_CMDtoTC4, "Send_READ_Task", 2048, NULL, 1, &xTASK_Send_READ_CMDtoTC4_handle, 0);
#if defined(DEBUG_MODE)
    Serial.printf("Start Send_READ_Task\n");
#endif

    xTaskCreatePinnedToCore(ReadSerialTask, "ReadSerialTask", 1024 * 8, NULL, 1, &xTASK_ReadSerialTask_handle, 1);
#if defined(DEBUG_MODE)
    Serial.printf("Start ReadSerialTask\n");
#endif
    xTaskCreatePinnedToCore(ReadBtTask, "ReadBtTask", 1024 * 8, NULL, 1, &xTASK_ReadBtTask_handle, 1);
#if defined(DEBUG_MODE)
    Serial.printf("Start ReadBtTask\n");
#endif

//     xTaskCreatePinnedToCore(TASK_TC4_data2Modbus, "TC4_data2Modbus", 1024 * 4, NULL, 1, &xTask_TC4_data2Modbus_handle, 1);
// #if defined(DEBUG_MODE)
//     Serial.printf("Start TC4_data2Modbus\n");
// #endif

    xTaskCreatePinnedToCore(TASK_Modbus_CMD2TC4, "Modbus_CMD2TC4", 1024 * 8, NULL, 1, &xTask_Modbus_CMD2TC4_handle, 1);
#if defined(DEBUG_MODE)
    Serial.printf("Start Modbus_CMD2TC4\n");
#endif

    // INIT MODBUS
    mb.server(502); // Start Modbus IP //default port :502
#if defined(DEBUG_MODE)
    Serial.printf("Start Modbus-TCP  service OK\n");
#endif

    // const uint16_t AMB_TEMP_HREG = 3001;
    // const uint16_t BT_HREG = 3002;
    // const uint16_t ET_HREG = 3003;
    // const uint16_t HEAT_HREG = 3004;
    // const uint16_t FAN_HREG = 3005;
    // const uint16_t PID_SV_HREG = 3006;
    // const uint16_t RESET_HREG = 3007;
    // const uint16_t PID_ON_HREG = 3008;
    // const uint16_t PID_STATUS_HREG = 3009;

    mb.addHreg(AMB_TEMP_HREG);
    mb.addHreg(BT_HREG);
    mb.addHreg(ET_HREG);

    mb.addHreg(HEAT_HREG);
    mb.addHreg(FAN_HREG);
    mb.addHreg(RESET_HREG);

    mb.addHreg(PID_ON_HREG);
    mb.addHreg(PID_SV_HREG);
    mb.addHreg(PID_STATUS_HREG);

#if defined(DEBUG_MODE)
    Serial.printf("modbus add Hreg OK\n");
#endif

    // INIT MODBUS HREG VALUE
    mb.Hreg(AMB_TEMP_HREG, 0); // 初始化赋值
    mb.Hreg(BT_HREG, 0);       // 初始化赋值
    mb.Hreg(ET_HREG, 0);       // 初始化赋值

    mb.Hreg(HEAT_HREG, 0); // 初始化赋值
    mb.Hreg(FAN_HREG, 0);  // 初始化赋值

    mb.Hreg(PID_ON_HREG, 0);     // 初始化赋值
    mb.Hreg(PID_SV_HREG, 0);     // 初始化赋值
    mb.Hreg(PID_STATUS_HREG, 0); // 初始化赋值

    last_FAN = mb.Hreg(FAN_HREG);
    last_PWR = mb.Hreg(HEAT_HREG);
    last_SV = mb.Hreg(PID_SV_HREG);
    pid_on_status = false;

#if defined(DEBUG_MODE)
    Serial.printf("modbus  Hreg init OK\n");
#endif
}
void loop()
{
    mb.task();
}
