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
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <ESP32Time.h>

BleSerial SerialBT;
String local_IP;
HardwareSerial Serial_in(2); // D16 RX_drumer  D17 TX_drumer
ESP32Time rtc;
WebServer server(80);

uint8_t unitMACAddress[6]; // Use MAC address in BT broadcast and display
char deviceName[30];       // The serial string that is broadcast.



uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];
unsigned long ota_progress_millis = 0;

void onOTAStart()
{
    // Log when OTA has started
    // Serial.println("OTA update started!");
    // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final)
{
    // Log every 1 second
    if (millis() - ota_progress_millis > 1000)
    {
        ota_progress_millis = millis();
        // Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
}

void onOTAEnd(bool success)
{
    // Log when OTA has finished
    if (success)
    {
        // Serial.println("OTA update finished successfully!");
    }
    else
    {
        // Serial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
}

// Handle root url (/)
void handle_root()
{
    char index_html[2048];
    String ver = VERSION;
    snprintf(index_html, 2048,
             "<html>\
<head>\
<title>MATCH BOX SETUP</title>\
    </head> \
    <body>\
        <main>\
        <h1 align='center'>BLE version:%s</h1>\
        <div align='center'><a href='/update' target='_blank'>FIRMWARE UPDATE</a>\
        </main>\
        </div>\
    </body>\
</html>\
",
             ver);
    server.send(200, "text/html", index_html);
}

String IpAddressToString(const IPAddress &ipAddress)
{
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}

String processor(const String &var)
{
    // //Serial.println(var);
    if (var == "version")
    {
        return VERSION;
    }

    return String();
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

    while (WiFi.status() != WL_CONNECTED)
    {

        delay(1000);
        // Serial.println("wifi not ready");

        if (tries++ > 2)
        {
            // init wifi
            // Serial.println("WiFi.mode(AP):");
            WiFi.mode(WIFI_AP);
            WiFi.softAP(deviceName, "88888888"); // defualt IP address :192.168.4.1 password min 8 digis
            break;
        }
    }
    // show AP's IP
    // Serial.printf("IP:");
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
                        if (serialReadBuffer[j] == '\n')
                        {
                            j = 0; // clearing
                            break; // 跳出循环
                        }
                        else
                        {
                            serialReadBuffer_clean_OUT[j] = serialReadBuffer[j]; // copy value
                            j++;
                        }
                    }
                    sprintf(BLE_Send_out, "#%s;\n", serialReadBuffer_clean_OUT);
                    // Serial.printf(BLE_Send_out);
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
                Serial.write(bleReadBuffer, count);


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

// Task for keep sending READ 指令写入queueCMD 传递给 TASK_SendCMDtoTC4
void TASK_TIMER(void *pvParameters)
{
    (void)pvParameters;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    String cmd;
    xLastWakeTime = xTaskGetTickCount();
    rtc.setTime(1609459200); // 1st Jan 2021 00:00:00
    for (;;)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);
        Serail.printf("stopwath: %d:%d\n", rtc.getMinute(), rtc.getSecond());
        // rtc.getSecond();
        // rtc.getMinute();
    }
}


void TASK_BLE_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t BLE_CMD_Buffer[BLE_BUFFER_SIZE];
    char BLE_data_buffer_char[BLE_BUFFER_SIZE];
    uint8_t BLE_data_buffer_uint8[BLE_BUFFER_SIZE];
    const TickType_t timeOut = 150;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 150 / portTICK_PERIOD_MS;
    int i = 0;
    int j = 0;
    String TC4_data_String;
    String CMD_String;
    while (1)
    {

        xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
                                  0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
                                  &ulNotificationValue, // 重置前的notification value
                                  portMAX_DELAY);       // 一直等待
        if (xResult == pdTRUE)
        {
            if (xQueueReceive(queueCMD_BLE, &BLE_CMD_Buffer, timeOut) == pdPASS)
            { // 从接收QueueCMD 接收指令

                if (xSemaphoreTake(xSerialReadBufferMutex, xIntervel) == pdPASS)
                {
                    // cmd from BLE cleaning
                    TC4_data_String = String((char *)BLE_CMD_Buffer);

                    while (j < TC4_data_String.length() && TC4_data_String.length() > 0)
                    {

                        if (TC4_data_String[j] == '\n')
                        {
                            CMD_String += TC4_data_String[j]; // copy value
                            TC4_data_String = "";
                            j = 0; // clearing
                            break; // 跳出循环
                        }
                        else
                        {
                            CMD_String += TC4_data_String[j]; // copy value
                            j++;
                        }
                    }
                    CMD_String.trim();
                    CMD_String.toUpperCase();
#if defined(DEBUG_MODE)
                    Serial.println(CMD_String); // for debug
#endif

                    // cmd from BLE cleaning
                    StringTokenizer BLE_CMD(CMD_String, ",");

                    while (BLE_CMD.hasNext())
                    {
                        CMD_Data[i] = BLE_CMD.nextToken(); // prints the next token in the string
                        // Serial.println(CMD_Data[i]);
                        i++;
                    }
                    i = 0;
                    CMD_String = "";
                    xSemaphoreGive(xSerialReadBufferMutex);
                }
                // big handle case switch
                if (CMD_Data[0] == "IO3")
                {
                    if (CMD_Data[1] == "UP")
                    {
                        levelIO3 = levelIO3 + DUTY_STEP;
                        if (levelIO3 > MAX_IO3)
                            levelIO3 = MAX_IO3; // don't allow OT1 to exceed maximum
                        if (levelIO3 < MIN_IO3)
                            levelIO3 = MIN_IO3; // don't allow OT1 to turn on less than minimum
                        pwm_fan.write(map(levelIO3, 0, 100, 230, 1000));
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("FAN:%d\n", levelIO3);//for debug
                        // #endif
                        // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
                        // // 格式转换
                        // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                        // if (deviceConnected)
                        // {
                        //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                        //     pTxCharacteristic->notify();
                        // }
                    }
                    else if (CMD_Data[1] == "DOWN")
                    {
                        levelIO3 = levelIO3 - DUTY_STEP;
                        if (levelIO3 < MIN_IO3 & levelIO3 != 0)
                            levelIO3 = 0; // turn ot1 off if trying to go below minimum. or use levelOT1 = MIN_HTR ?
                        pwm_fan.write(map(levelIO3, 0, 100, 230, 1000));
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("FAN:%d\n", levelIO3);//for debug
                        // #endif
                        // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
                        // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                        // if (deviceConnected)
                        // {
                        //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                        //     pTxCharacteristic->notify();
                        // }
                    }
                    else
                    {
                        uint8_t len = sizeof(CMD_Data[1]);
                        if (len > 0)
                        {
                            levelIO3 = CMD_Data[1].toInt();
                            if (levelIO3 > MAX_IO3)
                                levelIO3 = MAX_IO3; // don't allow OT1 to exceed maximum
                            if (levelIO3 < MIN_IO3 & levelIO3 != 0)
                                levelIO3 = MIN_IO3; // don't allow to set less than minimum unless setting to zero
                            pwm_fan.write(map(levelIO3, 0, 100, 230, 1000));
                            // #if defined(DEBUG_MODE)
                            //                             Serial.printf("FAN:%d\n", levelIO3);//for debug
                            // #endif
                            // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT3,%d\n", levelIO3);
                            // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                            // // 格式转换
                            // if (deviceConnected)
                            // {
                            //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                            //     pTxCharacteristic->notify();
                            // }
                        }
                    }
                }
                else if (CMD_Data[0] == "OT1")
                {
                    if (CMD_Data[1] == "UP")
                    {

                        levelOT1 = levelOT1 + DUTY_STEP;
                        if (levelOT1 > MAX_OT1)
                            levelOT1 = MAX_OT1; // don't allow OT1 to exceed maximum
                        if (levelOT1 < MIN_OT1)
                            levelOT1 = MIN_OT1; // don't allow OT1 to turn on less than minimum
                        pwm_heat.write(map(levelOT1, 0, 100, 0, 1000));
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("HEAT:%d\n", levelOT1);//for debug
                        // #endif
                        // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
                        // // Serial.print(BLE_data_buffer_char);
                        // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                        // // 格式转换
                        // if (deviceConnected)
                        // {
                        //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                        //     pTxCharacteristic->notify();
                        // }
                    }
                    else if (CMD_Data[1] == "DOWN")
                    {
                        levelOT1 = levelOT1 - DUTY_STEP;
                        if (levelOT1 < MIN_OT1 & levelOT1 != 0)
                            levelOT1 = 0; // turn ot1 off if trying to go below minimum. or use levelOT1 = MIN_HTR ?
                        pwm_heat.write(map(levelOT1, 0, 100, 0, 1000));
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("HEAT:%d\n", levelOT1);//for debug
                        // #endif
                        // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
                        // // Serial.print(BLE_data_buffer_char);
                        // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                        // // 格式转换
                        // if (deviceConnected)
                        // {
                        //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                        //     pTxCharacteristic->notify();
                        // }
                    }
                    else
                    {
                        uint8_t len = sizeof(CMD_Data[1]);
                        if (len > 0)
                        {
                            levelOT1 = CMD_Data[1].toInt();
                            if (levelOT1 > MAX_OT1)
                                levelOT1 = MAX_OT1; // don't allow OT1 to exceed maximum
                            if (levelOT1 < MIN_OT1 & levelOT1 != 0)
                                levelOT1 = MIN_OT1; // don't allow to set less than minimum unless setting to zero
                            pwm_heat.write(map(levelOT1, 0, 100, 0, 1000));
                            // #if defined(DEBUG_MODE)
                            //                             Serial.printf("HEAT:%d\n", levelOT1);//for debug
                            // #endif
                            // sprintf(BLE_data_buffer_char, "#DATA_OUT,OT1,%d\n", levelOT1);
                            // // Serial.print(BLE_data_buffer_char);
                            // memcpy(BLE_data_buffer_uint8, BLE_data_buffer_char, sizeof(BLE_data_buffer_char));
                            // // 格式转换
                            // if (deviceConnected)
                            // {
                            //     pTxCharacteristic->setValue(BLE_data_buffer_uint8, sizeof(BLE_data_buffer_uint8));
                            //     pTxCharacteristic->notify();
                            // }
                        }
                    }
                }
                else if (CMD_Data[0] == "PID")
                {
                    if (CMD_Data[1] == "ON")
                    {
                        pid_status = true;
                        Heat_pid_controller.SetMode(AUTOMATIC);
                        // Heat_pid_controller.start();
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("PID is ON\n");//for debug
                        // #endif
                    }
                    else if (CMD_Data[1] == "OFF")
                    {
                        Heat_pid_controller.SetMode(MANUAL);
                        // Heat_pid_controller.stop();
                        // #if defined(DEBUG_MODE)
                        //                         Serial.printf("PID is OFF\n");//for debug
                        // #endif
                        I2C_EEPROM.get(0, pid_parm);
                        Heat_pid_controller.SetTunings(pid_parm.p, pid_parm.i, pid_parm.d);
                        // Heat_pid_controller.setCoefficients(pid_parm.p, pid_parm.i, pid_parm.d);
                        pid_status = false;
                        pid_sv = 0;
                    }
                    else if (CMD_Data[1] == "SV")
                    {
                        if (pid_status == true)
                        {

                            pid_sv = CMD_Data[2].toFloat();
                            // #if defined(DEBUG_MODE)
                            //                             Serial.printf("PID set SV:%4.2f\n", pid_sv);//for debug
                            // #endif
                            // Heat_pid_controller.compute();
                            Heat_pid_controller.Compute();
                            levelOT1=int(round(PID_output));
                            //levelOT1 = map(PID_output, 0, 255, 0, 100);

                            // Serial.printf("PID ON OT1: %d;PID_output:%4.2f\n", levelOT1,PID_output);
                            pwm_heat.write(map(levelOT1, 0, 100, 0, 1000));
                            // #if defined(DEBUG_MODE)
                            //                             Serial.printf("HEAT PID set :%d\n", levelOT1);//for debug
                            // #endif
                        }
                    }
                    else if (CMD_Data[1] == "TUNE")
                    {
                        Heat_pid_controller.SetMode(MANUAL);
                        pid_status = false;
                        pid_sv = 0;

                        vTaskResume(xTask_PID_autotune);
                        delay(100);
                        xTaskNotify(xTask_PID_autotune, 0, eIncrement); // 通知处理任务干活
                        vTaskSuspend(xTASK_BLE_CMD_handle);
                    }
                }
                // END of  big handle case switch
                delay(50);
            }
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
    // Serial.printf("Disable watchdog timers\n");
    rtc.setTime(1609459200); // 1st Jan 2021 00:00:00
    xserialReadBufferMutex = xSemaphoreCreateMutex();
    // Start Serial
    Serial_in.setRxBufferSize(BUFFER_SIZE);
    Serial.begin(BAUDRATE);
    Serial_in.begin(BAUDRATE, SERIAL_8N1, RX, TX);

    // Start BLE
    startBluetooth();

    // Start tasks
    xTaskCreate(ReadSerialTask, "ReadSerialTask", 10240, NULL, 1, NULL);
    // Serial.printf("Start ReadSerialTask\n");
    xTaskCreate(ReadBtTask, "ReadBtTask", 10240, NULL, 1, NULL);
    // Serial.printf("Start ReadBtTask\n");
    xTaskCreate(TASK_Send_READ_CMDtoTC4, "Send_READ_Task", 10240, NULL, 1, NULL);
    // Serial.printf("Start Send_READ_Task\n");
    xTaskCreate(TASK_TIMER, "TASK_TIMER", 10240, NULL, 1, NULL);
    // Serial.printf("Start Send_READ_Task\n");
    vTaskSuspend(TASK_TIMER);

    server.on("/", handle_root);
    ElegantOTA.begin(&server); // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    // Serial.println("HTTP server started");
}
void loop()
{
    server.handleClient();
    ElegantOTA.loop();
}
