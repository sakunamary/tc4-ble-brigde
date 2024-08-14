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
#include <StringTokenizer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <StopWatch.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2_for_Adafruit_GFX.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;

BleSerial SerialBT;
String local_IP;
HardwareSerial Serial_in(2); // D16 RX_drumer  D17 TX_drumer
WebServer server(80);
StopWatch sw_secs(StopWatch::SECONDS);

uint8_t unitMACAddress[6]; // Use MAC address in BT broadcast and display
char deviceName[30];       // The serial string that is broadcast.
String CMD_Data[6];
byte tries;

SemaphoreHandle_t xserialReadBufferMutex = NULL; // Mutex for TC4数据输出时写入队列的数据
QueueHandle_t queueCMD_BLE = xQueueCreate(8, sizeof(uint8_t[BUFFER_SIZE]));
static TaskHandle_t xTASK_BLE_CMD_handle = NULL;
static TaskHandle_t xTask_TIMER = NULL;

uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];
unsigned long ota_progress_millis = 0;
unsigned long stopwatch_millis = 0;

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
        // u8g2_font_percent_circle_25_hn

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
    Serial.println("BTSerial is OK");

    while (WiFi.status() != WL_CONNECTED)
    {

        delay(1000);
        Serial.println("wifi not ready");

        if (tries++ > 1)
        {
            // init wifi
            Serial.println("WiFi.mode(AP):");
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
                // Serial.write(bleReadBuffer, count);
                xQueueSend(queueCMD_BLE, &bleReadBuffer, xIntervel); // 串口数据发送至队列
                xTaskNotify(xTASK_BLE_CMD_handle, 0, eIncrement);
                xSemaphoreGive(xserialReadBufferMutex);
            }
            delay(50);
        }
    }
}

// Task for reading Serial Port
void ReadSerialTask(void *e)
{
    (void)e;
    const TickType_t xIntervel = 300 / portTICK_PERIOD_MS;
    char BLE_Send_out[BUFFER_SIZE];
    uint8_t serialReadBuffer_clean_OUT[BUFFER_SIZE];
    String cmd_check;
    int j = 0;
    while (true)
    {
        if (Serial_in.available())
        {
            if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
            {
                auto count = Serial_in.readBytes(serialReadBuffer, BUFFER_SIZE);
                cmd_check = String((char *)serialReadBuffer);
                Serial.println(cmd_check);
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

//
void TASK_TIMER(void *pvParameters)
{
    (void)pvParameters;
    uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
    BaseType_t xResult;
    TickType_t xLastWakeTime;
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
    xLastWakeTime = xTaskGetTickCount();
    char time[5];
    while (1)
    {
        vTaskDelayUntil(&xLastWakeTime, xIntervel);

        display.clearDisplay();
        u8g2_for_adafruit_gfx.setFont(u8g2_font_open_iconic_all_4x_t); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
        u8g2_for_adafruit_gfx.setFontMode(1);                          // use u8g2 transparent mode (this is default)
        u8g2_for_adafruit_gfx.setFontDirection(0);                     // left to right (this is default)
        u8g2_for_adafruit_gfx.setForegroundColor(WHITE);               // apply Adafruit GFX color
        u8g2_for_adafruit_gfx.drawGlyph(0, 32, 0x007b);

        u8g2_for_adafruit_gfx.setFont(u8g2_font_maniac_tn); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall

        sprintf(time, "%02d:%02d", sw_secs.elapsed() / 60, sw_secs.elapsed() % 60);
        u8g2_for_adafruit_gfx.drawStr(42, 27, time);

        display.display();
        // Serial.printf("Roaset time %02d:%02d\n", sw_secs.elapsed() / 60, sw_secs.elapsed() % 60);
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
    const TickType_t xIntervel = 1000 / portTICK_PERIOD_MS;
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

                if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
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
                    // #if defined(DEBUG_MODE)
                    //                     // Serial.println(CMD_String); // for debug
                    // #endif
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
                    xSemaphoreGive(xserialReadBufferMutex);
                }
                // big handle case switch
                if (CMD_Data[0] == "PID")
                {
                    if (CMD_Data[1] == "ON")
                    {
                        sw_secs.reset();
                        sw_secs.start();
                        vTaskResume(xTask_TIMER);
                        // xTaskNotify(xTask_TIMER, 0, eIncrement);
                        Serial.printf("PID is ON\n"); // for debug
                    }
                    else if (CMD_Data[1] == "OFF")
                    {
                        Serial.printf("PID is OFF\n"); // for debug
                        sw_secs.stop();
                        sw_secs.reset();
                        vTaskSuspend(xTask_TIMER);
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
    xserialReadBufferMutex = xSemaphoreCreateMutex();

    sw_secs.start();
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    u8g2_for_adafruit_gfx.begin(display); // connect u8g2 procedures to Adafruit GFX

    display.clearDisplay();
    u8g2_for_adafruit_gfx.setFont(u8g2_font_open_iconic_all_4x_t); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
    u8g2_for_adafruit_gfx.setFontMode(1);                          // use u8g2 transparent mode (this is default)
    u8g2_for_adafruit_gfx.setFontDirection(0);                     // left to right (this is default)
    u8g2_for_adafruit_gfx.setForegroundColor(WHITE);               // apply Adafruit GFX color
    u8g2_for_adafruit_gfx.drawGlyph(0, 32, 0x007b);

    u8g2_for_adafruit_gfx.setFont(u8g2_font_maniac_tn); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
    u8g2_for_adafruit_gfx.drawStr(42, 27, "00:00");

    display.display();

    // Start Serial
    Serial_in.setRxBufferSize(BUFFER_SIZE);
    Serial.begin(BAUDRATE);
    Serial_in.begin(BAUDRATE, SERIAL_8N1, RX, TX);

    // Start BLE
    startBluetooth();

    // Start tasks
    Serial.printf("Start ReadSerialTask\n");
    xTaskCreate(ReadSerialTask, "ReadSerialTask", 1024 * 4, NULL, 1, NULL); // read serial(TC4) data ,and send to BLE

    Serial.printf("Start ReadBtTask\n");
    xTaskCreate(ReadBtTask, "ReadBtTask", 1024 * 4, NULL, 1, NULL); // read BLE cmnd

    Serial.printf("Start Send_READ_Task\n");
    xTaskCreate(TASK_Send_READ_CMDtoTC4, "Send_READ_Task", 1024 * 2, NULL, 1, NULL); // keep sending READ cmnd to TC4 every 1500ms

    Serial.printf("Start TASK_BLE_CMD_handle\n");
    xTaskCreate(TASK_BLE_CMD_handle, "TASK_BLE_CMD_handle", 10240, NULL, 1, &xTASK_BLE_CMD_handle); // once get cmnd form BLE service then do something

    Serial.printf("Start TASK_TIMER\n");
    xTaskCreate(TASK_TIMER, "TASK_TIMER", 1024 * 4, NULL, 1, &xTask_TIMER); // stopwatch task
    vTaskSuspend(xTask_TIMER);

    server.on("/", handle_root);
    ElegantOTA.begin(&server); // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    Serial.println("HTTP server started");
}
void loop()
{
    server.handleClient();
    ElegantOTA.loop();
}
//