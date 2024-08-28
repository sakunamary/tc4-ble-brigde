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
// #include <BleSerial.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"
// #include <HardwareSerial.h>
#include <StringTokenizer.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>
#include <StopWatch.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <U8g2_for_Adafruit_GFX.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

#define OLED_RESET -1       // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
U8G2_FOR_ADAFRUIT_GFX u8g2_for_adafruit_gfx;
BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;

// BleSerial SerialBT;
String local_IP;
// HardwareSerial Serial_in(1); // D16 RX_drumer  D17 TX_drumer
WebServer server(80);
StopWatch sw_secs(StopWatch::SECONDS);

uint8_t unitMACAddress[6]; // Use MAC address in BT broadcast and display
char deviceName[30];       // The serial string that is broadcast.
String CMD_Data[6];
byte tries;
bool deviceConnected = false;
bool oldDeviceConnected = false;

uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];
unsigned long ota_progress_millis = 0;
unsigned long stopwatch_millis = 0;

void onOTAStart()
{
    // Log when OTA has started
    // Serial.println("OTA update started!");
    display.clearDisplay();
    u8g2_for_adafruit_gfx.setFont(u8g2_font_open_iconic_embedded_4x_t); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
    u8g2_for_adafruit_gfx.drawGlyph(0, 32, 0x0048);
    u8g2_for_adafruit_gfx.setFont(u8g2_font_logisoso24_tr);
    u8g2_for_adafruit_gfx.drawStr(34, 28, "UPDATE");
    display.display();
}

void onOTAProgress(size_t current, size_t final)
{
    display.clearDisplay();
    u8g2_for_adafruit_gfx.setFont(u8g2_font_open_iconic_embedded_4x_t);
    u8g2_for_adafruit_gfx.drawGlyph(0, 32, 0x0048);
    u8g2_for_adafruit_gfx.setFont(u8g2_font_logisoso24_tr);
    u8g2_for_adafruit_gfx.drawStr(55, 28, "OTA");
    display.display();
}

void onOTAEnd(bool success)
{
    // Log when OTA has finished
    if (success)
    {
        display.clearDisplay();
        u8g2_for_adafruit_gfx.setFont(u8g2_font_open_iconic_embedded_4x_t); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
        u8g2_for_adafruit_gfx.drawGlyph(0, 32, 0x0048);
        u8g2_for_adafruit_gfx.setFont(u8g2_font_logisoso24_tr);
        u8g2_for_adafruit_gfx.drawStr(50, 28, "DONE");
        display.display();
    }
    else
    {
        display.clearDisplay();
        u8g2_for_adafruit_gfx.setFont(u8g2_font_open_iconic_embedded_4x_t); // select u8g2 font from here: https://github.com/olikraus/u8g2/wiki/fntlistall
        u8g2_for_adafruit_gfx.drawGlyph(0, 32, 0x0048);
        u8g2_for_adafruit_gfx.setFont(u8g2_font_logisoso24_tr);
        u8g2_for_adafruit_gfx.drawStr(42, 28, "ERROR");
        display.display();
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

class MyServerCallbacks : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer)
    {
        deviceConnected = false;
    }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        std::string rxValue = pCharacteristic->getValue();
        uint8_t BLE_DATA_Buffer[BUFFER_SIZE];
        int i = 0;
        while (i < rxValue.length() && rxValue.length() > 0)
        {
            Serial.print(rxValue[i]);
            if (rxValue[i] == 0x0A)
            {
                BLE_DATA_Buffer[i] = rxValue[i];                  // copy value
                xQueueSend(queueCMD_BLE, &BLE_DATA_Buffer, 300/ portTICK_PERIOD_MS);  // 串口数据发送至队列
                xTaskNotify(xTASK_BLE_CMD_handle, 0, eIncrement); // 通知处理任务干活
                memset(&BLE_DATA_Buffer, '\0', BUFFER_SIZE);
                i = 0; // clearing
                break; // 跳出循环
            }
            else
            {
                BLE_DATA_Buffer[i] = rxValue[i];
                i++;
            }
        }
        delay(50);
    }
};

// void TASK_DATA_to_BLE(void *pvParameters)
// {
//     (void)pvParameters;
//     uint8_t BLE_DATA_Buffer[BUFFER_SIZE];
//     const TickType_t timeOut = 150;
//     uint32_t ulNotificationValue; // 用来存放本任务的4个字节的notification value
//     BaseType_t xResult;

//     while (1)
//     {
//         xResult = xTaskNotifyWait(0x00,                 // 在运行前这个命令之前，先清除这几位
//                                   0x00,                 // 运行后，重置所有的bits 0x00 or ULONG_MAX or 0xFFFFFFFF
//                                   &ulNotificationValue, // 重置前的notification value
//                                   portMAX_DELAY);       // 一直等待
//         if (xResult == pdTRUE)
//         {
//             if (xQueueReceive(queue_data_to_BLE, &BLE_DATA_Buffer, timeOut) == pdPASS)

//             { // 从接收QueueCMD 接收指令
// #if defined(DEBUG_MODE)
//                 Serial.println(String((char *)BLE_DATA_Buffer));
// #endif
//                 if (deviceConnected)
//                 {
//                     pTxCharacteristic->setValue(BLE_DATA_Buffer, sizeof(BLE_DATA_Buffer));
//                     pTxCharacteristic->notify();
//                 }
//                 // data frame:PID ON:ambient,chan1,chan2,  heater duty, fan duty, SV
//                 delay(50);
//             }
//         }
//     }
// }

void startBluetooth()
{
    byte tries = 0;
    // Get unit MAC address
    WiFi.macAddress(unitMACAddress);
    sprintf(deviceName, "MATCHBOX_%02X%02X%02X", unitMACAddress[3], unitMACAddress[4], unitMACAddress[5]);

    // Init BLE Serial
    // Create the BLE Device
    BLEDevice::init(deviceName);
    // Create the BLE Server
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());
    // Create the BLE Service
    BLEService *pService = pServer->createService(SERVICE_UUID);
    // Create a BLE Characteristic
    pTxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_TX,
        BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());

    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID_RX,
        BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new MyCallbacks());
    // Start the service
    pService->start();
    // Start advertising
    pServer->getAdvertising()->start();

    // Serial.println("BTSerial is OK");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(deviceName, "88888888"); // defualt IP address :192.168.4.1 password min 8 digis

// show AP's IP
#ifdef DEBUG_MODE
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
#endif
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
        if (Serial.available())
        {
            if (xSemaphoreTake(xserialReadBufferMutex, xIntervel) == pdPASS)
            {
                auto count = Serial.readBytes(serialReadBuffer, BUFFER_SIZE);
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
                    Serial.printf(BLE_Send_out);
                    if (deviceConnected)
                    {
                        pTxCharacteristic->setValue(serialReadBuffer_clean_OUT, sizeof(serialReadBuffer_clean_OUT));
                        pTxCharacteristic->notify();
                    }
                    // xQueueSend(queueCMD_BLE, &BLE_DATA_Buffer, 100);  // 串口数据发送至队列
                    //  xTaskNotify(xTASK_BLE_CMD_handle, 0, eIncrement); // 通知处理任务干活
                    // SerialBT.printf(BLE_Send_out);
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
            Serial.printf("READ\n");
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
    }
}

void TASK_BLE_CMD_handle(void *pvParameters)
{
    (void)pvParameters;
    uint8_t BLE_CMD_Buffer[BUFFER_SIZE];
    char BLE_data_buffer_char[BUFFER_SIZE];
    uint8_t BLE_data_buffer_uint8[BUFFER_SIZE];
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
                        Serial.printf("PID is ON\n"); // for debug
                    }
                    else if (CMD_Data[1] == "OFF")
                    {
                        Serial.printf("PID is OFF\n"); // for debug
                        sw_secs.stop();
                        vTaskSuspend(xTask_TIMER);
                        display.clearDisplay();
                        u8g2_for_adafruit_gfx.setFont(u8g2_font_logisoso24_tr);
                        u8g2_for_adafruit_gfx.drawStr(4, 28, "MATCHBOX");
                        display.display();
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
    disableLoopWDT();
    xserialReadBufferMutex = xSemaphoreCreateMutex();

    sw_secs.start();
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    u8g2_for_adafruit_gfx.begin(display); // connect u8g2 procedures to Adafruit GFX

    display.clearDisplay();
    u8g2_for_adafruit_gfx.setFont(u8g2_font_logisoso24_tr);
    u8g2_for_adafruit_gfx.drawStr(2, 28, "MATCHBOX");
    display.display();

    // Start Serial
    // Serial.setRxBufferSize(BUFFER_SIZE);
    Serial.begin(BAUDRATE);
    // Serial.begin(BAUDRATE, SERIAL_8N1, RX, TX);

    // Start BLE
    startBluetooth();

    // Start tasks
    Serial.printf("Start ReadSerialTask\n");
    xTaskCreate(ReadSerialTask, "ReadSerialTask", 1024 * 4, NULL, 1, NULL); // read serial(TC4) data ,and send to BLE

    // // Serial.printf("Start ReadBtTask\n");
    // xTaskCreate(ReadBtTask, "ReadBtTask", 1024 * 4, NULL, 1, NULL); // read BLE cmnd

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
    // Serial.println("HTTP server started");
}
void loop()
{
    server.handleClient();
    ElegantOTA.loop();

    // disconnecting
    if (!deviceConnected && oldDeviceConnected)
    {
        delay(500);                  // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
#if defined(DEBUG_MODE)
        Serial.println("start advertising");
#endif
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected)
    {
        // do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}
//