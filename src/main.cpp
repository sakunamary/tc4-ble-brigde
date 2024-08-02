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
#include "BluetoothSerial.h"
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"
#include <HardwareSerial.h>

#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ElegantOTA.h>

void Bluetooth_Callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param); // bluetooth callback handler

// bluetooth declare
BluetoothSerial SerialBT;

String local_IP;
String BT_EVENT;

HardwareSerial Serial_in(2); // D16 RX_drumer  D17 TX_drumer

WebServer server(80);

uint8_t unitMACAddress[6]; // Use MAC address in BT broadcast and display
char deviceName[30];       // The serial string that is broadcast.

SemaphoreHandle_t xserialReadBufferMutex = NULL; // Mutex for TC4数据输出时写入队列的数据
uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];
unsigned long ota_progress_millis = 0;

void Bluetooth_Callback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event)
    {
    case ESP_SPP_INIT_EVT:
        //Serial.println("SPP is inited");
        BT_EVENT = "SPP INITED";
        break;
    case ESP_SPP_START_EVT:
        //Serial.println("SPP server started");
        BT_EVENT = "SPP STARTED";
        break;
    case ESP_SPP_SRV_OPEN_EVT:
        //Serial.println("Client Connected");
        BT_EVENT = "Client OK";
        break;
    case ESP_SPP_CLOSE_EVT:
        //Serial.println("Client disconnected");
        BT_EVENT = "Client lost";
        break;
    case ESP_SPP_DATA_IND_EVT:
        //Serial.println("SPP  received data");
        BT_EVENT = "DATA receiving";
        break;
    case ESP_SPP_WRITE_EVT:
        //Serial.println("SPP  write data");
        BT_EVENT = "DATA writing";
        break;
    default:
        //Serial.print("Unhandle Event: ");
        //Serial.println(event);
        break;
    }
}

void onOTAStart()
{
    // Log when OTA has started
    //Serial.println("OTA update started!");
    // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final)
{
    // Log every 1 second
    if (millis() - ota_progress_millis > 1000)
    {
        ota_progress_millis = millis();
        //Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
    }
}

void onOTAEnd(bool success)
{
    // Log when OTA has finished
    if (success)
    {
        //Serial.println("OTA update finished successfully!");
    }
    else
    {
        //Serial.println("There was an error during OTA update!");
    }
    // <Add your own code here>
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

    // Initial Bluetooth Serial Port Profile (SPP)
    SerialBT.register_callback(Bluetooth_Callback);
    // Setup bluetooth device name as
    if (!SerialBT.begin(deviceName))
    {
        //Serial.println("An error occurred during initialize");
    }
    else
    {
        //Serial.println("Bluetooth is ready for pairing");
        // Use FIXED pin-code for Legacy Pairing
        char pinCode[5];
        memset(pinCode, 0, sizeof(pinCode));
        pinCode[0] = '1';
        pinCode[1] = '2';
        pinCode[2] = '3';
        pinCode[3] = '4';
        SerialBT.setPin(pinCode);
    }

    while (WiFi.status() != WL_CONNECTED)
    {

        delay(1000);
        //Serial.println("wifi not ready");

        if (tries++ > 2)
        {
            // init wifi
            //Serial.println("WiFi.mode(AP):");
            WiFi.mode(WIFI_AP);
            WiFi.softAP(deviceName, "88888888"); // defualt IP address :192.168.4.1 password min 8 digis
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
                // //Serial.println(cmd_check);
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
                sprintf(BLE_Send_out, "%s\n", serialReadBuffer_clean_OUT);
                //Serial.printf(BLE_Send_out);
                SerialBT.printf(BLE_Send_out);
                xSemaphoreGive(xserialReadBufferMutex);
            }
        }

        delay(50);
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
                //Serial.write(bleReadBuffer, count);
                xSemaphoreGive(xserialReadBufferMutex);
            }
            delay(50);
        }
    }
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

void setup()
{

    // Disable watchdog timers
    disableCore0WDT();
    disableCore1WDT();
    disableLoopWDT();
    esp_task_wdt_delete(NULL);
    rtc_wdt_protect_off();
    rtc_wdt_disable();
    //Serial.printf("Disable watchdog timers\n");

    xserialReadBufferMutex = xSemaphoreCreateMutex();
    // Start Serial
    Serial_in.setRxBufferSize(BUFFER_SIZE);
    Serial.begin(BAUDRATE);
    Serial_in.begin(BAUDRATE, SERIAL_8N1, 16, 17);

    // Start BLE
    startBluetooth();

    // Start tasks
    xTaskCreate(ReadSerialTask, "ReadSerialTask", 10240, NULL, 1, NULL);
    //Serial.printf("Start ReadSerialTask\n");
    xTaskCreate(ReadBtTask, "ReadBtTask", 10240, NULL, 1, NULL);
    //Serial.printf("Start ReadBtTask\n");
    // xTaskCreate(TASK_Send_READ_CMDtoTC4, "Send_READ_Task", 10240, NULL, 1, NULL);
    // //Serial.printf("Start Send_READ_Task\n");

    server.on("/", handle_root);
    ElegantOTA.begin(&server); // Start ElegantOTA
    // ElegantOTA callbacks
    ElegantOTA.onStart(onOTAStart);
    ElegantOTA.onProgress(onOTAProgress);
    ElegantOTA.onEnd(onOTAEnd);

    server.begin();
    //Serial.println("HTTP server started");
}
void loop()
{
    server.handleClient();
    ElegantOTA.loop();
}
