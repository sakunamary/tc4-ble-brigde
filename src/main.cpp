#include <Arduino.h>
#include "config.h"

#include <BleSerial.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>


#include <HardwareSerial.h>


#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>

//spSoftwareSerial::UART Serial_in;// D16 RX_drumer  D17 TX_drumer 
HardwareSerial Serial_in(2);
SemaphoreHandle_t xThermoDataMutex = NULL;

AsyncWebServer server(80);

String IpAddressToString(const IPAddress &ipAddress);                         //转换IP地址格式
String processor(const String &var); // webpage function



char ap_name[30] ;
uint8_t macAddr[6];

String local_IP;
String MsgString;

const int BUFFER_SIZE = 1024;

BleSerial SerialBT;


uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];




//Task for reading Serial Port  模块发送 READ 指令后，读取Serial的数据 ，并写入数组
void TASK_ReadSerial(void *pvParameters) {
  while (true) {
    if (Serial_in.available()) {
      auto count = Serial_in.readBytes(serialReadBuffer, BUFFER_SIZE);
      SerialBT.write(serialReadBuffer, count);
#if defined(DEBUG_MODE)      
      Serial.write(serialReadBuffer, count);
#endif
    }
    delay(20);
  }
}

//Task for reading BLE Serial
void TASK_ReadBtTask(void *epvParameters) {
  while (true) {
    if (SerialBT.available()) {
      auto count = SerialBT.readBytes(bleReadBuffer, BUFFER_SIZE);
      Serial_in.write(bleReadBuffer, count);  

#if defined(DEBUG_MODE)       
                  Serial.write(bleReadBuffer, count); 
#endif                 
    }
    delay(20);
  }
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
if (var == "version")
    {
        return VERSION;
    }

    return String();
}


void setup() {

    xThermoDataMutex = xSemaphoreCreateMutex();

  //Disable watchdog timers
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();
  esp_task_wdt_delete(NULL);
  //rtc_wdt_protect_off();
  //rtc_wdt_disable();


    Serial.begin(BAUDRATE);
    Serial_in.begin(BAUDRATE, SERIAL_8N1, RX, TX);

#if defined(DEBUG_MODE)
    Serial.printf("\nWIFI  STARTING...\n");
#endif  
  //初始化网络服务

            WiFi.macAddress(macAddr); 
            // Serial_debug.println("WiFi.mode(AP):");
            WiFi.mode(WIFI_AP);
            sprintf( ap_name ,"MatchBox-%02X%02X%02X",macAddr[3],macAddr[4],macAddr[5]);
            WiFi.softAP(ap_name, "12345678"); // defualt IP address :192.168.4.1 password min 8 digis



#if defined(DEBUG_MODE)
Serial.printf("\nStart Task...\n");
#endif
    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TASK_ReadSerial, "ReadSerial" // 测量电池电源数据，每分钟测量一次
        ,
        1024 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL,  1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );


#if defined(DEBUG_MODE)
    Serial.printf("\nTASK1:ReadSerial...\n");
#endif

    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TASK_ReadBtTask, "ReadBtTask" // 测量电池电源数据，每分钟测量一次
        ,
        1024 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL,  1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );


#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=2:ReadBtTask...\n");
#endif

    // for index.html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send_P(200, "text/html", index_html, processor); });

  AsyncElegantOTA.begin(&server);    // Start AsyncElegantOTA
  server.begin();

#if defined(DEBUG_MODE)
    Serial.println("HTTP server started");
#endif

  //Init BLE Serial
  SerialBT.begin(ap_name,true,2);
  SerialBT.setTimeout(10);
#if defined(DEBUG_MODE)
     Serial.printf("\nSerial_BT setup OK\n");
#endif

}

void loop() {
Serial_in.print("READ;\n");
delay(1000);
}