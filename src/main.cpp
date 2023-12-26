#include <Arduino.h>
#include "config.h"
#include "EEPROM.h"
#include <BleSerial.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
//#include "soc/rtc_wdt.h"


#include <HardwareSerial.h>

#include <StringTokenizer.h>


#include "ArduinoJson.h"

  #include <WiFi.h>
  #include <WiFiClient.h>
  #include <WebServer.h>
#include <ElegantOTA.h>

//SoftwareSerial Serial_in ;
//spSoftwareSerial::UART Serial_in;// D16 RX_drumer  D17 TX_drumer 
 HardwareSerial Serial_in(2);
SemaphoreHandle_t xThermoDataMutex = NULL;

  WebServer server(80);

//AsyncWebSocket ws("/websocket"); // access at ws://[esp ip]/

char ap_name[30] ;
uint8_t macAddr[6];

String local_IP;
String MsgString;

const int BUFFER_SIZE = 1024;

BleSerial SerialBT;


uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];



//user_wifi_t user_wifi = {" ", " ", false};
//data_to_artisan_t To_artisan = {1.0,2.0,3.0,4.0};


//TaskHandle_t xHandle_indicator;

String IpAddressToString(const IPAddress &ipAddress);      



String IpAddressToString(const IPAddress &ipAddress)
{
    return String(ipAddress[0]) + String(".") +
           String(ipAddress[1]) + String(".") +
           String(ipAddress[2]) + String(".") +
           String(ipAddress[3]);
}



String processor(const String &var)
{
    Serial.println(var);
  if (var == "version")
    {
        return VERSION;
    }
    
    return String();
}



//Task for reading Serial Port  模块发送 READ 指令后，读取Serial的数据 ，并写入数组
void TASK_ReadSerial(void *pvParameters) {
  while (true) {
    if (Serial_in.available()) {
      auto count = Serial.readBytes(serialReadBuffer, BUFFER_SIZE);
      SerialBT.write(serialReadBuffer, count);
        Serial.write(serialReadBuffer, count);
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
     Serial.write(bleReadBuffer, count);
    }
    delay(20);
  }
}


unsigned long ota_progress_millis = 0;

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void setup() {

    xThermoDataMutex = xSemaphoreCreateMutex();

  //Disable watchdog timers
  disableCore0WDT();
  //disableCore1WDT();
  disableLoopWDT();
  esp_task_wdt_delete(NULL);
  //rtc_wdt_protect_off();
 //rtc_wdt_disable();


    Serial.begin(BAUDRATE);

    Serial.printf("\nWIFI  STARTING...\n");
  //初始化网络服务

            WiFi.macAddress(macAddr); 
            // Serial_debug.println("WiFi.mode(AP):");
            WiFi.mode(WIFI_AP);
            sprintf( ap_name ,"TC4_WIFI_%02X%02X%02X",macAddr[0],macAddr[1],macAddr[2]);
            WiFi.softAP(ap_name, "12345678"); // defualt IP address :192.168.4.1 password min 8 digis

 
    Serial.print("TC_WIFI's IP:");

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


    //Serial_in.begin(BAUDRATE,EspSoftwareSerial::SWSERIAL_8N1,10,9); //RX  TX
    Serial_in.begin(BAUDRATE, SERIAL_8N1, RX, TX);

    while (!Serial_in)
    {
        ; // wait for serial port ready
    }


    Serial.printf("\nSerial_in setup OK\n");



  //Init BLE Serial
  SerialBT.begin(ap_name,true,13);
  SerialBT.setTimeout(10);
    while (!SerialBT.connected())
    {
        ; // wait for serial port ready
    }

     Serial.printf("\nSerial_BT setup OK\n");



Serial.printf("\nStart Task...\n");
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



    Serial.printf("\nTASK1:ReadSerial...\n");

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



    Serial.printf("\nTASK=2:ReadBtTask...\n");


  server.on("/", []() {
    server.send(200, "text/plain", "Hi! This is ElegantOTA Demo.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP OTA server started");


}

void loop() {
  //vTaskDelete(NULL);
 server.handleClient();
  ElegantOTA.loop();
}