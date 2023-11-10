#include <Arduino.h>
#include "config.h"
#include "EEPROM.h"
#include <WiFi.h>
#include <AsyncTCP.h>
#include <AsyncUDP.h>
#include <ESPAsyncWebServer.h>
#include "Update.h"
#include <BleSerial.h>
#include <esp_attr.h>
#include <esp_task_wdt.h>
#include <driver/rtc_io.h>
#include "soc/rtc_wdt.h"


#include <HardwareSerial.h>

#include <StringTokenizer.h>


#include "ArduinoJson.h"


//SoftwareSerial Serial_in ;
//spSoftwareSerial::UART Serial_in;// D16 RX_drumer  D17 TX_drumer 
 HardwareSerial Serial_in(2);
SemaphoreHandle_t xThermoDataMutex = NULL;

AsyncWebServer server(80);

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

void notFound(AsyncWebServerRequest *request);    
//void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len);//Handle WebSocket event
void onUpload(AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){}
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
/*
void onEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){

     //    {"command": "getData", "id": 93609, "roasterID": 0}
    // Artisan schickt Anfrage als TXT
    // TXT zu JSON lt. https://forum.arduino.cc/t/assistance-parsing-and-reading-json-array-payload-websockets-solved/667917

    TickType_t xLastWakeTime;

    const TickType_t xIntervel = 1000/ portTICK_PERIOD_MS;

    const size_t capacity = JSON_OBJECT_SIZE(3) + 60; // Memory pool
    DynamicJsonDocument doc(capacity);

    switch (type)
    {
    case WS_EVT_DISCONNECT:
        Serial.printf("ws[%s][%u] disconnect: %u\n", server->url(), client->id());
        break;
    case WS_EVT_CONNECT:
        //client connected
         Serial.printf("ws[%s][%u] connect\n", server->url(), client->id());
         client->printf("Hello Client %u :", client->id());
         client->ping();
        break;
    case WS_EVT_ERROR:
        //error was received from the other end
         Serial.printf("ws[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t*)arg), (char*)data);
        break;
    case WS_EVT_PONG:
        //pong message was received (in response to a ping request maybe)
        Serial.printf("ws[%s][%u] pong[%u]: %s\n", server->url(), client->id(), len, (len)?(char*)data:"");   
        break;   
    case WS_EVT_DATA:
        AwsFrameInfo * info = (AwsFrameInfo*)arg;
       if(info->final && info->index == 0 && info->len == len){

         Serial.printf("ws[%s][%u] %s-message[%llu]: ",server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

            if(info->opcode == WS_TEXT){
                    // Extract Values lt. https://arduinojson.org/v6/example/http-client/
                    // Artisan Anleitung: https://artisan-scope.org/devices/websockets/
                    deserializeJson(doc, (char *)data);
                    // char* entspricht String
                    String command = doc["command"].as<  const char *>();
                    // Serial_debug.printf("Command received: %s \n",command);
                    long ln_id = doc["id"].as<long>();
                    // Send Values to Artisan over Websocket
                    JsonObject root = doc.to<JsonObject>();
                    JsonObject data = root.createNestedObject("data");

                if (xSemaphoreTake(xThermoDataMutex, xIntervel) == pdPASS) {//给温度数组的最后一个数值写入数据   

                    if (command == "getBT")
                    {
                        root["id"] = ln_id;
                        data["BT"] = To_artisan.BT;
                    }
                    else if (command == "getET")
                    {
                        root["id"] = ln_id;
                        data["ET"] = To_artisan.ET;
                    }
                    else if (command == "getData")
                    {
                        root["id"] = ln_id;
                        data["BT"] = To_artisan.BT;
                        data["ET"] = To_artisan.ET;
                        data["AP"] = To_artisan.AP;
                        data["inlet"] = To_artisan.inlet;                         
                    }

                    xSemaphoreGive(xThermoDataMutex);  //end of lock mutex
                } 


                    char buffer[200];                        // create temp buffer 200
                    size_t len = serializeJson(doc, buffer); // serialize to buffer

                    Serial.println(buffer);
                    client->text(buffer);
                }
            }   
    break;
    }
}

*/

void notFound(AsyncWebServerRequest *request)
{
    request->send(404, "text/plain", "Opps....Not found");
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



void setup() {

    xThermoDataMutex = xSemaphoreCreateMutex();

  //Disable watchdog timers
  disableCore0WDT();
  disableCore1WDT();
  disableLoopWDT();
  esp_task_wdt_delete(NULL);
  rtc_wdt_protect_off();
  rtc_wdt_disable();


    Serial.begin(BAUDRATE);

    Serial.printf("\nWIFI  STARTING...\n");
  //初始化网络服务

            WiFi.macAddress(macAddr); 
            // Serial_debug.println("WiFi.mode(AP):");
            WiFi.mode(WIFI_AP);
            sprintf( ap_name ,"TC4_WIFI_%02X%02X%02X",macAddr[0],macAddr[1],macAddr[2]);
            WiFi.softAP(ap_name, "12345678"); // defualt IP address :192.168.4.1 password min 8 digis

 

   // Serial.printf("\nRead data from EEPROM...\n");
    // set up eeprom data
     //EEPROM.begin(sizeof(user_wifi));
    //EEPROM.get(0, user_wifi);




 //user_wifi.Init_mode = true ;
/*
if (user_wifi.Init_mode) 
{
    strcat(user_wifi.ssid,"TC4_WIFI");
    strcat(user_wifi.password,"12345678");
    user_wifi.Init_mode = false ;
    EEPROM.put(0, user_wifi);
    EEPROM.commit();
}
*/
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




    // init websocket
    //Serial.println("WebSocket started!");
    // attach AsyncWebSocket
    //ws.onEvent(onEvent);
    //server.addHandler(&ws);



    // for index.html
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send_P(200, "text/html", index_html, processor); });

    // get the value from index.html
   // server.on("/get", HTTP_GET, [](AsyncWebServerRequest *request)
                  {
//get value form webpage      
   // strncpy(user_wifi.ssid,request->getParam("ssid")->value().c_str(), sizeof(user_wifi.ssid) );
   // strncpy(user_wifi.password,request->getParam("password")->value().c_str(), sizeof(user_wifi.password) );
   // user_wifi.ssid[request->getParam("ssid")->value().length()] = user_wifi.password[request->getParam("password")->value().length()] = '\0';  
//Svae EEPROM 
    //EEPROM.put(0, user_wifi);
   // EEPROM.commit();
//output wifi_sussce html;
   // request->send_P(200, "text/html", wifi_sussce_html); });


  // upload a file to /upload
  server.on("/upload", HTTP_POST, [](AsyncWebServerRequest *request){
    request->send(200);
  }, onUpload);
       // Simple Firmware Update Form
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
                    {
                    request->send(200, "text/html", update_html);
                    });

  server.on("/update", HTTP_POST, [](AsyncWebServerRequest *request){
                        //shouldReboot = !Update.hasError();
                        AsyncWebServerResponse *response = request->beginResponse(200, "text/plain", (Update.hasError())?update_fail_html:update_OK_html);
                        response->addHeader("Connection", "close");
                        request->send(response);
                        },[](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final){
                        if(!index){
                        //vTaskSuspend(xHandle_indicator); //停止显示
                        Serial.printf("Update Start: %s\n", filename.c_str());

                        if(!Update.begin((ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000)){
                            Update.printError(Serial);
                        }
                        }
                        if(!Update.hasError()){
                        if(Update.write(data, len) != len){
                            Update.printError(Serial);
                        }
                        }
                        if(final){
                        if(Update.end(true)){
                            Serial.printf("Update Success: %uB\n", index+len);
                            Serial.printf("ESP32 will reboot after 3s \n");
                            vTaskDelay(3000);
                            ESP.restart();

                        } else {
                            Update.printError(Serial);
                            Serial.printf("ESP32 will reboot after 3s \n");
                            vTaskDelay(3000);
                            ESP.restart();
                        }
                        }
  });         

    server.onNotFound(notFound); // 404 page seems not necessary...
    server.onFileUpload(onUpload);

    server.begin();
    Serial.println("HTTP server started");


                  }


}

void loop() {
  //vTaskDelete(NULL);
}