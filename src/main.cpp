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

#include <StringTokenizer.h>

#include <ModbusIP_ESP8266.h>

//spSoftwareSerial::UART Serial_in;// D16 RX_drumer  D17 TX_drumer 
HardwareSerial Serial_in(2);
SemaphoreHandle_t xThermoDataMutex = NULL;

AsyncWebServer server(80);

String IpAddressToString(const IPAddress &ipAddress);                         //转换IP地址格式
String processor(const String &var); // webpage function

String local_IP;


QueueHandle_t queueCMD = xQueueCreate(5, sizeof(char[64]));
QueueHandle_t queueTC4_data = xQueueCreate(10, sizeof(char[64]));


//Modbus Registers Offsets
const uint16_t BT_HREG = 3001;
const uint16_t ET_HREG = 3002;
const uint16_t HEAT_HREG = 3003;
const uint16_t FAN_HREG = 3004;
const uint16_t SV_HREG = 3005;



char ap_name[30] ;
uint8_t macAddr[6];
double Data[6];//温度数据


const int BUFFER_SIZE = 64;

BleSerial SerialBT;
//ModbusIP object
ModbusIP mb;

uint8_t bleReadBuffer[BUFFER_SIZE];
uint8_t serialReadBuffer[BUFFER_SIZE];




//Task for reading Serial Port  模块发送 READ 指令后，读取Serial的数据 ，写入QueueTC4_data 传递给 TASK_ModbusSendTask
void TASK_ReadSerial(void *pvParameters) {

 const TickType_t timeOut = 2000;
 String TC4_data_String;
  while (true) {
    if (Serial_in.available()) {
      auto count = Serial_in.readBytes(serialReadBuffer, BUFFER_SIZE);
      SerialBT.write(serialReadBuffer, count); 
      xQueueSend(queueTC4_data, &serialReadBuffer, timeOut) ;//发送数据到Queue
      memset(serialReadBuffer,'\0',sizeof(serialReadBuffer));
     }
    vTaskDelay(20);
    }
}

//Task  for Reading BLE
void TASK_ReadBtTask(void *pvParameters) {
  while (true) {
    if (SerialBT.available()) {
      auto count = SerialBT.readBytes(bleReadBuffer, BUFFER_SIZE);
      Serial_in.write(bleReadBuffer, count);   
      memset(bleReadBuffer,'\0',sizeof(bleReadBuffer));
    }
    vTaskDelay(20);
  }
}

void TASK_ModbusSendTask(void *pvParameters) {
//接收TASK_ReadSerial 传递 QueueTC4_data 数据serialReadBuffer
//将数据serialReadBuffer 转换为字符串 
//tokenizer 将数据写入Data[]数组
//Data[]数组 赋值给Hreg

    (void)pvParameters;
   //const  TickType_t xLastWakeTime;
    const TickType_t timeOut = 1000;
    int i = 0;
    uint8_t serialReadBuffer[BUFFER_SIZE];
    String TC4_data_String;

    for (;;) // A Task shall never return or exit.
    { //for loop
        if (xQueueReceive(queueTC4_data, &serialReadBuffer, timeOut) == pdPASS) {

            TC4_data_String = String((char *)serialReadBuffer);  
    
        }             
            Serial.print(TC4_data_String); 
            Serial.println();
        if (TC4_data_String.startsWith("#")) { //

        } else {

                StringTokenizer TC4_Data(TC4_data_String, ",");

                while(TC4_Data.hasNext()){
                    Data[i]=TC4_Data.nextToken().toDouble(); // prints the next token in the string
                    i++;
                }
                   mb.Hreg(BT_HREG,Data[1]*100); //初始化赋值
                   mb.Hreg(ET_HREG,Data[2]*100); //初始化赋值
                i = 0;
                }
    vTaskDelay(20);
    
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
    Serial.printf("\nSerial Started\n");
#endif
  //初始化网络服务

            WiFi.macAddress(macAddr); 
            // Serial_debug.println("WiFi.mode(AP):");
            WiFi.mode(WIFI_AP);
            sprintf( ap_name ,"MatchBox-%02X%02X%02X",macAddr[3],macAddr[4],macAddr[5]);
            WiFi.softAP(ap_name, "12345678"); // defualt IP address :192.168.4.1 password min 8 digis

#if defined(DEBUG_MODE)
    Serial.printf("\nWiFi AP Started\n");
#endif
    /*---------- Task Definition ---------------------*/
    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TASK_ReadSerial, "ReadSerial" // 测量电池电源数据，每分钟测量一次
        ,
       4096 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL,  1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=1:ReadSerial OK\n");
#endif


    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TASK_ReadBtTask, "ReadBtTask" // 测量电池电源数据，每分钟测量一次
        ,
        4096 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL,  1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );
#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=2:ReadBtTask OK\n");
#endif


    // Setup tasks to run independently.
    xTaskCreatePinnedToCore(
        TASK_ModbusSendTask, "ModbusSendTask" // 测量电池电源数据，每分钟测量一次
        ,
        2048 // This stack size can be checked & adjusted by reading the Stack Highwater
        ,
        NULL, 1 // Priority, with 1 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
        ,
        NULL,  1 // Running Core decided by FreeRTOS,let core0 run wifi and BT
    );


#if defined(DEBUG_MODE)
    Serial.printf("\nTASK=3:ModbusSendTask OK \n");
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


//Init Modbus-TCP 
#if defined(DEBUG_MODE)
    Serial.printf("\nStart Modbus-TCP   service...\n");
#endif
    mb.server(502);		//Start Modbus IP //default port :502
    // Add SENSOR_IREG register - Use addIreg() for analog Inputs
    mb.addHreg(BT_HREG);
    mb.addHreg(ET_HREG);
    mb.addHreg(FAN_HREG);
    mb.addHreg(FAN_HREG);
    mb.addHreg(SV_HREG);

    mb.Hreg(BT_HREG,0); //初始化赋值
    mb.Hreg(ET_HREG,0);  //初始化赋值
    mb.Hreg(FAN_HREG,0); //初始化赋值
    mb.Hreg(FAN_HREG,0);//初始化赋值
    mb.Hreg(SV_HREG,0);  //初始化赋值
}





void loop() {
    mb.task();

}