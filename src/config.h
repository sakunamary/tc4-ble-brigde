
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 9600        // serial port baudrate

//MATCH BOX MODULES

#define VERSION "1.1.6d"

//#define DEBUG_MODE
#define TX 17
#define RX 16

const int BUFFER_SIZE = 48;


static TaskHandle_t xTASK_Send_READ_CMDtoTC4_handle = NULL;
static TaskHandle_t xTASK_ReadBtTask_handle = NULL;
static TaskHandle_t xTASK_ReadSerialTask_handle = NULL;

const char index_html[] PROGMEM = R"rawliteral(

<!doctype html><html lang='cn'>
 <head>
<title>MATCH BOX SETUP</title>
</head> 
 <body>
<main>
    <h1 align='center'>BLE version:%version%</h1>
       <div align='center'><a href='/update'>FIRMWARE UPDATE</a>
        </br>
         </div>
    </body>
 </html>
)rawliteral";

#endif