
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 9600        // serial port baudrate

#define VERSION "1.1.4"

// #define DEBUG_MODE
#define TX 17
#define RX 16

const int BUFFER_SIZE = 64;


const char index_html[] PROGMEM = R"rawliteral(

<!doctype html><html lang='cn'>
 <head>
<title>MATCH BOX MINI SETUP</title>
</head> 
 <body>
<main>
    <h1 align='center'>BLE version:%version%</h1>
       <div align='center'><a href='/update' target='_blank'>FIRMWARE UPDATE</a>
    </body>
 </html>
)rawliteral";



#endif