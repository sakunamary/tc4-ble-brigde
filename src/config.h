
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define VERSION "1.1.3"

// #define DEBUG_MODE
#define TX 17
#define RX 16

const int BUFFER_SIZE = 128;

const char index_html[] PROGMEM = R"rawliteral(
<!doctype html><html lang='cn'>
<head>
    <meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
    <title>MATCH BOX UPDATE </title>
    <style>*,::after,::before{box-sizing:border-box;}
    body{margin:0;font-family:'Segoe UI',Roboto,'Helvetica Neue',Arial,'Noto Sans','Liberation Sans';
    font-size:1rem;
    font-weight:400;
    line-height:1.5;color:rgb(113, 58, 19);
    background-color:rgb(252, 231, 174);}
    .form-control{
    display:block;
    width: 400px;
    height:calc(1.5em + .75rem + 2px);
    border:1px solid #ced4da;}
    button{border:1px solid transparent;
    color:#fff;
    background-color:rgb(122, 72, 23);
    border-color:rgb(113, 59, 18);
    padding:.5rem 1rem;
    font-size:1.25rem;
    line-height:1.5;
    border-radius:.3rem;
    width:150px;
   }
    .form-signin{
    width: 400px;
    padding:15px;
    margin:auto;
    }
    h1,p{text-align:center}
    </style> 
</head> 
<body>
            <p>
            <a href='/update'>FIRMWARE UPDATE verison:%version%</a>
            </p>
</body></html>
)rawliteral";

#endif