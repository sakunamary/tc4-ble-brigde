
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define VERSION "1.1.1"

//#define DEBUG_MODE
#define TX 17
#define RX 16

const int BUFFER_SIZE = 128;




const char index_html[] PROGMEM = R"rawliteral(
<!doctype html><html lang='cn'>
<head>
<script>
  function submitMessage() {
    alert("数据已保存");
    setTimeout(function(){ document.location.reload(false); }, 500);
  }
</script>
    <meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
    <title>TC4-WB 设置</title>
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
    <main class='form-signin'> 
        <form action='/get' method='get'>
            <h1 class=''>TC4-WB 设置</h1>
            <h2 class=''>1.WIFI 设置</h2>
            <div class='form-floating'>
            <label>SSID/WIFI名字</label>
            <input type='text' class='form-control' name='ssid'> 
            </div>
            <div class='form-floating'>
            <br/>
            <label>PASSWORD</label>
            <input type='password' class='form-control' name='password'>
            </div>
            <p>
            提示:输入空白即恢复AP模式直链模式
            </p>
            <br/>
            <div align="center">
                <button  type='submit'>保存</button>
            </div>
            
        </form>     
        <form action='/compens' method='get'>                
            <br/>
            <br/>
            <h2 class=''>2.电偶温度补偿设置</h2>
            <div class='form-floating'>
            <label>Bean Temp/豆温 (current: %bt_compens%) </label>
            <input type='number' step = '0.01' max = '20' min='-20' class='form-control'  name='Btemp_fix'> 
            </div>
            <br/>
            <div class='form-floating'>
            <label>Env  Temp/炉温 (current:%et_compens%)</label>
            <input type='number' step = '0.01' max = '20' min='-20' class='form-control' name='Etemp_fix'> 
            </div>
            <br/>
            <div align="center">
                <button  type='submit'onclick="submitMessage()">保存</button>
            </div>
            
        </form> 
        <form action='/other' method='get'>   
            <br/>
            <br/>
            <div class='form-floating'>
            <h2 class=''>3.杂项</h2>  
            <label>温度采样时间 (current: %sampling_time%) s</label>
            <input type='number' step = '0.5' max = '4' min='1' class='form-control'  name='sampling_time'>     
            </div>
             <br/>
             <br/>
            <div align="center">
                <button type='submit'onclick="submitMessage()">保存</button>
            </div>
            <br/>
            <br/>
        </form> 
            <p>
            <a href='/update' target='_blank'>FIRMWARE UPDATE verison:%version%</a>
            </p>
            <br/>
    </main> 
</body></html>
)rawliteral";


const char update_html[] PROGMEM = R"rawliteral(
<!doctype html><html lang='cn'>
<head>
    <meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
    <title>TC4-WB Setup</title>
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
    width:400px}
    .form-signin{
    width: 400px;
    padding:15px;
    font-size:1rem;
    margin:auto;}
    h1,p{text-align:center}
    .input_control{
    color:#fff;
    background-color:rgb(122, 72, 23);
    border-color:rgb(113, 59, 18);
    padding:.5rem 1rem;
    font-size:1.25rem;
    line-height:1.5;
    border-radius:.3rem;
    align-items: center;
    width:150px   
    }
    </style> 
</head> 
<body>
    <main class='form-signin'> 
        <h1 class=''>固件升级</h1>
        <form method='POST' action='/update' enctype='multipart/form-data'>
         <div class='form-floating'>               
        <H2>选择固件文件</H2>
        <input type='file' name='update'class='form-signin'>
    </div> 
        <br/>
        <input type='submit' value='升级' class='input_control' >
   
    </form>
    </main> 
</body></html>
)rawliteral";


const char update_OK_html[] PROGMEM = R"rawliteral(
<!doctype html><html lang='cn'><head>
    <meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
        <title>固件升级成功</title>
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
            width:400px}
            .form-signin{
            width: 400px;
            padding:15px;
            margin:auto;}
            h1,p{text-align:center}</style> 
        </head> 
<body>
    <main class='form-signin'> 
        <h1>固件升级成功</h1> <br/> 
        </p>
        <p>设置成功<br />
        模块将在3秒后重启<br />
        </p>
    </main>
</body></html>
)rawliteral";


const char update_fail_html[] PROGMEM = R"rawliteral(
<!doctype html><html lang='cn'><head>
    <meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
        <title>固件升级成功</title>
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
            width:400px}
            .form-signin{
            width: 400px;
            padding:15px;
            margin:auto;}
            h1,p{text-align:center}</style> 
        </head> 
<body>
    <main class='form-signin'> 
        <h1>固件升级失败</h1> <br/> 
        </p>
        <p>模块将在3秒后重启<br />
        重启后请重新上传<br />
        </p>
    </main>
</body></html>
)rawliteral";


#endif