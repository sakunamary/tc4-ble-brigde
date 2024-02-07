
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define uS_TO_S_FACTOR 1000000 /* Conversion factor for micro seconds to seconds */
#define BAUDRATE 115200        // serial port baudrate

#define VERSION "1.0.9"

#define DEBUG_MODE
#define TX 17
#define RX 16



const char index_html[] PROGMEM = R"rawliteral(
<!doctype html><html lang='cn'>
<head>
<script>
  function submitMessage() {
    alert("VALUE SAVED  数据已保存");
    setTimeout(function(){ document.location.reload(false); }, 500);
  }
</script>
    <meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
    <title>TC4-WB Setup</title>
    <style>*,::after,::before{box-sizing:border-box;}
    body{margin:0;font-family:'Segoe UI',Roboto,'Helvetica Neue',Arial,'Noto Sans','Liberation Sans';
    font-size:1rem;
    font-weight:400;
    line-height:1.5;color:#212529;
    background-color:#f5f5f5;}
    .form-control{
    display:block;
    width: 400px;
    height:calc(1.5em + .75rem + 2px);
    border:1px solid #ced4da;}
    button{border:1px solid transparent;
    color:#fff;
    background-color:#007bff;
    border-color:#007bff;
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
        <form action='/get' method='get'>
            <h1 class=''>TC4-WB SETTING </h1>
            <h2 class=''>WIFI SETUP </h2>
            <div class='form-floating'>
            <label>SSID/WIFI名字</label>
            <input type='text' class='form-control' name='ssid'> 
            </div>
            <div class='form-floating'>
            <br/>
            <label>PASSWORD</label>
            <input type='password' class='form-control' name='password'>
            </div>
            <p>NOTICE:INPUT NULL will set back to AP mode <br/>
            提示:输入空白即恢复AP模式直链模式
            </p>
            <br/>
            <button type='submit'>SAVE WIFI INFO </button>
        </form>     
        <form action='/compens' method='get'>                
            <br/>
            <br/>
            <h2 class=''>THERMO COMPENSATE SETUP <br/>电偶温度补偿设置</h2>
            <div class='form-floating'>
            <label>Bean Temp/豆温 (current: %bt_compens%) </label>
            <input type='number' step = '0.01' max = '20' min='-20' class='form-control'  name='btemp_fix'> 
            </div>
            <br/>
            <div class='form-floating'>
            <label>Env  Temp/炉温 (current:%et_compens%)</label>
            <input type='number' step = '0.01' max = '20' min='-20' class='form-control' name='etemp_fix'> 
            </div>
            <br/>
            <div class='form-floating'>
            <label>Bean  Temp2/豆温2 (current:%et_compens%)</label>
            <input type='number' step = '0.01' max = '20' min='-20' class='form-control' name='bt2emp_fix'> 
            </div>
            <br/>
            <div class='form-floating'>
            <label>Env  Temp2/炉温2(current:%et_compens%)</label>
            <input type='number' step = '0.01' max = '20' min='-20' class='form-control' name=‘et2temp_fix'> 
            </div>
            <br/>
            <button type='submit'onclick="submitMessage()">SAVE</button>
        </form> 
        <form action='/other' method='get'>   
            <br/>
            <br/>
            <div class='form-floating'>
            <h2 class=''>OTHER SETTING <br/>杂项</h2>  
            <label>Sampling 采样时间 (current: %sampling_time%) s</label>
            <input type='number' step = '0.25' max = '4' min='0.75' class='form-control'  name='sampling_time'> 
            </div>
            <br/>
            <button type='submit'onclick="submitMessage()">SAVE</button>
        </form> 
            <p>
            <a href='/update' target='_blank'>FIRMWARE UPDATE verison:%version%</a>
            </p>
            <br/>
    </main> 
</body></html>
)rawliteral";


#endif