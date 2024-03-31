#ifndef HTML_CONTENT_H
#define HTML_CONTENT_H

const char* htmlContent = R"=====(
  <html>
  <head>
  <style>
  body { background-color: #f2f2f2; text-align: center; font-family: Arial, sans-serif;}
  h1 { color: #333; }
  input[type='text'] { padding: 10px; font-size: 16px; border: none; border-radius: 4px; box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);}
  input[type='submit'] { padding: 10px 20px; font-size: 16px; background-color: #4CAF50; color: white; border: none; cursor: pointer; border: none; border-radius: 4px; box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);}
  button[type='button'] { padding: 10px 20px; font-size: 16px; background-color: #4CAF50; color: white; border: none; cursor: pointer; border: none; border-radius: 4px; box-shadow: 0 2px 4px rgba(0, 0, 0, 0.1);}
  button[type='button']:hover,
  input[type='submit']:hover {
  background-color: #45a049;
  }
  </style>
  </head>
  <body>
  <h1>Welcome to the FC2.1 WiFi terminal!</h1>
  <form method='get'> <!-- <form method='post' action='/process_input'> -->
  Input: <input type='text' name='user_input'><br><br>
  <input type='submit' value='Submit'>
  <h1> </h1>
  <button type='button' onclick='reboot()'>Reboot</button>
  <button type='button' onclick='rebootESP()'>Reboot ESP</button>
  <button type='button' onclick='countdown()'>Countdown</button>
  <button type='button' onclick='wipeSD()'>Wipe MicroSD Card</button>
  <button type='button' onclick='wipeFlash()'>Wipe QSPI Flash</button>
  <h1> </h1>
  <button type='button' onclick='servox()'>Wiggle X axis TVC servo</button>
  <button type='button' onclick='servoy()'>Wiggle Y axis TVC servo</button>
  <h1>PYRO CHANNELS: BE VERY CAREFUL</h1>
  <button type='button' onclick='pyro1()'>Fire pyro channel 1</button>
  <button type='button' onclick='pyro2()'>Fire pyro channel 2</button>
  <button type='button' onclick='pyro3()'>Fire pyro channel 3</button>
  <h1> </h1>
  <button type='button' onclick='parachuteServo180()'>Actuate parachute servo to 180</button>
  <button type='button' onclick='parachuteServo0()'>Home parachute servo</button>
  <h1> </h1>
  <div id="runMode"></div>
  <!-- <div id="serialData"></div> -->
  </form>
  <script>
  function runMode(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET','/runMode');
    xhr.onreadystatechange = function() {
      if (xhr.readyState === 4 && xhr.status === 200) {
        document.getElementById("runMode").innerHTML = xhr.responseText;
      }
    };
    xhr.send();
  }
  function servox(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/servox', true);
    xhr.send();
  }
  function servoy(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/servoy', true);
    xhr.send();
  }
  function parachuteServo180(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/parachuteServo180', true);
    xhr.send();
  }
  function parachuteServo0(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/parachuteServo0', true);
    xhr.send();
  }
  function pyro1(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/pyro1', true);
    xhr.send();
  }
  function pyro2(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/pyro2', true);
    xhr.send();
  }
  function pyro3(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/pyro3', true);
    xhr.send();
  }
  function wipeFlash(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/wipeFlash', true);
    xhr.send();
  }
  function wipeSD(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/wipeSD', true);
    xhr.send();
  }
  function countdown(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/countdown', true);
    xhr.send();
  }
  function reboot(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/reboot', true);
    xhr.send();
  }
  function rebootESP(){
    var xhr = new XMLHttpRequest();
    xhr.open('GET', '/rebootESP', true);
    xhr.send();
  }
  function updateSerialData() {
    var serialDataElement = document.getElementById("serialData");
    var xhr = new XMLHttpRequest();

    xhr.onreadystatechange = function () {
      if (xhr.readyState === 4 && xhr.status === 200) {
        var serialData = xhr.responseText;
        serialDataElement.innerHTML = serialData;
      }
    };

    xhr.open("GET", "/getSerialData", true);
    xhr.send();
  }
  setInterval(updateSerialData, 1000); // Update every 1 second
  function isMobileDevice() {
    return /Android|webOS|iPhone|iPad|iPod|BlackBerry|IEMobile|Opera Mini/i.test(navigator.userAgent);
  }
  </script>
  </body>
  </html>
)=====";

#endif
