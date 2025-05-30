#pragma once
#include <Arduino.h>
#include <WebServer.h>

// Web Server Handlers
void handleRoot();
void handleSet();
void handleValues();

// HTML Content (PROGMEM for flash storage)
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>VentCon Pressure Control</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>

    body 
    {
      font-family: Arial, sans-serif;
      margin: 20px;
      background: #f0f0f0;
      max-width: 700px;
      margin-left: auto;
      margin-right: auto;
    }

    h1 
    {
      text-align: center;
    }

    .control-group 
    {
      background: #fff;
      margin: 20px 0;
      padding: 20px;
      border-radius: 8px;
      box-shadow: 0 2px 4px rgba(0,0,0,0.08);
    }

    .control-row 
    {
      display: flex;
      align-items: center;
      gap: 15px;
      margin: 15px 0;
    }

    .control-row label 
    {
      min-width: 140px;
    }

    input[type="range"] 
    {
      flex: 1 1 200px;
      margin-right: 10px;
    }

    input[type="number"] 
    {
      width: 70px;
      padding: 4px;
      border-radius: 4px;
      border: 1px solid #ccc;
    }

    .ap-notice 
    {
      background: #ffeeba;
      padding: 15px;
      border-radius: 8px;
      margin-bottom: 20px;
      text-align: center;
    }

  </style>
</head>

<body>

  <div class="ap-notice">
    <h3>Connected to VENTCON_AP</h3>
    <p>IP: 192.168.4.1 | No internet access</p>
  </div>

  <h1>Pressure Control System</h1>

  <div class="control-group">

    <h2>Control Parameters</h2>

    <div class="control-row">
      <label for="sp_slider">Pressure Setpoint (bar):</label>
      <input type="range" id="sp_slider" min="0" max="10" step="0.1" value="%SP%">
      <input type="number" id="sp_text" value="%SP%" step="0.1">
    </div>

    <div class="control-row">
      <label for="kp_slider">Proportional (Kp):</label>
      <input type="range" id="kp_slider" min="0" max="5" step="0.1" value="%KP%">
      <input type="number" id="kp_text" value="%KP%" step="0.1">
    </div>

    <div class="control-row">
      <label for="ki_slider">Integral (Ki):</label>
      <input type="range" id="ki_slider" min="0" max="2" step="0.01" value="%KI%">
      <input type="number" id="ki_text" value="%KI%" step="0.01">
    </div>

    <div class="control-row">
      <label for="kd_slider">Derivative (Kd):</label>
      <input type="range" id="kd_slider" min="0" max="1" step="0.01" value="%KD%">
      <input type="number" id="kd_text" value="%KD%" step="0.01">
    </div>

    <div class="control-row">
      <label for="flt_slider">Filter (0-1):</label>
      <input type="range" id="flt_slider" min="0" max="1" step="0.01" value="%FLT%">
      <input type="number" id="flt_text" value="%FLT%" step="0.01">
    </div>

    <div class="control-row">
      <label for="freq_slider">PWM Freq (Hz):</label>
      <input type="range" id="freq_slider" min="100" max="10000" step="100" value="%FREQ%">
      <input type="number" id="freq_text" value="%FREQ%" step="100">
    </div>

    <div class="control-row">
      <label for="res_slider">PWM Res (bits):</label>
      <input type="range" id="res_slider" min="1" max="16" step="1" value="%RES%">
      <input type="number" id="res_text" value="%RES%" step="1">
    </div>

    <div class="control-row" style="justify-content: flex-end;">
      <button id="applyBtn" disabled style="padding:8px 24px;font-size:1em;">Apply</button>
    </div>

  </div>

  <div class="control-group">
    <h2>Status</h2>
    <div class="control-row">
      <label>Pressure (bar):</label>
      <span id="pressure">--</span>
      <progress id="pressure_bar" value="0" max="10" style="width:180px;height:18px;margin-left:10px;"></progress>
    </div>
    <div class="control-row">
      <label>PWM Output (%):</label>
      <span id="pwm">--</span>
    </div>
    <div class="control-row">
      <label>ADC status:</label>
      <span id="adc_status">--</span>
    </div>
  </div>

  <script>
    // Synchronize range and number inputs, enable Apply button on change
    [
      "sp", "kp", "ki", "kd", "flt", "freq", "res"
    ].forEach(function(param) 
    {
      const slider = document.getElementById(param + "_slider");
      const text = document.getElementById(param + "_text");
      slider.addEventListener('input', function() 
      {
        text.value = slider.value;
        enableApply();
      });

      text.addEventListener('input', function() 
      {
        slider.value = text.value;
        enableApply();
      });

    });

    function enableApply() 
    {
      document.getElementById('applyBtn').disabled = false;
    }

    document.getElementById('applyBtn').addEventListener('click', function() 
    {
      // params is a string of the form "sp=1&kp=2&ki=3&kd=4&flt=5&freq=6&res=7"
      // where each value is taken from the corresponding text input
      // and encoded for URL
      // map each parameter to its value
      // and join them with "&"
      /*
        The arrow function syntax (=>) is a concise way to write functions in JavaScript.
        Example:
          (param) => { ... }
        In this code, it's used to define a function that takes 'param' and returns a string.
      */
      // The map function creates a new array by applying the function to each element of the array.  
      // The join function combines the elements of the array into a single string, separated by "&".
      // The encodeURIComponent function encodes special characters in the string to make it safe for URLs.
      // The final result is a string that can be used in a URL query string.

      const params = ["sp", "kp", "ki", "kd", "flt", "freq", "res"].map(param => 
      {
        const value = document.getElementById(param + "_text").value;
        return param + "=" + encodeURIComponent(value);
      }).join("&");

      // Send the parameters to the server
      // The "server" refers to the device or computer that hosts the backend application handling requests from this web page.
      // In this context, it processes the "/set" endpoint to update control parameters and responds to "/values" for status updates.
      // In this context, the server refers to the ESP32 device running a web server that handles requests from this web page.
      
      // The fetch function is used to make a network request to the server.
      // The URL is constructed by appending the parameters to the "/set" endpoint.
      // The then function is used to handle the response from the server.
      // If the request is successful, the Apply button is disabled to indicate that the settings have been applied.
      // If the request fails, an alert is shown to the user.
      // The catch function is used to handle any errors that occur during the request.
      // The disabled property of the Apply button is set to true to indicate that the settings have been applied.
      // The alert function is used to show an error message to the user.

      fetch("/set?" + params).then(() => 
      {
          document.getElementById('applyBtn').disabled = true;
      }).catch(err => 
      {
          alert("Failed to apply settings to server: " + err);
      });
    });

    // Periodically update status and UI controls
    setInterval(() => 
    {
      fetch('/values')
        .then(r => r.json())
        .then(data => 
        {
          document.getElementById('pressure').textContent = data.pressure !== undefined ? data.pressure.toFixed(2) : "--";
          // Update progress bar for pressure
          if (typeof data.pressure !== "undefined") 
          {
            document.getElementById('pressure_bar').value = data.pressure;
          }
          else
          {
            document.getElementById('pressure_bar').value = 0;
          }
          document.getElementById('pwm').textContent = data.pwm !== undefined ? data.pwm.toFixed(1) : "--";
          if (data.adc_status === "100") 
          {
            document.getElementById('adc_status').textContent = "external ADS1015 ADC";
          } 
          else if(data.adc_status === "000")
          {
            document.getElementById('adc_status').textContent = "internal ESP 32 ADC";
          }
          // Only update UI controls if Apply button is disabled (no unsaved changes)
          if (document.getElementById('applyBtn').disabled) 
          {

            if (typeof data.sp !== "undefined") 
            {
              document.getElementById('sp_slider').value = data.sp;
              document.getElementById('sp_text').value = data.sp;
            }

            if (typeof data.kp !== "undefined") 
            {
              document.getElementById('kp_slider').value = data.kp;
              document.getElementById('kp_text').value = data.kp;
            }

            if (typeof data.ki !== "undefined") 
            {
              document.getElementById('ki_slider').value = data.ki;
              document.getElementById('ki_text').value = data.ki;
            }

            if (typeof data.kd !== "undefined") 
            {
              document.getElementById('kd_slider').value = data.kd;
              document.getElementById('kd_text').value = data.kd;
            }

            if (typeof data.flt !== "undefined") 
            {
              document.getElementById('flt_slider').value = data.flt;
              document.getElementById('flt_text').value = data.flt;
            }

            if (typeof data.freq !== "undefined") 
            {
              document.getElementById('freq_slider').value = data.freq;
              document.getElementById('freq_text').value = data.freq;
            }

            if (typeof data.res !== "undefined") 
            {
              document.getElementById('res_slider').value = data.res;
              document.getElementById('res_text').value = data.res;
            }

          }
        });
    }, 300);
  </script>
</body>
</html>
)rawliteral";