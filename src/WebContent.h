#pragma once
#include <Arduino.h>
#include <WebServer.h>
#include "styles.h" // Include the CSS styles

// Web Server Handlers
void handleRoot();
void handleSet();
void handleValues();

// HTML Content (PROGMEM for flash storage)
const char HTML_CONTENT[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <title>VentCon Pressure Control</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <!-- Load JS libraries from LittleFS root with deferred loading -->
  <script src="/chart.min.js" defer></script>
  <script src="/moment.min.js" defer></script>
  <script src="/chartjs-adapter-moment.min.js" defer></script>

)rawliteral";

// Concatenate parts of the HTML
const char HTML_CONTENT_AFTER_STYLE[] PROGMEM = R"rawliteral(

</head>

<body>
  <div class="loader" id="loader" style="position:fixed;top:0;left:0;width:100vw;height:100vh;z-index:9999;background:#fff;display:flex;align-items:center;justify-content:center;">
    <div style="display:flex;flex-direction:column;align-items:center;">
      <img src="/Logo.svg" alt="VENTREX Logo" style="height:30px;margin-bottom:16px;">
      <div style="border:6px solid #f3f3f3;border-top:6px solid #2563eb;border-radius:50%;width:48px;height:48px;animation:spin 1s linear infinite;"></div>
      <span style="margin-top:12px;color:#2563eb;font-weight:600;">Loading...</span>
    </div>
  </div>

  <header>
    <div class="logo-container" style="margin-bottom: 0;">
      <img src="/Logo.svg" alt="VENTREX Logo" class="logo" id="ventrexLogo">
    </div>
    <h1 style="margin-top: 0; margin-bottom: 10px;">
      <span style="background: linear-gradient(90deg, #002f87 0%,  #32c09d 100%);
           -webkit-background-clip: text;
           background-clip: text;
           color: transparent;
           display: inline-block;
           font-size: 0.75em;
           font-weight: 1000;"
           aria-label="VENTCON Pressure Control System">
        VENTCON Pressure Control System
      </span>
    </h1>
  </header>

  <main>
    <section class="card">
      <h2 style="text-align: center;">Real Time Monitoring</h2>
      
      <div class="gauge-container">
        <div class="gauge">
          <div class="gauge-title">Outlet Pressure</div>
          <div class="gauge-value">
            <span id="pressure">--</span>
            <small>bar(g)</small>
            <span id="pressure-trend" class="trend-indicator trend-stable">▲</span>
          </div>
          <div class="gauge-bar">
            <div id="pressure-fill" class="gauge-fill" style="width:0%"></div>
            <div id="pressure-target" class="gauge-target" style="left:30%"></div>
          </div>
        </div>
        
        <div class="gauge">
          <div class="gauge-title">PWM Output</div>
          <div class="gauge-value">
            <span id="pwm" style="color: #10b981;">--</span>
            <small>%</small>
            <span id="pwm-trend" class="trend-indicator trend-stable">▲</span>
          </div>
          <div class="gauge-bar">
            <div id="pwm-fill" class="gauge-fill" style="width:0%"></div>
          </div>
        </div>
      </div>
      
      <div class="chart-header">
        <h3>Live Chart</h3>
        <div class="chart-toggle">
          <label for="chartToggle" class="toggle-label">Show Chart</label>
          <input type="checkbox" id="chartToggle" checked>
        </div>
      </div>
      
      <div class="chart-container" id="chartContainer">
        <canvas id="pressureChart"></canvas>
      </div>
    </section>   
   
    <section class="card">
      <div style="display: flex; justify-content: space-between; align-items: center; margin-bottom: 0.75rem;">
        <h2 style="margin-bottom: 0;">Set Parameters</h2>
      </div>
      
      <!-- Snackbar Apply Changes button (hidden by default) -->
      <div id="saveSnackbar" style="position:fixed; bottom:24px; left:50%; transform:translateX(-50%); background:#2563eb; color:white; padding:12px 24px; border-radius:24px; box-shadow:0 4px 12px rgba(0,0,0,0.15); font-weight:600; z-index:1000; display:none; cursor:pointer; transition:all 0.3s ease;">
        <span id="saveSnackbarText">Apply Changes</span>
      </div>
      
      <div class="control-row">
        <label for="sp_slider">Setpoint Outlet Pressure in bar(g)</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="sp_decrement">-</button>
          <input type="range" id="sp_slider" min="0" max="10" step="0.1" value="%SP%">
          <button type="button" class="slider-btn increment-btn" id="sp_increment">+</button>
          <input type="number" id="sp_text" value="%SP%" step="0.1">
        </div>
      </div>    
      
      <hr>
      
      <div class="section-title" style="display: flex; align-items: center; justify-content: space-between;margin-top: 1.0rem;">
        <span>PID</span>
        <button id="resetPidBtn" class="reset-btn" title="Re-initialize PID with current settings" style="padding: 8px 12px; font-size: 12px; min-width: auto;">Reset PID</button>
      </div>

      <div class="control-row">
        <label for="kp_slider">Proportional</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="kp_decrement">-</button>
          <input type="range" id="kp_slider" min="0" max="3000" step="1" value="%KP%">
          <button type="button" class="slider-btn increment-btn" id="kp_increment">+</button>
          <input type="number" id="kp_text" value="%KP%" step="1">
        </div>
      </div>

      <div class="control-row">
        <label for="ki_slider">Integral</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="ki_decrement">-</button>
          <input type="range" id="ki_slider" min="0" max="5000" step="1" value="%KI%">
          <button type="button" class="slider-btn increment-btn" id="ki_increment">+</button>
          <input type="number" id="ki_text" value="%KI%" step="1">
        </div>
      </div>

      <div class="control-row">
        <label for="kd_slider">Derivative</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="kd_decrement">-</button>
          <input type="range" id="kd_slider" min="0" max="1000" step="1" value="%KD%">
          <button type="button" class="slider-btn increment-btn" id="kd_increment">+</button>
          <input type="number" id="kd_text" value="%KD%" step="1">
        </div>
      </div>

      <div class="section-title" style="display: flex; align-items: center; justify-content: space-between;margin-top: 1.75rem;">
        <span>System</span>
        <div></div>
      </div>

      <div class="control-row">
        <label for="flt_slider">Low Pass Filter Strength on Pressure Sensor (α)</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="flt_decrement">-</button>
          <input type="range" id="flt_slider" min="0" max="1" step="0.01" value="%FLT%">
          <button type="button" class="slider-btn increment-btn" id="flt_increment">+</button>
          <input type="number" id="flt_text" value="%FLT%" step="0.01">
        </div>
      </div>

      <div class="control-row">
        <label for="freq_slider">PWM Frequency in Hz</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="freq_decrement">-</button>
          <input type="range" id="freq_slider" min="100" max="10000" step="100" value="%FREQ%">
          <button type="button" class="slider-btn increment-btn" id="freq_increment">+</button>
          <input type="number" id="freq_text" value="%FREQ%" step="100">
        </div>
      </div>

      <div class="control-row">
        <label for="res_slider">PWM Resolution in bits</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="res_decrement">-</button>
          <input type="range" id="res_slider" min="8" max="16" step="1" value="%RES%">
          <button type="button" class="slider-btn increment-btn" id="res_increment">+</button>
          <input type="number" id="res_text" value="%RES%" step="1">
        </div>
      </div>

    </section>

    <section class="card">
      <h2>System Information</h2>
      <div class="control-row">
        <label>ADC Status</label>
        <div class="hardware-info">
          <span class="status-indicator" id="adc_indicator"></span>
          <span id="adc_status" class="status-text">--</span>
        </div>
      </div>

      <div class="control-row">
        <label>Network Status</label>
        <div class="hardware-info">
          <span class="status-indicator" id="network_indicator"></span>
          <span id="network_status" class="status-text">--</span>
        </div>
      </div>
    </section>
  </main>

  <footer>
    <p>VENTCON Control System v%VERSION% by HAB</p>
    <!-- Hidden Easter egg panel -->
    <div id="easterEgg" style="display: none; margin-top: 20px; padding: 15px; background: #f0f8ff; border-radius: 8px; text-align: left;">
      <h3 style="color: #002f87; text-align: center;">Developer Mode Activated! 🚀</h3>
      <p style="text-align: center;">Hello there, curious one!</p> 
      <p style="text-align: center;">You've found the secret developer panel.</p>
      <p style="text-align: center;">VENTCON Control System - Made with ❤️ by VENTREX</p>
      
      <div style="margin-top: 20px;">
        <h4 style="color: #002f87; margin-bottom: 10px;">📊 System Information</h4>
        <div id="devInfo" style="font-family: monospace; font-size: 12px; margin-bottom: 15px;"></div>
          <h4 style="color: #002f87; margin-bottom: 10px;">💻 Serial Commands Reference</h4>
        <div style="font-family: monospace; font-size: 11px; background: #fff; padding: 12px; border-radius: 6px; max-height: 300px; overflow-y: auto; border: 1px solid #ddd;">
          <strong>PID Control:</strong><br>
          KP 0.5 &nbsp;&nbsp;&nbsp;&nbsp; Set proportional gain<br>
          KI 0.1 &nbsp;&nbsp;&nbsp;&nbsp; Set integral gain<br>
          KD 0.01 &nbsp;&nbsp;&nbsp;&nbsp;Set derivative gain<br>
          SP 3.0 &nbsp;&nbsp;&nbsp;&nbsp; Set pressure setpoint (bar)<br>          
          SAMPLE 10 &nbsp;&nbsp;Set PID sample time (1-1000 ms)<br>
          RESET &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Reset PID controller (clear integral windup)<br><br>
          
          <strong>Signal Processing:</strong><br>
          FLT 0.2 &nbsp;&nbsp;&nbsp;&nbsp;Set filter strength (0.0-1.0)<br>
          AW ON/OFF &nbsp;&nbsp;Enable/disable anti-windup for deadband<br>
          HYST ON/OFF &nbsp;Enable/disable hysteresis compensation<br>
          HYSTAMT 5 &nbsp;&nbsp;Set hysteresis compensation amount (%)<br><br>
          
          <strong>PWM Control:</strong><br>
          FREQ 1000 &nbsp;&nbsp;Set PWM frequency (100-10000Hz)<br>
          RES 8 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Set PWM resolution (1-16 bits)<br>
          PWM 25 &nbsp;&nbsp;&nbsp;&nbsp;Force PWM duty cycle (0-100%) for testing<br>
          RESUME &nbsp;&nbsp;&nbsp;&nbsp;Resume normal PID control after manual PWM<br><br>
          
          <strong>Control Loop:</strong><br>
          CONTROL FREQ 1000 Set control loop frequency (10-1000 Hz)<br><br>
          
          <strong>Auto-Tuning:</strong><br>
          TUNE START &nbsp;Start PID auto-tuning process<br>
          TUNE STOP &nbsp;&nbsp;Cancel auto-tuning process<br>
          TUNE CANCEL Cancel auto-tuning process<br>
          TUNE ACCEPT Accept auto-tuned PID parameters<br>
          TUNE REJECT Reject auto-tuned PID parameters<br>
          TUNE SP 3.0 Set auto-tuning test setpoint (0.5-10.0 bar)<br>
          TUNE MIN 65 Set auto-tuning minimum PWM (50-90%)<br>
          TUNE MAX 85 Set auto-tuning maximum PWM (60-95%)<br>
          TUNE CYCLE 100 Set min cycle time for auto-tuning (50-2000ms)<br>
          TUNE RULE n Select auto-tuning rule (0-3, see TUNE RULES)<br>
          TUNE AGGR x Set tuning aggressiveness (0.5-2.0)<br>
          TUNE RULES &nbsp;Show available tuning rules<br><br>
          
          <strong>System & Data:</strong><br>
          STATUS &nbsp;&nbsp;&nbsp;&nbsp;Show current parameters<br>
          SAVE &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Force save settings to flash<br>
          READ &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Read settings stored in flash<br>
          STARTCD &nbsp;&nbsp;&nbsp;Start continuous data output for plotting<br>
          STOPCD &nbsp;&nbsp;&nbsp;&nbsp;Stop continuous data output<br>
          PAGE ON &nbsp;&nbsp;&nbsp;Enable web server processing<br>
          PAGE OFF &nbsp;&nbsp;Disable web server processing<br>
          DIR &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;List all files in flash memory with sizes<br>
          VER &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Display firmware version and build info<br>
          HELP &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Show this help message<br>
        </div>
      </div>
    </div>
  </footer>

  <script defer>
    // Cached DOM elements
    let cachedElements = {};
    
    // Cache DOM elements on load
    function cacheElements() {
      cachedElements = {
        loader: document.getElementById('loader'),
        pressure: document.getElementById('pressure'),
        pressureFill: document.getElementById('pressure-fill'),
        pressureTarget: document.getElementById('pressure-target'),
        pressureTrend: document.getElementById('pressure-trend'),
        pwm: document.getElementById('pwm'),
        pwmFill: document.getElementById('pwm-fill'),
        pwmTrend: document.getElementById('pwm-trend'),
        adcIndicator: document.getElementById('adc_indicator'),
        adcStatus: document.getElementById('adc_status'),
        networkIndicator: document.getElementById('network_indicator'),
        networkStatus: document.getElementById('network_status'),
        chartToggle: document.getElementById('chartToggle'),
        chartContainer: document.getElementById('chartContainer'),
        saveSnackbar: document.getElementById('saveSnackbar'),
        saveSnackbarText: document.getElementById('saveSnackbarText'),
        // Cache all slider and text elements
        sliders: {
          sp: document.getElementById('sp_slider'),
          kp: document.getElementById('kp_slider'),
          ki: document.getElementById('ki_slider'),
          kd: document.getElementById('kd_slider'),
          flt: document.getElementById('flt_slider'),
          freq: document.getElementById('freq_slider'),
          res: document.getElementById('res_slider')
        },
        texts: {
          sp: document.getElementById('sp_text'),
          kp: document.getElementById('kp_text'),
          ki: document.getElementById('ki_text'),
          kd: document.getElementById('kd_text'),
          flt: document.getElementById('flt_text'),
          freq: document.getElementById('freq_text'),
          res: document.getElementById('res_text')
        }
      };
    }

    // Wait for all deferred scripts to load before initializing
    function initializeApp() {
      // Check if Chart.js is available
      if (typeof Chart === 'undefined') {
        setTimeout(initializeApp, 50);
        return;
      }
      
      cacheElements();
      if(cachedElements.loader) cachedElements.loader.style.display = 'none';
      
      // Initialize the chart after Chart.js is loaded
      initializeChart();
      
      // Setup all other functionality
      setupEventListeners();
      startDataUpdates();
    }
    
    // Initialize chart in separate function
    function initializeChart() {
      const ctx = document.getElementById('pressureChart');
      if (!ctx) return;
      
      const chartCtx = ctx.getContext('2d');
      window.pressureData = [];
      window.setpointData = [];
      window.pwmData = [];
      
      // Create chart instance
      window.pressureChart = new Chart(chartCtx, 
      {
        type: 'line',
        data: {
          datasets: [
            {
              label: 'Outlet',
              data: pressureData,
              borderColor: '#2563eb',
              backgroundColor: 'rgba(37, 99, 235, 0.1)',
              tension: 0.3,
              borderWidth: 2,
              pointRadius: 2,
              pointHoverRadius: 4,
              pointBorderWidth: 1,
              pointStyle: 'circle',
              yAxisID: 'y'  // Assign to left y-axis
            },
            {
              label: 'Setpoint',
              data: setpointData,
              borderColor: '#f59e0b',
              backgroundColor: 'rgba(245, 158, 11, 0.1)',
              borderDash: [5, 5],
              tension: 0.1,
              borderWidth: 2,
              pointRadius: 0,
              pointHoverRadius: 3,
              yAxisID: 'y'  // Assign to left y-axis
            },
            {
              label: 'PWM Output',
              data: pwmData,
              borderColor: '#10b981',  // Green color for PWM
              backgroundColor: 'rgba(16, 185, 129, 0.1)',
              tension: 0.3,
              borderWidth: 2,
              pointRadius: 1,
              pointHoverRadius: 3,
              pointBorderWidth: 1,
              pointStyle: 'circle',
              yAxisID: 'pwm'  // Assign to right y-axis
            }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: {
            duration: 0
          },
          scales: {
            y: {
              beginAtZero: true,
              min: 0,
              max: 10,
              position: 'left',
              title: {
                display: true,
                text: 'Pressure in bar(g)'
              },
              ticks: {
                color: '#2563eb',  // Match the color of the Pressure line
                stepSize: 2,
                autoSkip: false,
                callback: function(value) {
                  return value;
                }
              },
              grid: {
                display: true
              }
            },
            pwm: {
              beginAtZero: true,
              min: 0,
              max: 100,
              position: 'right',
              title: {
                display: true,
                text: 'PWM (%)'
              },
              ticks: {
                color: '#10b981',  // Match the color of the PWM line
                stepSize: 20,
                autoSkip: false,
                callback: function(value) {
                  return value;
                }
              },
              grid: {
                display: false  // Don't show grid lines for secondary axis
              }
            },
            x: {
              type: 'time',
              time: {
                unit: 'second',
                displayFormats: {
                  second: 'HH:mm:ss'
                },
                tooltipFormat: 'HH:mm:ss'
              },
              title: {
                display: true,
                text: new Date().toLocaleDateString() + ' ' + new Date().toLocaleTimeString(),
                font: {
                  size: 8,
                  family: 'courier, monospace',
                  weight: 'bold'
                }
              },
              ticks: {
                color: '#1e293b',  // Dark color for x-axis labels
                autoSkip: true,
                maxTicksLimit: 10,
                font: {
                  size: 8,
                  family: 'courier, monospace',
                  weight: 'bold'
                },
                maxRotation: 45,  // Prevent label rotation
                minRotation: 45   // Prevent label rotation
              },

            }
          },
          plugins: {
            legend: {
              position: 'top',
              labels: {
                boxWidth: 25,
                usePointStyle: false,
                generateLabels: function(chart) {
                  // Get the default labels
                  const original = Chart.defaults.plugins.legend.labels.generateLabels(chart);
                  
                  // Apply custom styling for lines instead of points
                  original.forEach(label => {
                    // For the setpoint dataset (which has dashed style)
                    if (label.text === 'Setpoint') {
                      label.lineDash = [5, 5]; // Match the graph's dashed style
                    }
                  });
                  
                  return original;
                }
              }
            },
            tooltip: {
              backgroundColor: 'rgba(255, 255, 255, 0.9)',
              titleColor: '#1e293b',
              bodyColor: '#1e293b',
              borderColor: '#e2e8f0',
              borderWidth: 1,
              cornerRadius: 6,
              displayColors: true,
              callbacks: {
                label: function(context) {
                  const label = context.dataset.label;
                  const value = context.parsed.y.toFixed(3);
                  if (label === 'PWM Output') {
                    return label + ': ' + value + '%';
                  }
                  return label + ': ' + value + ' bar';
                }
              }
            }
          }
        }
      });
    }
    
    // Setup all event listeners
    function setupEventListeners() {
      // Chart toggle functionality
      if (cachedElements.chartToggle && cachedElements.chartContainer) {
        cachedElements.chartToggle.addEventListener('change', function() {
          if (!cachedElements.chartContainer || !window.pressureChart) return;
          
          if (this.checked) {
            cachedElements.chartContainer.style.display = 'block';
            // Force a single update when showing chart with 'none' animation mode
            window.pressureChart.update('none');
          } else {
            cachedElements.chartContainer.style.display = 'none';
          }
        });
      }
        // Setup slider and text input synchronization
      ["sp", "kp", "ki", "kd", "flt", "freq", "res"].forEach(function(param) {
        const slider = cachedElements.sliders ? cachedElements.sliders[param] : null;
        const text = cachedElements.texts ? cachedElements.texts[param] : null;
        
        if (slider && text) {
          slider.addEventListener('input', function() {
            text.value = slider.value;
            showSaveSnackbar(param, slider.value);
          });

          text.addEventListener('input', function() {
            slider.value = text.value;
            showSaveSnackbar(param, text.value);
          });
        }
      });

      // Setup increment/decrement buttons
      [
        {param: 'sp', min: 0, max: 10, step: 0.1},
        {param: 'kp', min: 0, max: 3000, step: 100},
        {param: 'ki', min: 0, max: 5000, step: 200},
        {param: 'kd', min: 0, max: 1000, step: 10},
        {param: 'flt', min: 0, max: 1, step: 0.01},
        {param: 'freq', min: 100, max: 10000, step: 100},
        {param: 'res', min: 8, max: 16, step: 1},
        {param: 'psamt', min: 5, max: 200, step: 1}
      ].forEach(function(cfg) {
        const slider = cachedElements.sliders ? cachedElements.sliders[cfg.param] : null;
        const text = cachedElements.texts ? cachedElements.texts[cfg.param] : null;
        const decBtn = document.getElementById(cfg.param + '_decrement');
        const incBtn = document.getElementById(cfg.param + '_increment');
        
        if (decBtn && incBtn && slider && text) {
          decBtn.addEventListener('click', function() {
            let value = parseFloat(text.value);
            value = Math.max(cfg.min, +(value - cfg.step).toFixed(10));
            text.value = value;
            slider.value = value;
            showSaveSnackbar(cfg.param, value);
          });
          incBtn.addEventListener('click', function() {
            let value = parseFloat(text.value);
            value = Math.min(cfg.max, +(value + cfg.step).toFixed(10));
            text.value = value;
            slider.value = value;
            showSaveSnackbar(cfg.param, value);
          });
        }
      });
      
      // Setup save snackbar click handler
      if (cachedElements.saveSnackbar && cachedElements.saveSnackbarText) {
        cachedElements.saveSnackbar.addEventListener('click', handleSaveClick);
      }
      
      // Setup PID reset button
      const resetBtn = document.getElementById('resetPidBtn');
      if (resetBtn) {
        resetBtn.addEventListener('click', handlePidReset);
      }
      
      // Setup Easter egg
      setupEasterEgg();
      
      // Setup scroll handler
      setupScrollHandler();
    }
    
    // Start periodic data updates
    function startDataUpdates() {
      setInterval(updateData, 250);
    }

    // Data update function
    function updateData() {
      fetch('/values')
        .then(r => r.json())
        .then(data => {
          // Update pressure display with cached elements
          if (typeof data.pressure !== "undefined") {
            const pressureVal = data.pressure;
            if (cachedElements.pressure) {
              cachedElements.pressure.textContent = pressureVal.toFixed(2);
            }
            
            // Update pressure fill and calculate percentage (0-10 bar range)
            const pressurePercent = (pressureVal / 10) * 100;
            if (cachedElements.pressureFill) {
              cachedElements.pressureFill.style.width = `${pressurePercent}%`;
            }
            
            // Update setpoint target marker
            const setpointPercent = (data.sp / 10) * 100;
            if (cachedElements.pressureTarget) {
              cachedElements.pressureTarget.style.left = `${setpointPercent}%`;
            }
            
            // Update trend indicator
            updatePressureTrend(pressureVal);
            
            // Get PWM value for chart
            const pwmVal = (data.pwm !== undefined) ? data.pwm : 0;
            
            // Always call updateChart, it will handle visibility internally
            updateChart(pressureVal, data.sp, pwmVal);
          } else {
            if (cachedElements.pressure) cachedElements.pressure.textContent = "--";
            if (cachedElements.pressureFill) cachedElements.pressureFill.style.width = "0%";
          }
          
          // Update PWM output with cached elements
          if (data.pwm !== undefined) {
            const pwmVal = parseFloat(data.pwm);
            if (cachedElements.pwm) {
              cachedElements.pwm.textContent = pwmVal.toFixed(3);
            }
            if (cachedElements.pwmFill) {
              cachedElements.pwmFill.style.width = `${pwmVal}%`;
            }
            
            // Update PWM trend indicator
            updatePwmTrend(pwmVal);
          } else {
            if (cachedElements.pwm) cachedElements.pwm.textContent = "--";
            if (cachedElements.pwmFill) cachedElements.pwmFill.style.width = "0%";
          }
          
          // Update ADC status with cached elements
          if (cachedElements.adcIndicator && cachedElements.adcStatus) {
            if (data.adc_status === "100") {
              cachedElements.adcStatus.textContent = "External ADS1015 ADC - 12 Bit";
              cachedElements.adcIndicator.style.backgroundColor = 'var(--success)';
            } else if(data.adc_status === "000") {
              cachedElements.adcStatus.textContent = "Internal ESP32 ADC - 12 Bit";
              cachedElements.adcIndicator.style.backgroundColor = 'var(--accent)';
            } else {
              cachedElements.adcStatus.textContent = "Unknown";
              cachedElements.adcIndicator.style.backgroundColor = 'var(--danger)';
            }
          }
          
          // Update Network status with cached elements
          if (cachedElements.networkIndicator && cachedElements.networkStatus) {
            cachedElements.networkStatus.textContent = "VENTCON_AP, IP: 192.168.4.1";
            cachedElements.networkIndicator.style.backgroundColor = 'var(--success)';
          }
          
          // Only update UI controls if no unsaved changes (saveSnackbar is hidden)
          if (cachedElements.saveSnackbar && cachedElements.saveSnackbar.style.display === 'none') {
            // Update all sliders and inputs with current values from server using cached elements
            ["sp", "kp", "ki", "kd", "flt", "freq", "res"].forEach(param => {
              if (typeof data[param] !== "undefined") {
                if (cachedElements.sliders && cachedElements.sliders[param]) {
                  cachedElements.sliders[param].value = data[param];
                }
                if (cachedElements.texts && cachedElements.texts[param]) {
                  cachedElements.texts[param].value = data[param];
                }
              }
            });
          }
        })
        .catch(err => {
          console.error("Error fetching values:", err);
        });
    }

    // Hide loader as soon as DOM is interactive
    document.addEventListener('DOMContentLoaded', initializeApp);

    // Function to add data to the chart
    function updateChart(pressure, setpoint, pwm) {
      if (!window.pressureChart || !cachedElements.chartToggle) return;
      
      const now = new Date();
      
      // Always collect data, even when chart is hidden
      window.pressureData.push({
        x: now,
        y: pressure
      });
      
      window.setpointData.push({
        x: now,
        y: setpoint
      });
      
      window.pwmData.push({
        x: now,
        y: pwm
      });
      
      // Maintain fixed data window size
      if (window.pressureData.length > 30) {
        window.pressureData.shift();
        window.setpointData.shift();
        window.pwmData.shift();
      }
      
      // Only update the chart if it's visible - use optimized update
      if (cachedElements.chartToggle.checked) {
        // Use 'none' mode for better performance - no animations
        window.pressureChart.update('none');
      }
    }

    // Track pressure values for trend indicator
    let lastPressureValues = [];
    let pressureTrend = 0; // -1: down, 0: stable, 1: up
    
    // Update pressure trend
    function updatePressureTrend(newValue) {
      // Keep last 3 values for trend calculation
      if (lastPressureValues.length >= 3) {
        lastPressureValues.shift();
      }
      lastPressureValues.push(newValue);
      
      // Calculate trend only when we have enough values
      if (lastPressureValues.length >= 3) {
        const latest = lastPressureValues[lastPressureValues.length - 1];
        const oldest = lastPressureValues[0];
        const diff = latest - oldest;
        
        // Determine trend direction with a threshold to avoid minor fluctuations
        const threshold = 0.05;
        if (diff > threshold) {
          pressureTrend = 1; // up
        } else if (diff < -threshold) {
          pressureTrend = -1; // down
        } else {
          pressureTrend = 0; // stable
        }
        
        // Update trend indicator using cached element
        if (cachedElements.pressureTrend) {
          if (pressureTrend > 0) {
            cachedElements.pressureTrend.className = 'trend-indicator trend-up';
          } else if (pressureTrend < 0) {
            cachedElements.pressureTrend.className = 'trend-indicator trend-down';
          } else {
            cachedElements.pressureTrend.className = 'trend-indicator trend-stable';
          }
        }
      }
    }
    
    // Track PWM values for trend indicator
    let lastPwmValues = [];
    let pwmTrend = 0; // -1: down, 0: stable, 1: up
    
    // Update PWM trend
    function updatePwmTrend(newValue) {
      // Keep last 3 values for trend calculation
      if (lastPwmValues.length >= 3) {
        lastPwmValues.shift();
      }
      lastPwmValues.push(newValue);
      
      // Calculate trend only when we have enough values
      if (lastPwmValues.length >= 3) {
        const latest = lastPwmValues[lastPwmValues.length - 1];
        const oldest = lastPwmValues[0];
        const diff = latest - oldest;
        
        // Determine trend direction with a threshold to avoid minor fluctuations
        const threshold = 0.5; // Higher threshold for PWM since it can fluctuate more
        if (diff > threshold) {
          pwmTrend = 1; // up
        } else if (diff < -threshold) {
          pwmTrend = -1; // down
        } else {
          pwmTrend = 0; // stable
        }
        
        // Update trend indicator using cached element
        if (cachedElements.pwmTrend) {
          if (pwmTrend > 0) {
            cachedElements.pwmTrend.className = 'trend-indicator trend-up';
          } else if (pwmTrend < 0) {
            cachedElements.pwmTrend.className = 'trend-indicator trend-down';
          } else {
            cachedElements.pwmTrend.className = 'trend-indicator trend-stable';
          }
        }
      }
    }

    // Track changes to show the save snackbar
    let pendingChanges = {};
    let changeTimeout;

    // Snackbar management functions
    function showSaveSnackbar(param, value) {
      if (!cachedElements.saveSnackbar) return;
      
      // Store the changed parameter
      pendingChanges[param] = value;
      
      // Show the save snackbar
      cachedElements.saveSnackbar.style.display = 'block';
      
      // Add animation for a subtle bounce effect with null check
      if (cachedElements.saveSnackbar.animate) {
        cachedElements.saveSnackbar.animate([
          { transform: 'translateX(-50%) scale(0.95)' },
          { transform: 'translateX(-50%) scale(1.02)' },
          { transform: 'translateX(-50%) scale(1)' }
        ], { duration: 300, easing: 'ease-out' });
      }
      
      // Auto-hide after 8 seconds of inactivity
      clearTimeout(changeTimeout);
      changeTimeout = setTimeout(() => {
        if (cachedElements.saveSnackbar && cachedElements.saveSnackbar.animate) {
          cachedElements.saveSnackbar.animate([
            { opacity: 1 },
            { opacity: 0 }
          ], { duration: 300, easing: 'ease-out' });
          
          setTimeout(() => {
            if (cachedElements.saveSnackbar) {
              cachedElements.saveSnackbar.style.display = 'none';
            }
          }, 300);
        }
      }, 8000);
    }
    
    // Handle save button click
    function handleSaveClick() {
      if (!cachedElements.saveSnackbarText) return;
      
      cachedElements.saveSnackbarText.textContent = "Saving...";
      cachedElements.saveSnackbar.style.pointerEvents = 'none';
      
      // Build parameters string from pending changes
      const params = Object.entries(pendingChanges).map(([param, value]) => 
        param + "=" + encodeURIComponent(value)
      ).join("&");

      fetch("/set?" + params)
        .then(() => {
          if (cachedElements.saveSnackbarText && cachedElements.saveSnackbar) {
            cachedElements.saveSnackbarText.textContent = "Settings Updated";
            cachedElements.saveSnackbar.style.background = '#10b981'; // Success green
            
            // Clear pending changes
            pendingChanges = {};
            
            // Hide snackbar after a short delay
            setTimeout(() => {
              if (cachedElements.saveSnackbar && cachedElements.saveSnackbar.animate) {
                cachedElements.saveSnackbar.animate([
                  { opacity: 1 },
                  { opacity: 0 }
                ], { duration: 300, easing: 'ease-out' });
                
                setTimeout(() => {
                  if (cachedElements.saveSnackbar && cachedElements.saveSnackbarText) {
                    cachedElements.saveSnackbar.style.display = 'none';
                    cachedElements.saveSnackbar.style.background = '#2563eb'; // Reset to blue
                    cachedElements.saveSnackbarText.textContent = "Apply Changes";
                    cachedElements.saveSnackbar.style.pointerEvents = 'auto';
                  }
                }, 300);
              }
            }, 1000);
          }
        })
        .catch(err => {
          if (cachedElements.saveSnackbarText && cachedElements.saveSnackbar) {
            cachedElements.saveSnackbarText.textContent = "Try Again";
            cachedElements.saveSnackbar.style.background = '#dc2626'; // Error red
            cachedElements.saveSnackbar.style.pointerEvents = 'auto';
          }
          console.error("Failed to save settings:", err);
        });
    }

    // Handle PID reset
    function handlePidReset() {
      const btn = this;
      const originalText = btn.textContent;
      btn.textContent = "Resetting...";
      btn.style.backgroundColor = "#60a5fa"; // Lighter blue during reset
      
      // Call the reset endpoint
      fetch('/resetPID')
        .then(response => response.json())
        .then(data => {
          if (data.success) {
            btn.textContent = "Reset!";
            btn.style.backgroundColor = "#10b981"; // Success green
            
            // Display a notification using the save snackbar
            const saveSnackbar = cachedElements.saveSnackbar || document.getElementById('saveSnackbar');
            const saveSnackbarText = cachedElements.saveSnackbarText || document.getElementById('saveSnackbarText');
            
            if (saveSnackbar && saveSnackbarText) {
              saveSnackbar.style.display = 'block';
              saveSnackbar.style.background = '#10b981'; // Success green
              saveSnackbarText.textContent = "PID Controller Reset";
              
              // Hide snackbar after 2 seconds
              setTimeout(() => {
                if (saveSnackbar && saveSnackbar.animate) {
                  saveSnackbar.animate([
                    { opacity: 1 },
                    { opacity: 0 }
                  ], { duration: 300, easing: 'ease-out' });
                  
                  setTimeout(() => {
                    if (saveSnackbar && saveSnackbarText) {
                      saveSnackbar.style.display = 'none';
                      saveSnackbar.style.background = '#2563eb'; // Reset to blue
                      saveSnackbarText.textContent = "Apply Changes";
                    }
                  }, 300);
                }
              }, 2000);
            }
            
            // Reset button after 1 second
            setTimeout(() => {
              btn.textContent = originalText;
              btn.style.backgroundColor = "#f59e0b"; // Back to original color
            }, 1000);
          } else {
            btn.textContent = "Error";
            btn.style.backgroundColor = "#ef4444"; // Error red
            setTimeout(() => {
              btn.textContent = originalText;
              btn.style.backgroundColor = "#f59e0b";
            }, 1500);
          }
        })
        .catch(error => {
          console.error('Error resetting PID:', error);
          btn.textContent = "Error";
          btn.style.backgroundColor = "#ef4444"; // Error red
          setTimeout(() => {
            btn.textContent = originalText;
            btn.style.backgroundColor = "#f59e0b";
          }, 1500);
        });
    }

    // Setup Easter egg
    function setupEasterEgg() {
      const logo = document.getElementById('ventrexLogo');
      const easterEgg = document.getElementById('easterEgg');
      const devInfo = document.getElementById('devInfo');
      
      if (logo && easterEgg && devInfo) {
        let clickCount = 0;
        let clickTimer;
        
        logo.addEventListener('click', function() {
          clickCount++;
          
          // Reset click count after 2 seconds of inactivity
          clearTimeout(clickTimer);
          clickTimer = setTimeout(() => { clickCount = 0; }, 2000);
          
          // Activate Easter egg after 5 clicks
          if (clickCount >= 5) {
            if (easterEgg.style.display === 'none') {
              // Show Easter egg and populate with system info
              easterEgg.style.display = 'block';
              devInfo.innerHTML = 
                `<div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px;">
                   <div>Memory: ${performance?.memory?.usedJSHeapSize ? 
                     (performance.memory.usedJSHeapSize/1048576).toFixed(2) + ' MB' : 'N/A'}</div>
                   <div>Chart Points: ${window.pressureData ? window.pressureData.length : 0}</div>
                   <div>Time: ${new Date().toLocaleTimeString()}</div>
                   <div>Language: ${navigator.language}</div>
                   <div>Resolution: ${window.screen.width}x${window.screen.height}</div>
                   <div>Timezone: ${Intl.DateTimeFormat().resolvedOptions().timeZone}</div>
                 </div>
                 <div style="margin-top: 8px; font-size: 10px; color: #666;">
                   Platform: ${navigator.platform || 'Unknown'}<br>
                   User Agent: ${navigator.userAgent.substring(0, 80)}...
                 </div>`;
              
              // Add a small animation to the logo
              logo.style.transition = 'transform 1s';
              logo.style.transform = 'rotate(360deg)';
              setTimeout(() => { logo.style.transform = 'rotate(0deg)'; }, 1000);
            } else {
              // Hide Easter egg
              easterEgg.style.display = 'none';
            }
            
            clickCount = 0;
          }
        });
      }
    }

    // Setup scroll handler
    function setupScrollHandler() {
      const sliders = document.querySelectorAll('input[type="range"]');
      if (sliders && sliders.length > 0) {
        let scrollTimeout;
        
        // Add a class to disable sliders during scroll
        function disableSliders() {
          sliders.forEach(slider => {
            if (slider && slider.classList) {
              slider.classList.add('scrolling');
            }
          });
        }
        
        // Remove the class with a small delay after scrolling stops
        function enableSliders() {
          clearTimeout(scrollTimeout);
          scrollTimeout = setTimeout(() => {
            sliders.forEach(slider => {
              if (slider && slider.classList) {
                slider.classList.remove('scrolling');
              }
            });
          }, 250); // Wait 250ms after scrolling stops before re-enabling
        }
        
        // Attach scroll event listener
        if (window.addEventListener) {
          window.addEventListener('scroll', () => {
            disableSliders();
            enableSliders();
          }, { passive: true });
        }
      }
    }
  </script>
</body>
</html>
)rawliteral";

// Function to assemble the full HTML content at runtime
// Adding 'inline' to prevent multiple definition errors when included in multiple files

inline String getFullHtmlContent() 
{
  String fullHtml = FPSTR(HTML_CONTENT);
  fullHtml += FPSTR(CSS_STYLES);
  fullHtml += FPSTR(HTML_CONTENT_AFTER_STYLE);
  return fullHtml;
}