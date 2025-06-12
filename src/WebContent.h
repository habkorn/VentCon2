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
  <!-- Load JS libraries from LittleFS root -->
  <script src="/chart.min.js"></script>
  <script src="/moment.min.js"></script>
  <script src="/chartjs-adapter-moment.min.js"></script>

)rawliteral";

// Concatenate parts of the HTML
const char HTML_CONTENT_AFTER_STYLE[] PROGMEM = R"rawliteral(

</head>

<body>
  <div class="loader" id="loader" style="position:fixed;top:0;left:0;width:100vw;height:100vh;z-index:9999;background:#fff;display:flex;align-items:center;justify-content:center;">
    <div style="display:flex;flex-direction:column;align-items:center;">
      <img src="/Logo.svg" alt="VENTREX Logo" style="height:60px;margin-bottom:16px;">
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
        <h3>Pressure Chart</h3>
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
          <input type="range" id="kp_slider" min="0" max="100" step="1" value="%KP%">
          <button type="button" class="slider-btn increment-btn" id="kp_increment">+</button>
          <input type="number" id="kp_text" value="%KP%" step="1">
        </div>
      </div>

      <div class="control-row">
        <label for="ki_slider">Integral</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="ki_decrement">-</button>
          <input type="range" id="ki_slider" min="0" max="100" step="1" value="%KI%">
          <button type="button" class="slider-btn increment-btn" id="ki_increment">+</button>
          <input type="number" id="ki_text" value="%KI%" step="1">
        </div>
      </div>

      <div class="control-row">
        <label for="kd_slider">Derivative</label>
        <div class="control-slider">
          <button type="button" class="slider-btn decrement-btn" id="kd_decrement">-</button>
          <input type="range" id="kd_slider" min="0" max="10" step="0.1" value="%KD%">
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
    <div id="easterEgg" style="display: none; margin-top: 20px; padding: 15px; background: #f0f8ff; border-radius: 8px; text-align: center;">
      <h3 style="color: #002f87;">Developer Mode Activated! 🚀</h3>
      <p>Hello there, curious one!</p> 
      <p>You've found the secret developer panel.</p>
      <p>VENTCON Control System - Created with ❤️ by VENTREX</p>
      <div id="devInfo"></div>
    </div>
  </footer>

  <script>
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

    // Hide loader as soon as DOM is interactive
    document.addEventListener('DOMContentLoaded', function() {
      cacheElements();
      if(cachedElements.loader) cachedElements.loader.style.display = 'none';
    });

    // Initialize the chart
    const ctx = document.getElementById('pressureChart').getContext('2d');
    let pressureData = [];
    let setpointData = [];
    let pwmData = [];  // New dataset for PWM values
    
    // Create chart instance
    const pressureChart = new Chart(ctx, 
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

    // Function to add data to the chart
    function updateChart(pressure, setpoint, pwm) 
    {
      const now = new Date();
      
      // Always collect data, even when chart is hidden
      pressureData.push({
        x: now,
        y: pressure
      });
      
      setpointData.push({
        x: now,
        y: setpoint
      });
      
      pwmData.push({
        x: now,
        y: pwm
      });
      
      if (pressureData.length > 30) 
      {
        pressureData.shift();
        setpointData.shift();
        pwmData.shift();
      }
      
      // Only update the chart if it's visible
      if (cachedElements.chartToggle && cachedElements.chartToggle.checked) 
      {
        pressureChart.update();
      }
    }

    // Toggle chart visibility
    document.addEventListener('DOMContentLoaded', function() {
      if (cachedElements.chartToggle) {
        cachedElements.chartToggle.addEventListener('change', function() 
        {
          if (this.checked) 
          {
            cachedElements.chartContainer.style.display = 'block';
            // Don't clear the data, just update the chart with existing data
            pressureChart.update();
          } 
          else 
          {
            cachedElements.chartContainer.style.display = 'none';
          }
        });
      }
    });

    // Track changes to show the save snackbar
    let pendingChanges = {};
    let changeTimeout;
    
    // Synchronize range and number inputs, show save snackbar on change
    document.addEventListener('DOMContentLoaded', function() {
      ["sp", "kp", "ki", "kd", "flt", "freq", "res"].forEach(function(param) 
      {
        const slider = cachedElements.sliders[param];
        const text = cachedElements.texts[param];
        
        if (slider && text) {
          slider.addEventListener('input', function() 
          {
            text.value = slider.value;
            showSaveSnackbar(param, slider.value);
          });

          text.addEventListener('input', function() 
          {
            slider.value = text.value;
            showSaveSnackbar(param, text.value);
          });
        }
      });

      // Add increment/decrement button logic for all sliders
      [
        {param: 'sp', min: 0, max: 10, step: 0.1},
        {param: 'kp', min: 0, max: 100, step: 1},
        {param: 'ki', min: 0, max: 100, step: 1},
        {param: 'kd', min: 0, max: 10, step: 0.1},
        {param: 'flt', min: 0, max: 1, step: 0.01},
        {param: 'freq', min: 100, max: 10000, step: 100},
        {param: 'res', min: 8, max: 16, step: 1}
      ].forEach(function(cfg) {
        const slider = cachedElements.sliders[cfg.param];
        const text = cachedElements.texts[cfg.param];
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
    });

    // Snackbar management functions
    function showSaveSnackbar(param, value) {
      // Store the changed parameter
      pendingChanges[param] = value;
      
      // Show the save snackbar
      if (cachedElements.saveSnackbar) {
        cachedElements.saveSnackbar.style.display = 'block';
        
        // Add animation for a subtle bounce effect
        cachedElements.saveSnackbar.animate([
          { transform: 'translateX(-50%) scale(0.95)' },
          { transform: 'translateX(-50%) scale(1.02)' },
          { transform: 'translateX(-50%) scale(1)' }
        ], { duration: 300, easing: 'ease-out' });
      }
      
      // Auto-hide after 8 seconds of inactivity
      clearTimeout(changeTimeout);
      changeTimeout = setTimeout(() => {
        if (cachedElements.saveSnackbar) {
          cachedElements.saveSnackbar.animate([
            { opacity: 1 },
            { opacity: 0 }
          ], { duration: 300, easing: 'ease-out' });
          
          setTimeout(() => {
            cachedElements.saveSnackbar.style.display = 'none';
          }, 300);
        }
      }, 8000);
    }
    
    // Initialize save snackbar click handler
    document.addEventListener('DOMContentLoaded', function() {
      if (cachedElements.saveSnackbar) {
        cachedElements.saveSnackbar.addEventListener('click', function() 
        {
          cachedElements.saveSnackbarText.textContent = "Saving...";
          cachedElements.saveSnackbar.style.pointerEvents = 'none';
          
          // Build parameters string from pending changes
          const params = Object.entries(pendingChanges).map(([param, value]) => 
            param + "=" + encodeURIComponent(value)
          ).join("&");

          fetch("/set?" + params)
            .then(() => {
              cachedElements.saveSnackbarText.textContent = "Settings Updated";
              cachedElements.saveSnackbar.style.background = '#10b981'; // Success green
              
              // Clear pending changes
              pendingChanges = {};
              
              // Hide snackbar after a short delay
              setTimeout(() => {
                cachedElements.saveSnackbar.animate([
                  { opacity: 1 },
                  { opacity: 0 }
                ], { duration: 300, easing: 'ease-out' });
                
                setTimeout(() => {
                  cachedElements.saveSnackbar.style.display = 'none';
                  cachedElements.saveSnackbar.style.background = '#2563eb'; // Reset to blue
                  cachedElements.saveSnackbarText.textContent = "Apply Changes";
                  cachedElements.saveSnackbar.style.pointerEvents = 'auto';
                }, 300);
              }, 1000);
            })
            .catch(err => {
              cachedElements.saveSnackbarText.textContent = "Try Again";
              cachedElements.saveSnackbar.style.background = '#dc2626'; // Error red
              cachedElements.saveSnackbar.style.pointerEvents = 'auto';
              console.error("Failed to save settings:", err);
            });
        });
      }
    });

    // Track pressure values for trend indicator
    let lastPressureValues = [];
    let pressureTrend = 0; // -1: down, 0: stable, 1: up
    
    // Update pressure trend
    function updatePressureTrend(newValue) 
    {
      // Keep last 3 values for trend calculation
      if (lastPressureValues.length >= 3) 
      {
        lastPressureValues.shift();
      }
      lastPressureValues.push(newValue);
      
      // Calculate trend only when we have enough values
      if (lastPressureValues.length >= 3) 
      {
        const latest = lastPressureValues[lastPressureValues.length - 1];
        const oldest = lastPressureValues[0];
        const diff = latest - oldest;
        
        // Determine trend direction with a threshold to avoid minor fluctuations
        const threshold = 0.05;
        if (diff > threshold) 
        {
          pressureTrend = 1; // up
        } 
        else if (diff < -threshold) 
        {
          pressureTrend = -1; // down
        } 
        else 
        {
          pressureTrend = 0; // stable
        }
        
        // Update trend indicator using cached element
        if (cachedElements.pressureTrend) {
          if (pressureTrend > 0) 
          {
            cachedElements.pressureTrend.className = 'trend-indicator trend-up';
          } 
          else if (pressureTrend < 0) 
          {
            cachedElements.pressureTrend.className = 'trend-indicator trend-down';
          } 
          else 
          {
            cachedElements.pressureTrend.className = 'trend-indicator trend-stable';
          }
        }
      }
    }
    
    // Track PWM values for trend indicator
    let lastPwmValues = [];
    let pwmTrend = 0; // -1: down, 0: stable, 1: up
    
    // Update PWM trend
    function updatePwmTrend(newValue) 
    {
      // Keep last 3 values for trend calculation
      if (lastPwmValues.length >= 3) 
      {
        lastPwmValues.shift();
      }
      lastPwmValues.push(newValue);
      
      // Calculate trend only when we have enough values
      if (lastPwmValues.length >= 3) 
      {
        const latest = lastPwmValues[lastPwmValues.length - 1];
        const oldest = lastPwmValues[0];
        const diff = latest - oldest;
        
        // Determine trend direction with a threshold to avoid minor fluctuations
        const threshold = 0.5; // Higher threshold for PWM since it can fluctuate more
        if (diff > threshold) 
        {
          pwmTrend = 1; // up
        } 
        else if (diff < -threshold) 
        {
          pwmTrend = -1; // down
        } 
        else 
        {
          pwmTrend = 0; // stable
        }
        
        // Update trend indicator using cached element
        if (cachedElements.pwmTrend) {
          if (pwmTrend > 0) 
          {
            cachedElements.pwmTrend.className = 'trend-indicator trend-up';
          } 
          else if (pwmTrend < 0) 
          {
            cachedElements.pwmTrend.className = 'trend-indicator trend-down';
          } 
          else 
          {
            cachedElements.pwmTrend.className = 'trend-indicator trend-stable';
          }
        }
      }
    }

    // Periodically update status and UI controls
    setInterval(() => 
    {
      fetch('/values')
        .then(r => r.json())
        .then(data => 
        {
          // Update pressure display with cached elements
          if (typeof data.pressure !== "undefined") 
          {
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
          } 
          else 
          {
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
          } 
          else 
          {
            if (cachedElements.pwm) cachedElements.pwm.textContent = "--";
            if (cachedElements.pwmFill) cachedElements.pwmFill.style.width = "0%";
          }
          
          // Update ADC status with cached elements
          if (cachedElements.adcIndicator && cachedElements.adcStatus) {
            if (data.adc_status === "100") {
              cachedElements.adcStatus.textContent = "External ADS1015 ADC - 12 Bit";
              cachedElements.adcIndicator.style.backgroundColor = 'var(--success)';
            } 
            else if(data.adc_status === "000") {
              cachedElements.adcStatus.textContent = "Internal ESP32 ADC - 12 Bit";
              cachedElements.adcIndicator.style.backgroundColor = 'var(--accent)';
            } 
            else 
            {
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
          if (cachedElements.saveSnackbar && cachedElements.saveSnackbar.style.display === 'none') 
          {
            // Update all sliders and inputs with current values from server using cached elements
            ["sp", "kp", "ki", "kd", "flt", "freq", "res"].forEach(param => {
              if (typeof data[param] !== "undefined") {
                if (cachedElements.sliders[param]) {
                  cachedElements.sliders[param].value = data[param];
                }
                if (cachedElements.texts[param]) {
                  cachedElements.texts[param].value = data[param];
                }
              }
            });
          }
        })
        .catch(err => 
        {
          console.error("Error fetching values:", err);
        });
    }, 300);    // Add event listener for PID reset button
    document.getElementById('resetPidBtn').addEventListener('click', function() {
      // Create visual feedback
      const btn = document.getElementById('resetPidBtn');
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
            const saveSnackbar = document.getElementById('saveSnackbar');
            const saveSnackbarText = document.getElementById('saveSnackbarText');
            saveSnackbar.style.display = 'block';
            saveSnackbar.style.background = '#10b981'; // Success green
            saveSnackbarText.textContent = "PID Controller Reset";
            
            // Hide snackbar after 2 seconds
            setTimeout(() => {
              saveSnackbar.animate([
                { opacity: 1 },
                { opacity: 0 }
              ], { duration: 300, easing: 'ease-out' });
              
              setTimeout(() => {
                saveSnackbar.style.display = 'none';
                saveSnackbar.style.background = '#2563eb'; // Reset to blue
                saveSnackbarText.textContent = "Apply Changes";
              }, 300);
            }, 2000);
            
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
    });
    
    // Easter egg implementation
    (function() {
      const logo = document.getElementById('ventrexLogo');
      let clickCount = 0;
      let clickTimer;
      
      logo.addEventListener('click', function() {
        clickCount++;
        
        // Reset click count after 2 seconds of inactivity
        clearTimeout(clickTimer);
        clickTimer = setTimeout(() => { clickCount = 0; }, 2000);
        
        // Activate Easter egg after 5 clicks
        if (clickCount >= 5) {
          const easterEgg = document.getElementById('easterEgg');
          
          if (easterEgg.style.display === 'none') {
            // Show Easter egg and populate with system info
            easterEgg.style.display = 'block';
            document.getElementById('devInfo').innerHTML = 
              `<p>ESP32 Uptime: ${Math.floor(Date.now()/1000)} seconds</p>
               <p>Memory Usage: JavaScript Heap ${performance?.memory?.usedJSHeapSize ? 
                 (performance.memory.usedJSHeapSize/1048576).toFixed(2) + ' MB' : 'Not available'}</p>
               <p>Chart Data Points: ${pressureData.length}</p>
               <p>Browser: ${navigator.userAgent}</p>`;
            
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
    })();

    // Add scroll handler to temporarily disable sliders during scrolling
    (function() {
      const sliders = document.querySelectorAll('input[type="range"]');
      let scrollTimeout;
      
      // Add a class to disable sliders during scroll
      function disableSliders() {
        sliders.forEach(slider => {
          slider.classList.add('scrolling');
        });
      }
      
      // Remove the class with a small delay after scrolling stops
      function enableSliders() {
        clearTimeout(scrollTimeout);
        scrollTimeout = setTimeout(() => {
          sliders.forEach(slider => {
            slider.classList.remove('scrolling');
          });
        }, 250); // Wait 250ms after scrolling stops before re-enabling
      }
      
      // Attach scroll event listener
      window.addEventListener('scroll', () => {
        disableSliders();
        enableSliders();
      }, { passive: true });
    })();
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