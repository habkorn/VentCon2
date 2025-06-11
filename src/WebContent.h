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
  <!-- Load JS libraries from SPIFFS root -->
  <script src="/chart.min.js"></script>
  <script src="/moment.min.js"></script>
  <script src="/chartjs-adapter-moment.min.js"></script>

)rawliteral";

// Concatenate parts of the HTML
const char HTML_CONTENT_AFTER_STYLE[] PROGMEM = R"rawliteral(

</head>

<body>

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
      <h2>Real-time Monitoring</h2>
      
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
            <span id="pwm">--</span>
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
      <div style="display: flex; justify-content: space-between; align-items: center;">
        <h2 style="margin-bottom: 0;">Control Parameters</h2>
        <button id="applyBtn" disabled>Apply Changes</button>
      </div>
      
      <div class="control-row">
        <label for="sp_slider">Setpoint Outlet Pressure in bar(g)</label>
        <div class="control-slider">
          <button type="button" class="slider-btn" id="sp_decrement">-</button>
          <input type="range" id="sp_slider" min="0" max="10" step="0.1" value="%SP%">
          <button type="button" class="slider-btn" id="sp_increment">+</button>
          <input type="number" id="sp_text" value="%SP%" step="0.1">
        </div>
      </div>

      <div class="section-title">
        <span>PID</span>
        <div></div>
      </div>

      <div class="control-row">
        <label for="kp_slider">Proportional</label>
        <div class="control-slider">
          <button type="button" class="slider-btn" id="kp_decrement">-</button>
          <input type="range" id="kp_slider" min="0" max="100" step="1" value="%KP%">
          <button type="button" class="slider-btn" id="kp_increment">+</button>
          <input type="number" id="kp_text" value="%KP%" step="1">
        </div>
      </div>

      <div class="control-row">
        <label for="ki_slider">Integral</label>
        <div class="control-slider">
          <button type="button" class="slider-btn" id="ki_decrement">-</button>
          <input type="range" id="ki_slider" min="0" max="100" step="1" value="%KI%">
          <button type="button" class="slider-btn" id="ki_increment">+</button>
          <input type="number" id="ki_text" value="%KI%" step="1">
        </div>
      </div>

      <div class="control-row">
        <label for="kd_slider">Derivative</label>
        <div class="control-slider">
          <button type="button" class="slider-btn" id="kd_decrement">-</button>
          <input type="range" id="kd_slider" min="0" max="10" step="0.1" value="%KD%">
          <button type="button" class="slider-btn" id="kd_increment">+</button>
          <input type="number" id="kd_text" value="%KD%" step="1">
        </div>
      </div>

      <div class="section-title">
        <span>System</span>
        <div></div>
      </div>

      <div class="control-row">
        <label for="flt_slider">Low Pass Filter Strength on Pressure Sensor (alpha)</label>
        <div class="control-slider">
          <button type="button" class="slider-btn" id="flt_decrement">-</button>
          <input type="range" id="flt_slider" min="0" max="1" step="0.01" value="%FLT%">
          <button type="button" class="slider-btn" id="flt_increment">+</button>
          <input type="number" id="flt_text" value="%FLT%" step="0.01">
        </div>
      </div>

      <div class="control-row">
        <label for="freq_slider">PWM Frequency in Hz</label>
        <div class="control-slider">
          <button type="button" class="slider-btn" id="freq_decrement">-</button>
          <input type="range" id="freq_slider" min="100" max="10000" step="100" value="%FREQ%">
          <button type="button" class="slider-btn" id="freq_increment">+</button>
          <input type="number" id="freq_text" value="%FREQ%" step="100">
        </div>
      </div>

      <div class="control-row">
        <label for="res_slider">PWM Resolution in bits</label>
        <div class="control-slider">
          <button type="button" class="slider-btn" id="res_decrement">-</button>
          <input type="range" id="res_slider" min="8" max="16" step="1" value="%RES%">
          <button type="button" class="slider-btn" id="res_increment">+</button>
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
              text: 'Time'
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
      if (document.getElementById('chartToggle').checked) 
      {
        pressureChart.update();
      }
    }

    // Toggle chart visibility
    document.getElementById('chartToggle').addEventListener('change', function() 
    {
      const chartContainer = document.getElementById('chartContainer');
      if (this.checked) 
      {
        chartContainer.style.display = 'block';
        // Don't clear the data, just update the chart with existing data
        pressureChart.update();
      } 
      else 
      {
        chartContainer.style.display = 'none';
      }
    });

    // Synchronize range and number inputs, enable Apply button on change
    ["sp", "kp", "ki", "kd", "flt", "freq", "res"].forEach(function(param) 
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
      const slider = document.getElementById(cfg.param + '_slider');
      const text = document.getElementById(cfg.param + '_text');
      const decBtn = document.getElementById(cfg.param + '_decrement');
      const incBtn = document.getElementById(cfg.param + '_increment');
      
      decBtn.addEventListener('click', function() {
        let value = parseFloat(text.value);
        value = Math.max(cfg.min, +(value - cfg.step).toFixed(10));
        text.value = value;
        slider.value = value;
        enableApply();
      });
      incBtn.addEventListener('click', function() {
        let value = parseFloat(text.value);
        value = Math.min(cfg.max, +(value + cfg.step).toFixed(10));
        text.value = value;
        slider.value = value;
        enableApply();
      });
    });

    function enableApply() 
    {
      document.getElementById('applyBtn').disabled = false;
      document.getElementById('applyBtn').textContent = "Apply Changes";
    }

    document.getElementById('applyBtn').addEventListener('click', function() 
    {
      const params = ["sp", "kp", "ki", "kd", "flt", "freq", "res"].map(param => 
      {
        const value = document.getElementById(param + "_text").value;
        return param + "=" + encodeURIComponent(value);
      }).join("&");

      // Show loading state
      const btn = document.getElementById('applyBtn');
      btn.textContent = "Applying...";

      fetch("/set?" + params)
        .then(() => {
          btn.disabled = true;
          btn.textContent = "Changes Applied";
        })
        .catch(err => {
          btn.textContent = "Try Again";
          alert("Failed to apply settings: " + err);
        });
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
        
        // Update trend indicator
        const trendEl = document.getElementById('pressure-trend');
        if (pressureTrend > 0) 
        {
          trendEl.className = 'trend-indicator trend-up';
        } 
        else if (pressureTrend < 0) 
        {
          trendEl.className = 'trend-indicator trend-down';
        } 
        else 
        {
          trendEl.className = 'trend-indicator trend-stable';
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
        
        // Update trend indicator
        const trendEl = document.getElementById('pwm-trend');
        if (pwmTrend > 0) 
        {
          trendEl.className = 'trend-indicator trend-up';
        } 
        else if (pwmTrend < 0) 
        {
          trendEl.className = 'trend-indicator trend-down';
        } 
        else 
        {
          trendEl.className = 'trend-indicator trend-stable';
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
          // Update pressure display with new gauge style
          if (typeof data.pressure !== "undefined") 
          {
            const pressureVal = data.pressure;
            document.getElementById('pressure').textContent = pressureVal.toFixed(2);
            
            // Update pressure fill and calculate percentage (0-10 bar range)
            const pressurePercent = (pressureVal / 10) * 100;
            document.getElementById('pressure-fill').style.width = `${pressurePercent}%`;
            
            // Update setpoint target marker
            const setpointPercent = (data.sp / 10) * 100;
            document.getElementById('pressure-target').style.left = `${setpointPercent}%`;
            
            // Update trend indicator
            updatePressureTrend(pressureVal);
            
            // Get PWM value for chart
            const pwmVal = (data.pwm !== undefined) ? data.pwm : 0;
            
            // Always call updateChart, it will handle visibility internally
            updateChart(pressureVal, data.sp, pwmVal);
          } 
          else 
          {
            document.getElementById('pressure').textContent = "--";
            document.getElementById('pressure-fill').style.width = "0%";
          }
          
          // Update PWM output with new gauge style
          if (data.pwm !== undefined) {
            const pwmVal = parseFloat(data.pwm);
            document.getElementById('pwm').textContent = pwmVal.toFixed(3);
            document.getElementById('pwm-fill').style.width = `${pwmVal}%`;
            
            // Update PWM trend indicator
            updatePwmTrend(pwmVal);
          } 
          else 
          {
            document.getElementById('pwm').textContent = "--";
            document.getElementById('pwm-fill').style.width = "0%";
          }
          
          // Update ADC status with indicator
          const adcIndicator = document.getElementById('adc_indicator');
          if (data.adc_status === "100") {
            document.getElementById('adc_status').textContent = "External ADS1015 ADC - 12 Bit";
            adcIndicator.style.backgroundColor = 'var(--success)';
          } 
          else if(data.adc_status === "000") {
            document.getElementById('adc_status').textContent = "Internal ESP32 ADC - 12 Bit";
            adcIndicator.style.backgroundColor = 'var(--accent)';
          } 
          else 
          {
            document.getElementById('adc_status').textContent = "Unknown";
            adcIndicator.style.backgroundColor = 'var(--danger)';
          }
          
          // Update Network status with indicator
          const networkIndicator = document.getElementById('network_indicator');
          document.getElementById('network_status').textContent = "VENTCON_AP, IP: 192.168.4.1";
          networkIndicator.style.backgroundColor = 'var(--success)';
          
          // Only update UI controls if Apply button is disabled (no unsaved changes)
          if (document.getElementById('applyBtn').disabled) 
          {
            // Update all sliders and inputs with current values from server
            if (typeof data.sp !== "undefined") {
              document.getElementById('sp_slider').value = data.sp;
              document.getElementById('sp_text').value = data.sp;
            }

            if (typeof data.kp !== "undefined") {
              document.getElementById('kp_slider').value = data.kp;
              document.getElementById('kp_text').value = data.kp;
            }

            if (typeof data.ki !== "undefined") {
              document.getElementById('ki_slider').value = data.ki;
              document.getElementById('ki_text').value = data.ki;
            }

            if (typeof data.kd !== "undefined") {
              document.getElementById('kd_slider').value = data.kd;
              document.getElementById('kd_text').value = data.kd;
            }

            if (typeof data.flt !== "undefined") {
              document.getElementById('flt_slider').value = data.flt;
              document.getElementById('flt_text').value = data.flt;
            }

            if (typeof data.freq !== "undefined") {
              document.getElementById('freq_slider').value = data.freq;
              document.getElementById('freq_text').value = data.freq;
            }

            if (typeof data.res !== "undefined") {
              document.getElementById('res_slider').value = data.res;
              document.getElementById('res_text').value = data.res;
            }
          }
        })
        .catch(err => 
        {
          console.error("Error fetching values:", err);
        });
    }, 300);

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