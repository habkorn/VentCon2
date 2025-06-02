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
  <!-- Add Chart.js and required adapters from CDN -->
  <script src="https://cdn.jsdelivr.net/npm/chart.js@3.9.1/dist/chart.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/moment@2.29.4/moment.min.js"></script>
  <script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-moment@1.0.1/dist/chartjs-adapter-moment.min.js"></script>
  <style>
)rawliteral";

// Concatenate parts of the HTML
const char HTML_CONTENT_AFTER_STYLE[] PROGMEM = R"rawliteral(
  </style>
</head>

<body>
  <header>
    <h1>
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
          <input type="range" id="sp_slider" min="0" max="10" step="0.1" value="%SP%">
          <input type="number" id="sp_text" value="%SP%" step="0.1">
          <span>bar</span>
        </div>
      </div>

      <div class="section-title">
        <span>PID</span>
        <div></div>
      </div>

      <div class="control-row">
        <label for="kp_slider">Proportional</label>
        <div class="control-slider">
          <input type="range" id="kp_slider" min="0" max="100" step="0.1" value="%KP%">
          <input type="number" id="kp_text" value="%KP%" step="0.1">
        </div>
      </div>

      <div class="control-row">
        <label for="ki_slider">Integral</label>
        <div class="control-slider">
          <input type="range" id="ki_slider" min="0" max="10" step="0.01" value="%KI%">
          <input type="number" id="ki_text" value="%KI%" step="0.01">
        </div>
      </div>

      <div class="control-row">
        <label for="kd_slider">Derivative</label>
        <div class="control-slider">
          <input type="range" id="kd_slider" min="0" max="1" step="0.01" value="%KD%">
          <input type="number" id="kd_text" value="%KD%" step="0.01">
        </div>
      </div>

      <div class="section-title">
        <span>System</span>
        <div></div>
      </div>

      <div class="control-row">
        <label for="flt_slider">Low Pass Filter Strength on Pressure Sensor (alpha)</label>
        <div class="control-slider">
          <input type="range" id="flt_slider" min="0" max="1" step="0.01" value="%FLT%">
          <input type="number" id="flt_text" value="%FLT%" step="0.01">
        </div>
      </div>

      <div class="control-row">
        <label for="freq_slider">PWM Frequency in Hz</label>
        <div class="control-slider">
          <input type="range" id="freq_slider" min="100" max="10000" step="100" value="%FREQ%">
          <input type="number" id="freq_text" value="%FREQ%" step="100">
          <span>Hz</span>
        </div>
      </div>

      <div class="control-row">
        <label for="res_slider">PWM Resolution in bit</label>
        <div class="control-slider">
          <input type="range" id="res_slider" min="8" max="16" step="1" value="%RES%">
          <input type="number" id="res_text" value="%RES%" step="1">
          <span>bits</span>
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
    <p>VENTCON Control System v2.0</p>
  </footer>

  <script>
    // Initialize the chart
    const ctx = document.getElementById('pressureChart').getContext('2d');
    let pressureData = [];
    let setpointData = [];
    
    // Create chart instance
    const pressureChart = new Chart(ctx, {
      type: 'line',
      data: {
        datasets: [
          {
            label: 'Pressure',
            data: pressureData,
            borderColor: '#2563eb',
            backgroundColor: 'rgba(37, 99, 235, 0.1)',
            tension: 0.3,
            borderWidth: 2,
            pointRadius: 2,
            pointHoverRadius: 4,
            pointBorderWidth: 1,
            pointStyle: 'circle'
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
            pointHoverRadius: 3
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
            max: 10,
            title: {
              display: true,
              text: 'Pressure (bar)'
            },
            ticks: {
              callback: function(value) {
                return value + ' bar';
              }
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
            }
          }
        },
        plugins: {
          legend: {
            position: 'top',
            labels: {
              boxWidth: 25,
              usePointStyle: false, // Changed to false to show lines instead of circles
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
                return context.dataset.label + ': ' + context.parsed.y.toFixed(2) + ' bar';
              }
            }
          }
        }
      }
    });

    // Function to add data to the chart
    function updateChart(pressure, setpoint) 
    {
      if (!document.getElementById('chartToggle').checked) 
      {
        return;
      }
      
      const now = new Date();
      
      pressureData.push({
        x: now,
        y: pressure
      });
      
      setpointData.push({
        x: now,
        y: setpoint
      });
      
      if (pressureData.length > 30) 
      {
        pressureData.shift();
        setpointData.shift();
      }
      
      pressureChart.update();
    }

    // Toggle chart visibility
    document.getElementById('chartToggle').addEventListener('change', function() 
    {
      const chartContainer = document.getElementById('chartContainer');
      if (this.checked) 
      {
        chartContainer.style.display = 'block';
        pressureData = [];
        setpointData = [];
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
            
            // Still update the chart
            updateChart(pressureVal, data.sp);
          } 
          else 
          {
            document.getElementById('pressure').textContent = "--";
            document.getElementById('pressure-fill').style.width = "0%";
          }
          
          // Update PWM output with new gauge style
          if (data.pwm !== undefined) {
            const pwmVal = data.pwm.toFixed(1);
            document.getElementById('pwm').textContent = pwmVal;
            document.getElementById('pwm-fill').style.width = `${pwmVal}%`;
          } 
          else 
          {
            document.getElementById('pwm').textContent = "--";
            document.getElementById('pwm-fill').style.width = "0%";
          }
          
          // Update ADC status with indicator
          const adcIndicator = document.getElementById('adc_indicator');
          if (data.adc_status === "100") {
            document.getElementById('adc_status').textContent = "External ADS1015 ADC";
            adcIndicator.style.backgroundColor = 'var(--success)';
          } 
          else if(data.adc_status === "000") {
            document.getElementById('adc_status').textContent = "Internal ESP32 ADC";
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