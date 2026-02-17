#pragma once
#include <Arduino.h>

// ============================================================================
// Section 5: JavaScript and closing HTML tags (static, ~25KB)
// ============================================================================

const char HTML_SCRIPT[] PROGMEM = R"rawliteral(
  <script defer>
    // Cached DOM elements
    let cachedElements = {};
    
    // Centralized parameter configuration: URL param name → JSON key in default.json
    const PARAM_CONFIG = {
      sp:   { jsonKey: 'setpoint' },
      kp:   { jsonKey: 'Kp' },
      ki:   { jsonKey: 'Ki' },
      kd:   { jsonKey: 'Kd' },
      flt:  { jsonKey: 'filter_strength' },
      freq: { jsonKey: 'pwm_freq' },
      res:  { jsonKey: 'pwm_res' }
    };
    const SLIDER_PARAMS = Object.keys(PARAM_CONFIG);
    const CONFIGURABLE_SLIDERS = ['sp', 'kp', 'ki', 'kd'];
    
    // Timing constants (shared between chart and data polling)
    const POLLING_INTERVAL_MS = 100;  // Data fetch interval
    const DISPLAY_WINDOW_S = 8;       // Seconds of chart history to display

    // Helper to get CSS variable value (as rgb hex)
    function getCssVar(name) {
      return getComputedStyle(document.documentElement).getPropertyValue(name).trim();
    }
    // Helper to get rgba from CSS var and alpha
    function cssVarRgba(name, alpha) {
      const hex = getCssVar(name).replace('#', '');
      if (hex.length === 6) {
        const r = parseInt(hex.substring(0,2), 16);
        const g = parseInt(hex.substring(2,4), 16);
        const b = parseInt(hex.substring(4,6), 16);
        return `rgba(${r},${g},${b},${alpha})`;
      }
      return getCssVar(name);
    }
    
    // Cache DOM elements on load
    function cacheElements()
    {
      cachedElements = {
        loader: document.getElementById('loader'),
        pressure: document.getElementById('pressure'),
        pressureFill: document.getElementById('pressure-fill'),
        pressureTarget: document.getElementById('pressure-target'),
        pressureTrend: document.getElementById('pressure-trend'),
        pwm: document.getElementById('pwm'),
        pwmFill: document.getElementById('pwm-fill'),
        pwmTrend: document.getElementById('pwm-trend'),
        networkIndicator: document.getElementById('network_indicator'),
        networkStatus: document.getElementById('network_status'),
        chartToggle: document.getElementById('chartToggle'),
        chartContainer: document.getElementById('chartContainer'),
        saveSnackbar: document.getElementById('saveSnackbar'),
        saveSnackbarText: document.getElementById('saveSnackbarText'),
        // Cache modal elements
        modal: document.getElementById('sliderSettingsModal'),
        modalTitle: document.getElementById('modalTitle'),
        modalMin: document.getElementById('modalMin'),
        modalMax: document.getElementById('modalMax'),
        modalStep: document.getElementById('modalStep'),
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
        },
        tiDisplay: document.getElementById('ti_display'),
        tdDisplay: document.getElementById('td_display'),
        // Cache sensor calibration inputs
        sensorInputs: {
          sensor_minP: document.getElementById('sensor_minP'),
          sensor_maxP: document.getElementById('sensor_maxP'),
          sensor_minV: document.getElementById('sensor_minV'),
          sensor_maxV: document.getElementById('sensor_maxV')
        }
      };
    }

    // Wait for all deferred scripts to load before initializing
    let initAttempts = 0;
    const MAX_INIT_ATTEMPTS = 100; // 5 seconds max wait (100 * 50ms)

    // Track loader display time for minimum duration
    const MIN_LOADER_TIME = 3000; // ms
    let loaderShownAt = Date.now();
    
    // Initialize chart update tracking variables globally
    window.lastChartUpdate = 0;
    window.chartUpdateInterval = 200;
    
    function initializeApp()
    {
      initAttempts++;
      
      // Check if Chart.js is available
      if (typeof Chart === 'undefined')
      {
        if (initAttempts < MAX_INIT_ATTEMPTS)
        {
          setTimeout(initializeApp, 50);
          return;
        }
        else
        {
          // Chart.js failed to load, proceed without chart
          console.error('Chart.js failed to load after 5 seconds');
          cacheElements();
          hideLoaderAndContinue();
          if(cachedElements.chartContainer) cachedElements.chartContainer.style.display = 'none';
          setupEventListeners();
          startDataUpdates();
          return;
        }
      }

      cacheElements();
      hideLoaderAndContinue();

      // Initialize the chart after Chart.js is loaded
      initializeChart();

      // Setup all other functionality
      setupEventListeners();
      updateTimeConstants();
      startDataUpdates();

    }

    // Hide loader after minimum display time
    function hideLoaderAndContinue() {
      const elapsed = Date.now() - loaderShownAt;
      const remaining = MIN_LOADER_TIME - elapsed;
      if (remaining > 0) {
        setTimeout(() => {
          if (cachedElements.loader) cachedElements.loader.style.display = 'none';
        }, remaining);
      } else {
        if (cachedElements.loader) cachedElements.loader.style.display = 'none';
      }
    }
    
    // Initialize chart in separate function
    function initializeChart()
    {
      const ctx = document.getElementById('pressureChart');
      if (!ctx) return;
      
      const chartCtx = ctx.getContext('2d');
      
      // Efficient circular buffer implementation
      const BUFFER_SIZE = 128;         // Power of 2 >= DISPLAY_SIZE
      const DISPLAY_SIZE = Math.round(DISPLAY_WINDOW_S / (POLLING_INTERVAL_MS / 1000)); // 80 points
      
      // Create circular buffers with object pooling
      window.chartData = {
        pressure: new Array(BUFFER_SIZE),
        setpoint: new Array(BUFFER_SIZE),
        pwm: new Array(BUFFER_SIZE),
        currentIndex: 0,
        count: 0,
        
        // Object pool for data points to reduce GC pressure
        pointPool: [],
        poolIndex: 0,
        
        // Get a pooled point object
        getPoint: function(x, y)
        {
          if (this.poolIndex >= this.pointPool.length)
          {
            this.pointPool.push({ x: null, y: null });
          }
          const point = this.pointPool[this.poolIndex++];
          point.x = x;
          point.y = y;
          return point;
        },
        
        // Reset pool for reuse
        resetPool: function()
        {
          this.poolIndex = 0;
        },
        
        // Add data point efficiently
        addData: function(pressure, setpoint, pwm)
        {
          this.pressure[this.currentIndex] = pressure;
          this.setpoint[this.currentIndex] = setpoint;
          this.pwm[this.currentIndex] = pwm;
          
          this.currentIndex = (this.currentIndex + 1) % BUFFER_SIZE;
          if (this.count < BUFFER_SIZE) this.count++;
        },
        
        // Get display data with elapsed time (0 = now, negative = past)
        getDisplayData: function()
        {
          this.resetPool();
          
          const displayCount = Math.min(this.count, DISPLAY_SIZE);
          
          const timeStep = POLLING_INTERVAL_MS / 1000; // Derive from polling interval
          
          const pressureData = [];
          const setpointData = [];
          const pwmData = [];
          
          for (let i = 0; i < displayCount; i++)
          {
            const bufferIndex = (this.currentIndex - displayCount + i + BUFFER_SIZE) % BUFFER_SIZE;
            const elapsed = -(displayCount - i - 1) * timeStep; // seconds ago (negative)
            
            pressureData.push(this.getPoint(elapsed, this.pressure[bufferIndex]));
            setpointData.push(this.getPoint(elapsed, this.setpoint[bufferIndex]));
            pwmData.push(this.getPoint(elapsed, this.pwm[bufferIndex]));
          }
          
          return { pressureData, setpointData, pwmData };
        }
      };
      
      // Create chart instance with optimized configuration
      window.pressureChart = new Chart(chartCtx, {
        type: 'line',
        data: {
          datasets: [
            {
              label: 'Outlet',
              data: [],
              borderColor: getCssVar('--primary'),
              backgroundColor: cssVarRgba('--primary', 0.1),
              tension: 0.3,
              borderWidth: 2,
              pointRadius: 0,
              yAxisID: 'y'
            },
            {
              label: 'Setpoint',
              data: [],
              borderColor: getCssVar('--accent'),
              backgroundColor: cssVarRgba('--accent', 0.1),
              borderDash: [5, 5],
              tension: 0.1,
              borderWidth: 2,
              pointRadius: 0,
              yAxisID: 'y'
            },
            {
              label: 'Valve Duty Cycle',
              data: [],
              borderColor: getCssVar('--success'),
              backgroundColor: cssVarRgba('--success', 0.1),
              tension: 0.3,
              borderWidth: 2,
              pointRadius: 0,
              yAxisID: 'pwm'
            }
          ]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: false,
          events: [],
          interaction: {
            intersect: false,
            mode: 'index'
          },
          elements: {
            point: {
              radius: 0
            }
          },
          scales: {
            y: {
              beginAtZero: true,
              min: 0,
              max: CFG.maxBar,
              position: 'left',
              title: {
                display: true,
                text: 'Outlet Pressure in bar(g)',
                color: getCssVar('--primary')
              },
              ticks: {
                color: getCssVar('--primary'),
                stepSize: 2,
                autoSkip: false
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
                text: 'Valve Duty Cycle (%)',
                color: getCssVar('--success')
              },
              ticks: {
                color: getCssVar('--success'),
                stepSize: 20,
                autoSkip: false
              },
              grid: {
                display: false
              }
            },
            x: {
              type: 'linear',
              min: -DISPLAY_WINDOW_S,
              max: 0,
              title: {
                display: false,
                text: 'Elapsed Time (s)',
                font: {
                  size: 8,
                  family: 'courier, monospace',
                  weight: 'bold'
                }
              },
              ticks: {
                color: getCssVar('--text'),
                stepSize: 1,
                autoSkip: false,
                font: {
                  size: 8,
                  family: 'courier, monospace',
                  weight: 'bold'
                },
                maxRotation: 0,
                minRotation: 0,
                callback: function(value) {
                  return value === 0 ? 'now' : value + 's';
                }
              },
              grid: {
                display: true
              }
            }
          },
          plugins: {
            legend: {
              display: false,
              position: 'top',
              labels: {
                boxWidth: 25,
                boxHeight:0,
                usePointStyle: false,
                generateLabels: function(chart) {
                  const original = Chart.defaults.plugins.legend.labels.generateLabels(chart);
                  original.forEach(label => {
                    if (label.text === 'Setpoint') {
                      label.lineDash = [5, 5];
                    }
                  });
                  return original;
                }
              }
            },
            tooltip: {
              enabled: false,
              backgroundColor: 'rgba(255,255,255,0.9)',
              titleColor: getCssVar('--text'),
              bodyColor: getCssVar('--text'),
              borderColor: getCssVar('--border'),
              borderWidth: 1,
              cornerRadius: 6,
              displayColors: true,
              callbacks: {
                label: function(context)
                {
                  const label = context.dataset.label;
                  const value = context.parsed.y.toFixed(3);
                  if (label === 'Valve Duty Cycle')
                  {
                    return label + ': ' + value + '%';
                  }
                  return label + ': ' + value + ' bar';
                }
              }
            }
          }
        }
      });
      
      // Chart update tracking already initialized globally
    }
    
    // Setup all event listeners
    function setupEventListeners()
    {
      // Chart toggle functionality
      if (cachedElements.chartToggle && cachedElements.chartContainer)
      {
        cachedElements.chartToggle.addEventListener('change', function()
        {
          if (!cachedElements.chartContainer || !window.pressureChart) return;
          
          if (this.checked)
          {
            cachedElements.chartContainer.style.display = 'block';
            
            // Force immediate update when showing chart
            if (window.chartData)
            {
              const displayData = window.chartData.getDisplayData();
              window.pressureChart.data.datasets[0].data = displayData.pressureData;
              window.pressureChart.data.datasets[1].data = displayData.setpointData;
              window.pressureChart.data.datasets[2].data = displayData.pwmData;
              window.pressureChart.update('none');
              window.lastChartUpdate = Date.now();
            }
          }
          else
          {
            cachedElements.chartContainer.style.display = 'none';
          }
        });
      }
        // Setup slider and text input synchronization
      const PID_PARAMS = ['kp', 'ki', 'kd'];
      SLIDER_PARAMS.forEach(function(param)
      {
        const slider = cachedElements.sliders ? cachedElements.sliders[param] : null;
        const text = cachedElements.texts ? cachedElements.texts[param] : null;
        const isPid = PID_PARAMS.indexOf(param) !== -1;
        
        if (slider && text)
        {
          slider.addEventListener('input', function()
          {
            text.value = slider.value;
            if (isPid) updateTimeConstants();
            showSaveSnackbar(param, slider.value);
          });

          text.addEventListener('input', function()
          {
            const val = parseFloat(text.value);
            const lo = parseFloat(slider.min);
            const hi = parseFloat(slider.max);
            if (!isNaN(val) && val >= lo && val <= hi)
            {
              setInputError(text, null);
              slider.value = val;
            }
            else
            {
              setInputError(text, isNaN(val) ? 'Enter a number' : 'Range: ' + lo + ' – ' + hi);
            }
            if (isPid) updateTimeConstants();
            validationOk = !text.classList.contains('input-error');
            if (validationOk) showSaveSnackbar(param, text.value);
          });

          // On blur: clamp value to slider range and clear any error
          text.addEventListener('blur', function()
          {
            clampTextToSlider(text, slider);
            if (isPid) updateTimeConstants();
            validationOk = true;
          });
        }
      });

      // Setup increment/decrement buttons
      // For sp, kp, ki, kd: use dynamic limits from slider attributes (step is user-configurable)
      // For flt, freq, res: use fixed values
      function adjustValue(cfg, direction)
      {
        const slider = cachedElements.sliders ? cachedElements.sliders[cfg.param] : null;
        const text = cachedElements.texts ? cachedElements.texts[cfg.param] : null;
        if (!slider || !text) return;

        const limit = cfg.fixed
          ? (direction > 0 ? cfg.max : cfg.min)
          : parseFloat(direction > 0 ? slider.max : slider.min);
        const step = cfg.fixed ? cfg.step : parseFloat(slider.step);
        let value = parseFloat(text.value);
        value = direction > 0
          ? Math.min(limit, +(value + step).toFixed(10))
          : Math.max(limit, +(value - step).toFixed(10));
        text.value = value;
        slider.value = value;
        if (PID_PARAMS.indexOf(cfg.param) !== -1) updateTimeConstants();
        showSaveSnackbar(cfg.param, value);
      }

      [
        {param: 'sp'},
        {param: 'kp'},
        {param: 'ki'},
        {param: 'kd'},
        {param: 'flt', min: CFG.flt.min, max: CFG.flt.max, step: CFG.flt.step, fixed: true},
        {param: 'freq', min: CFG.freq.min, max: CFG.freq.max, step: CFG.freq.step, fixed: true},
        {param: 'res', min: CFG.res.min, max: CFG.res.max, step: CFG.res.step, fixed: true}
      ].forEach(function(cfg)
      {
        const decBtn = document.getElementById(cfg.param + '_decrement');
        const incBtn = document.getElementById(cfg.param + '_increment');
        
        if (decBtn && incBtn)
        {
          decBtn.addEventListener('click', function() { adjustValue(cfg, -1); });
          incBtn.addEventListener('click', function() { adjustValue(cfg, 1); });
        }
      });
      
      // Setup sensor calibration input change listeners with cross-validation
      const SENSOR_PARAMS = ['sensor_minP', 'sensor_maxP', 'sensor_minV', 'sensor_maxV'];
      SENSOR_PARAMS.forEach(function(param)
      {
        const input = cachedElements.sensorInputs ? cachedElements.sensorInputs[param] : null;
        if (input)
        {
          input.addEventListener('input', function()
          {
            const sensorValid = validateSensorInputs();
            validationOk = sensorValid;
            if (sensorValid) showSaveSnackbar(param, input.value);
          });
        }
      });
      
      // Setup save snackbar click handler
      if (cachedElements.saveSnackbar && cachedElements.saveSnackbarText)
      {
        cachedElements.saveSnackbar.addEventListener('click', handleSaveClick);
      }
      
      // Setup PID reset button
      const resetBtn = document.getElementById('resetPidBtn');
      if (resetBtn)
      {
        resetBtn.addEventListener('click', handlePidReset);
      }
      
      // Setup Reset to Default button
      const resetDefaultsBtn = document.getElementById('resetDefaultsBtn');
      if (resetDefaultsBtn)
      {
        resetDefaultsBtn.addEventListener('click', handleResetToDefaults);
      }
      
      // Keep snackbar visible when mobile keyboard / address bar changes viewport
      if (window.visualViewport && cachedElements.saveSnackbar)
      {
        window.visualViewport.addEventListener('resize', function()
        {
          const offset = window.innerHeight - window.visualViewport.height + window.visualViewport.offsetTop;
          cachedElements.saveSnackbar.style.bottom = (24 + offset) + 'px';
        });
        window.visualViewport.addEventListener('scroll', function()
        {
          const offset = window.innerHeight - window.visualViewport.height + window.visualViewport.offsetTop;
          cachedElements.saveSnackbar.style.bottom = (24 + offset) + 'px';
        });
      }

      // Setup Easter egg
      setupEasterEgg();
      
      // Setup scroll handler
      setupScrollHandler();
    }
    
    // Connection tracking for disconnect detection
    let fetchInFlight = false;
    let consecutiveFailures = 0;
    const MAX_FAILURES_BEFORE_DISCONNECT = 5;

    // Start periodic data updates using setTimeout chaining to prevent request pileup
    function startDataUpdates()
    {
      scheduleNextUpdate();
    }

    function scheduleNextUpdate()
    {
      setTimeout(updateData, POLLING_INTERVAL_MS);
    }

    // Data update function
    function updateData()
    {
      if (fetchInFlight) { scheduleNextUpdate(); return; }
      fetchInFlight = true;

      fetch('/values')
        .then(r => r.json())
        .then(data =>
        {
          fetchInFlight = false;
          consecutiveFailures = 0;

          // Show connected status
          if (cachedElements.networkIndicator)
          {
            cachedElements.networkIndicator.style.backgroundColor = 'var(--success)';
          }
          if (cachedElements.networkStatus)
          {
            cachedElements.networkStatus.textContent = CFG.ssid + ', IP: ' + CFG.ip;
          }
          // Update pressure display with cached elements
          if (typeof data.pressure !== "undefined")
          {
            const pressureVal = data.pressure;
            if (cachedElements.pressure)
            {
              cachedElements.pressure.textContent = pressureVal.toFixed(2);
            }
            
            // Update pressure fill and calculate percentage (0-maxBar range)
            const pressurePercent = (pressureVal / CFG.maxBar) * 100;
            if (cachedElements.pressureFill)
            {
              cachedElements.pressureFill.style.width = `${pressurePercent}%`;
            }
            
            // Update setpoint target marker
            const setpointPercent = (data.sp / CFG.maxBar) * 100;
            if (cachedElements.pressureTarget)
            {
              cachedElements.pressureTarget.style.left = `${setpointPercent}%`;
            }
            
            // Update trend indicator
            updateTrend('pressure', pressureVal);
            
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
          if (data.pwm !== undefined)
          {
            const pwmVal = parseFloat(data.pwm);
            if (cachedElements.pwm)
            {
              cachedElements.pwm.textContent = pwmVal.toFixed(3);
            }
            if (cachedElements.pwmFill)
            {
              cachedElements.pwmFill.style.width = `${pwmVal}%`;
            }
            
            // Update PWM trend indicator
            updateTrend('pwm', pwmVal);
          }
          else
          {
            if (cachedElements.pwm) cachedElements.pwm.textContent = "--";
            if (cachedElements.pwmFill) cachedElements.pwmFill.style.width = "0%";
          }
          
          // Update Network status with cached elements
          // (handled above in connection tracking)
          
          // Only update UI controls if no unsaved changes (saveSnackbar is hidden)
          if (cachedElements.saveSnackbar && cachedElements.saveSnackbar.style.display === 'none')
          {
            // Update all sliders and inputs with current values from server using cached elements
            SLIDER_PARAMS.forEach(param =>
            {
              if (typeof data[param] !== "undefined")
              {
                if (cachedElements.sliders && cachedElements.sliders[param])
                {
                  cachedElements.sliders[param].value = data[param];
                }
                if (cachedElements.texts && cachedElements.texts[param])
                {
                  cachedElements.texts[param].value = data[param];
                }
              }
            });
          }

          scheduleNextUpdate();
        })
        .catch(err =>
        {
          fetchInFlight = false;
          consecutiveFailures++;
          console.error("Error fetching values:", err);

          // Show disconnected status after repeated failures
          if (consecutiveFailures >= MAX_FAILURES_BEFORE_DISCONNECT)
          {
            if (cachedElements.networkIndicator)
            {
              cachedElements.networkIndicator.style.backgroundColor = getCssVar('--danger') || '#ef4444';
            }
            if (cachedElements.networkStatus)
            {
              cachedElements.networkStatus.textContent = 'Disconnected';
            }
          }

          scheduleNextUpdate();
        });
    }

    // Hide loader as soon as DOM is interactive
    document.addEventListener('DOMContentLoaded', initializeApp);

    // Function to add data to the chart (optimized)
    function updateChart(pressure, setpoint, pwm)
    {
      if (!window.pressureChart || !window.chartData) return;
      
      const now = Date.now();
      
      // Always collect data in efficient circular buffer
      window.chartData.addData(pressure, setpoint, pwm);
      
      // Only update chart display if visible and enough time has passed
      if (cachedElements.chartToggle && cachedElements.chartToggle.checked)
      {
        if (now - window.lastChartUpdate >= window.chartUpdateInterval)
        {
          // Get optimized display data
          const displayData = window.chartData.getDisplayData();
          
          // Update chart datasets efficiently
          window.pressureChart.data.datasets[0].data = displayData.pressureData;
          window.pressureChart.data.datasets[1].data = displayData.setpointData;
          window.pressureChart.data.datasets[2].data = displayData.pwmData;
          
          // Use 'none' mode for no animations - fastest update
          window.pressureChart.update('none');
          window.lastChartUpdate = now;
        }
      }
    }

    // Unified trend tracker for pressure and PWM
    const trendTracker = {
      pressure: { values: [], trend: 0, threshold: 0.05 },
      pwm: { values: [], trend: 0, threshold: 0.5 }
    };
    
    // Update trend indicator for any metric
    function updateTrend(type, newValue)
    {
      const tracker = trendTracker[type];
      const element = type === 'pressure' ? cachedElements.pressureTrend : cachedElements.pwmTrend;
      if (!tracker || !element) return;
      
      // Keep last 3 values for trend calculation
      if (tracker.values.length >= 3) tracker.values.shift();
      tracker.values.push(newValue);
      
      // Calculate trend only when we have enough values
      if (tracker.values.length >= 3)
      {
        const diff = tracker.values[tracker.values.length - 1] - tracker.values[0];
        
        if (diff > tracker.threshold)
        {
          tracker.trend = 1;
          element.className = 'trend-indicator trend-up';
        }
        else if (diff < -tracker.threshold)
        {
          tracker.trend = -1;
          element.className = 'trend-indicator trend-down';
        }
        else
        {
          tracker.trend = 0;
          element.className = 'trend-indicator trend-stable';
        }
      }
    }

    // Track changes to show the save snackbar
    let pendingChanges = {};
    let changeTimeout;
    
    // Reusable button feedback helper: shows temporary text/color then reverts
    function showButtonFeedback(btn, text, color, revertDelay, originalText, originalColor, onRevert)
    {
      btn.textContent = text;
      btn.style.backgroundColor = color;
      setTimeout(() =>
      {
        btn.textContent = originalText;
        btn.style.backgroundColor = originalColor || '';
        if (onRevert) onRevert();
      }, revertDelay);
    }

    // Reusable snackbar fade-out animation
    function fadeOutSnackbar(callback)
    {
      if (!cachedElements.saveSnackbar) return;
      if (cachedElements.saveSnackbar.animate)
      {
        cachedElements.saveSnackbar.animate([{ opacity: 1 }, { opacity: 0 }], { duration: 300, easing: 'ease-out' });
        setTimeout(() =>
        {
          if (cachedElements.saveSnackbar) cachedElements.saveSnackbar.style.display = 'none';
          if (callback) callback();
        }, 300);
      }
      else
      {
        cachedElements.saveSnackbar.style.display = 'none';
        if (callback) callback();
      }
    }
    
    // Reset snackbar to default state
    function resetSnackbar()
    {
      if (cachedElements.saveSnackbar)
      {
        cachedElements.saveSnackbar.style.background = getCssVar('--primary');
        cachedElements.saveSnackbar.style.pointerEvents = 'auto';
      }
      if (cachedElements.saveSnackbarText)
      {
        cachedElements.saveSnackbarText.textContent = 'Apply Changes';
      }
    }

    // ── Input Validation Helpers ──────────────────────────────────────────
    // Track whether all pending changes are valid (blocks save when false)
    let validationOk = true;

    // Set or clear error state on a single input element
    function setInputError(input, errorMsg)
    {
      if (!input) return;
      if (errorMsg)
      {
        input.classList.add('input-error');
        input.title = errorMsg;
      }
      else
      {
        input.classList.remove('input-error');
        input.title = '';
      }
    }

    // Cross-validate all four sensor calibration inputs.
    // Returns true when every field is valid.
    function validateSensorInputs()
    {
      if (!cachedElements.sensorInputs) return true;
      const si = cachedElements.sensorInputs;
      const minP = parseFloat(si.sensor_minP.value);
      const maxP = parseFloat(si.sensor_maxP.value);
      const minV = parseFloat(si.sensor_minV.value);
      const maxV = parseFloat(si.sensor_maxV.value);
      let ok = true;

      // Min pressure must be a number and < max pressure
      if (isNaN(minP))            { setInputError(si.sensor_minP, 'Enter a valid number');           ok = false; }
      else if (minP >= maxP)      { setInputError(si.sensor_minP, 'Must be less than Max Pressure'); ok = false; }
      else                        { setInputError(si.sensor_minP, null); }

      // Max pressure must be a number and > min pressure
      if (isNaN(maxP))            { setInputError(si.sensor_maxP, 'Enter a valid number');              ok = false; }
      else if (maxP <= minP)      { setInputError(si.sensor_maxP, 'Must be greater than Min Pressure'); ok = false; }
      else                        { setInputError(si.sensor_maxP, null); }

      // Min voltage must be >= 0 and < max voltage
      if (isNaN(minV))            { setInputError(si.sensor_minV, 'Enter a valid number');           ok = false; }
      else if (minV < 0)          { setInputError(si.sensor_minV, 'Cannot be negative');             ok = false; }
      else if (minV >= maxV)      { setInputError(si.sensor_minV, 'Must be less than Max Voltage');  ok = false; }
      else                        { setInputError(si.sensor_minV, null); }

      // Max voltage must be > min voltage and <= 5.0V (ADC limit)
      if (isNaN(maxV))            { setInputError(si.sensor_maxV, 'Enter a valid number');              ok = false; }
      else if (maxV <= minV)      { setInputError(si.sensor_maxV, 'Must be greater than Min Voltage');  ok = false; }
      else if (maxV > 5.0)        { setInputError(si.sensor_maxV, 'Cannot exceed 5.0 V (ADC limit)');   ok = false; }
      else                        { setInputError(si.sensor_maxV, null); }

      return ok;
    }

    // Clamp a slider text input value to its slider's min/max on blur.
    // Also rejects non-numeric input by reverting to the slider's current value.
    function clampTextToSlider(text, slider)
    {
      let val = parseFloat(text.value);
      if (isNaN(val))
      {
        text.value = slider.value;
        setInputError(text, null);
        return;
      }
      const lo = parseFloat(slider.min);
      const hi = parseFloat(slider.max);
      val = Math.max(lo, Math.min(hi, val));
      text.value = val;
      slider.value = val;
      setInputError(text, null);
    }

    // Format time constant with appropriate unit (s or ms)
    function formatTimeConst(seconds)
    {
      if (!isFinite(seconds) || seconds <= 0) return '';
      if (seconds < 1) return  (seconds * 1000).toFixed(1) + ' ms';
      return  seconds.toFixed(3) + ' s';
    }

    // Update Ti and Td displays from current Kp, Ki, Kd values
    function updateTimeConstants()
    {
      const kp = parseFloat(cachedElements.texts.kp.value) || 0;
      const ki = parseFloat(cachedElements.texts.ki.value) || 0;
      const kd = parseFloat(cachedElements.texts.kd.value) || 0;

      if (cachedElements.tiDisplay)
      {
        cachedElements.tiDisplay.textContent = (ki > 0 && kp > 0)
          ? '(Ti=' + formatTimeConst(kp / ki) + ')' : '';
      }
      if (cachedElements.tdDisplay)
      {
        cachedElements.tdDisplay.textContent = (kp > 0 && kd > 0)
          ? '(Td=' + formatTimeConst(kd / kp) + ')' : '';
      }
    }

    // Snackbar management functions
    function showSaveSnackbar(param, value)
    {
      if (!cachedElements.saveSnackbar) return;
      
      // Store the changed parameter
      pendingChanges[param] = value;
      
      // Show the save snackbar
      cachedElements.saveSnackbar.style.display = 'block';
      
      // Add animation for a subtle bounce effect with null check
      if (cachedElements.saveSnackbar.animate)
      {
        cachedElements.saveSnackbar.animate([
          { transform: 'translateX(-50%) scale(0.95)' },
          { transform: 'translateX(-50%) scale(1.02)' },
          { transform: 'translateX(-50%) scale(1)' }
        ], { duration: 300, easing: 'ease-out' });
      }
      
      // Auto-hide after 8 seconds of inactivity
      clearTimeout(changeTimeout);
      changeTimeout = setTimeout(() => fadeOutSnackbar(), 8000);
    }
    
    // Handle save button click
    function handleSaveClick()
    {
      if (!cachedElements.saveSnackbarText) return;

      // Block save if any input has a validation error
      if (!validationOk)
      {
        cachedElements.saveSnackbarText.textContent = "Fix errors first";
        cachedElements.saveSnackbar.style.background = getCssVar('--danger');
        setTimeout(() => resetSnackbar(), 1500);
        return;
      }
      
      cachedElements.saveSnackbarText.textContent = "Saving...";
      cachedElements.saveSnackbar.style.pointerEvents = 'none';
      
      // Build parameters string from pending changes
      const params = Object.entries(pendingChanges).map(([param, value]) => 
        param + "=" + encodeURIComponent(value)
      ).join("&");

      fetch("/set?" + params)
        .then(() =>
        {
          if (cachedElements.saveSnackbarText && cachedElements.saveSnackbar)
          {
            cachedElements.saveSnackbarText.textContent = "Settings Updated";
            cachedElements.saveSnackbar.style.background = getCssVar('--success'); // Success green
            
            // Clear pending changes
            pendingChanges = {};
            
            // Hide snackbar after a short delay
            setTimeout(() => fadeOutSnackbar(resetSnackbar), 1000);
          }
        })
        .catch(err =>
        {
          if (cachedElements.saveSnackbarText && cachedElements.saveSnackbar)
          {
            cachedElements.saveSnackbarText.textContent = "Try Again";
            cachedElements.saveSnackbar.style.background = getCssVar('--danger'); // Error red
            cachedElements.saveSnackbar.style.pointerEvents = 'auto';
          }
          console.error("Failed to save settings:", err);
        });
    }

    // Handle PID reset
    function handlePidReset()
    {
      const btn = this;
      const originalText = btn.textContent;
      btn.textContent = "Resetting...";
      btn.style.backgroundColor = getCssVar('--accent'); // Use accent color during reset
      
      // Call the reset endpoint
      fetch('/resetPID')
        .then(response => response.json())
        .then(data =>
        {
          if (data.success)
          {
            // Display a notification using the save snackbar
            if (cachedElements.saveSnackbar && cachedElements.saveSnackbarText)
            {
              cachedElements.saveSnackbar.style.display = 'block';
              cachedElements.saveSnackbar.style.background = getCssVar('--success');
              cachedElements.saveSnackbarText.textContent = 'PID Controller Reset';
              
              // Hide snackbar after 2 seconds
              setTimeout(() => fadeOutSnackbar(resetSnackbar), 2000);
            }
            
            showButtonFeedback(btn, 'Reset!', getCssVar('--success'), 1000, originalText, getCssVar('--accent'));
          }
          else
          {
            showButtonFeedback(btn, 'Error', getCssVar('--danger'), 1500, originalText, getCssVar('--accent'));
          }
        })
        .catch(error =>
        {
          console.error('Error resetting PID:', error);
          showButtonFeedback(btn, 'Error', getCssVar('--danger'), 1500, originalText, getCssVar('--accent'));
        });
    }

    // Handle Reset to Default: load values from settings.json and apply them
    function handleResetToDefaults()
    {
      if (!confirm('Are you sure you want to reset all settings to their default values?')) return;

      const btn = document.getElementById('resetDefaultsBtn');
      if (!btn) return;
      const originalText = btn.textContent;
      btn.textContent = 'Loading...';
      btn.disabled = true;

      fetch('/default.json')
        .then(r => r.json())
        .then(defaults =>
        {
          // Build /set query string using centralized PARAM_CONFIG
          let params = SLIDER_PARAMS
            .filter(param => defaults[PARAM_CONFIG[param].jsonKey] !== undefined)
            .map(param => param + '=' + encodeURIComponent(defaults[PARAM_CONFIG[param].jsonKey]))
            .join('&');

          // Include sensor calibration defaults
          if (defaults.sensor_limits)
          {
            const sl = defaults.sensor_limits;
            if (sl.minP !== undefined) params += '&sensor_minP=' + encodeURIComponent(sl.minP);
            if (sl.maxP !== undefined) params += '&sensor_maxP=' + encodeURIComponent(sl.maxP);
            if (sl.minV !== undefined) params += '&sensor_minV=' + encodeURIComponent(sl.minV);
            if (sl.maxV !== undefined) params += '&sensor_maxV=' + encodeURIComponent(sl.maxV);
          }

          return fetch('/set?' + params).then(() => defaults);
        })
        .then(defaults =>
        {
          // Also reset the PID controller
          // Update sensor inputs in the UI from defaults
          if (defaults.sensor_limits && cachedElements.sensorInputs)
          {
            const sl = defaults.sensor_limits;
            if (cachedElements.sensorInputs.sensor_minP && sl.minP !== undefined) cachedElements.sensorInputs.sensor_minP.value = sl.minP;
            if (cachedElements.sensorInputs.sensor_maxP && sl.maxP !== undefined) cachedElements.sensorInputs.sensor_maxP.value = sl.maxP;
            if (cachedElements.sensorInputs.sensor_minV && sl.minV !== undefined) cachedElements.sensorInputs.sensor_minV.value = sl.minV;
            if (cachedElements.sensorInputs.sensor_maxV && sl.maxV !== undefined) cachedElements.sensorInputs.sensor_maxV.value = sl.maxV;
          }
          return fetch('/resetPID');
        })
        .then(r => r.json())
        .then(data =>
        {
          // Clear any pending changes
          pendingChanges = {};
          if (cachedElements.saveSnackbar) cachedElements.saveSnackbar.style.display = 'none';

          showButtonFeedback(btn, 'Defaults Restored!', getCssVar('--success'), 1500, originalText, '', () => { btn.disabled = false; });
        })
        .catch(err =>
        {
          console.error('Error resetting to defaults:', err);
          showButtonFeedback(btn, 'Error', getCssVar('--danger'), 1500, originalText, '', () => { btn.disabled = false; });
        });
    }

    // Setup Easter egg
    function setupEasterEgg()
    {
      const logo = document.getElementById('ventrexLogo');
      const easterEgg = document.getElementById('easterEgg');
      const devInfo = document.getElementById('devInfo');
      
      if (logo && easterEgg && devInfo)
      {
        let clickCount = 0;
        let clickTimer;
        
        logo.addEventListener('click', function()
        {
          clickCount++;
          
          // Reset click count after 2 seconds of inactivity
          clearTimeout(clickTimer);
          clickTimer = setTimeout(() => { clickCount = 0; }, 2000);
          
          // Activate Easter egg after 5 clicks
          if (clickCount >= 5)
          {
            if (easterEgg.style.display === 'none')
            {
              // Show Easter egg and populate with system info
              easterEgg.style.display = 'block';
              devInfo.innerHTML = 
                `<div class="dev-info">
                   <div>Memory: ${performance?.memory?.usedJSHeapSize ? 
                     (performance.memory.usedJSHeapSize/1048576).toFixed(2) + ' MB' : 'N/A'}</div>
                   <div>Chart Points: ${window.chartData ? window.chartData.count : 0}</div>
                   <div>Time: ${new Date().toLocaleTimeString()}</div>
                   <div>Language: ${navigator.language}</div>
                   <div>Resolution: ${window.screen.width}x${window.screen.height}</div>
                   <div>Timezone: ${Intl.DateTimeFormat().resolvedOptions().timeZone}</div>
                 </div>
                 <div class="dev-info-footer">
                   Platform: ${navigator.platform || 'Unknown'}<br>
                   User Agent: ${navigator.userAgent.substring(0, 80)}...
                 </div>`;
              
              // Add a small animation to the logo
              logo.style.transition = 'transform 1s';
              logo.style.transform = 'rotate(360deg)';
              setTimeout(() => { logo.style.transform = 'rotate(0deg)'; }, 1000);
            }
            else
            {
              // Hide Easter egg
              easterEgg.style.display = 'none';
            }
            
            clickCount = 0;
          }
        });
      }
    }

    // ========== Slider Settings Modal Functions ==========
    let currentSliderParam = null;
    let sliderLimits = {};
    
    // Update chart primary Y-axis max to match setpoint slider max
    function updateChartYAxisMax(maxVal)
    {
      if (window.pressureChart && window.pressureChart.options.scales.y)
      {
        window.pressureChart.options.scales.y.max = maxVal;
        window.pressureChart.update('none');
      }
    }
    
    // Slider names for modal title
    const sliderNames = {
      sp: 'Setpoint',
      kp: 'Proportional (Kp)',
      ki: 'Integral (Ki)',
      kd: 'Derivative (Kd)'
    };
    
    // Fetch slider limits on page load
    function fetchSliderLimits()
    {
      fetch('/api/slider-limits')
        .then(r => r.json())
        .then(data =>
        {
          sliderLimits = data;
          applySliderLimits();
          // Sync chart Y-axis with setpoint max from saved settings
          if (data.sp && typeof data.sp.max === 'number') updateChartYAxisMax(data.sp.max);
        })
        .catch(err => console.error('Failed to fetch slider limits:', err));
    }
    
    // Apply loaded limits to sliders
    function applySliderLimits()
    {
      CONFIGURABLE_SLIDERS.forEach(param =>
      {
        const limits = sliderLimits[param];
        if (limits)
        {
          const slider = cachedElements.sliders[param];
          const text = cachedElements.texts[param];
          if (slider)
          {
            slider.min = limits.min;
            slider.max = limits.max;
            slider.step = limits.step;
          }
          if (text)
          {
            text.step = limits.step;
          }
        }
      });
    }
    
    // Open modal for a specific slider
    function openSliderSettings(param)
    {
      currentSliderParam = param;
      
      if (!cachedElements.modal || !cachedElements.modalTitle || 
          !cachedElements.modalMin || !cachedElements.modalMax || !cachedElements.modalStep) return;
      
      // Set title
      cachedElements.modalTitle.textContent = (sliderNames[param] || param) + ' Settings';
      
      // Load current values
      const limits = sliderLimits[param] || { min: 0, max: 100, step: 1 };
      cachedElements.modalMin.value = limits.min;
      cachedElements.modalMax.value = limits.max;
      cachedElements.modalStep.value = limits.step;
      
      cachedElements.modal.style.display = 'flex';
    }
    
    // Close modal
    function closeSliderModal()
    {
      if (cachedElements.modal) cachedElements.modal.style.display = 'none';
      currentSliderParam = null;
    }
    
    // Save slider settings
    function saveSliderSettings()
    {
      if (!currentSliderParam) return;
      
      const newMin = parseFloat(cachedElements.modalMin.value);
      const newMax = parseFloat(cachedElements.modalMax.value);
      const newStep = parseFloat(cachedElements.modalStep.value);
      
      // Validate
      if (isNaN(newMin) || isNaN(newMax) || isNaN(newStep) || newMax <= newMin || newStep <= 0)
      {
        alert('Invalid values. Max must be greater than Min, and Step must be positive.');
        return;
      }
      
      // Send to server
      const formData = new URLSearchParams();
      formData.append('param', currentSliderParam);
      formData.append('min', newMin);
      formData.append('max', newMax);
      formData.append('step', newStep);
      
      fetch('/api/slider-limits', {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
        body: formData.toString()
      })
      .then(r => r.json())
      .then(data =>
      {
        if (data.success)
        {
          // Update local cache
          sliderLimits[currentSliderParam] = { min: newMin, max: newMax, step: newStep };
          
          // Apply to slider
          const slider = cachedElements.sliders[currentSliderParam];
          const text = cachedElements.texts[currentSliderParam];
          if (slider)
          {
            slider.min = newMin;
            slider.max = newMax;
            slider.step = newStep;
            // Clamp current value if needed
            let val = parseFloat(slider.value);
            if (val < newMin) slider.value = newMin;
            if (val > newMax) slider.value = newMax;
            if (text) text.value = slider.value;
          }
          if (text) text.step = newStep;
          
          // If setpoint max changed, update chart Y-axis
          if (currentSliderParam === 'sp') updateChartYAxisMax(newMax);
          
          closeSliderModal();
        }
        else
        {
          alert('Failed to save settings');
        }
      })
      .catch(err =>
      {
        console.error('Error saving slider limits:', err);
        alert('Error saving settings');
      });
    }
    
    // Close modal on overlay click
    document.addEventListener('click', function(e)
    {
      if (e.target && e.target.id === 'sliderSettingsModal')
      {
        closeSliderModal();
      }
    });
    
    // Fetch limits on load
    document.addEventListener('DOMContentLoaded', fetchSliderLimits);

    // Setup scroll handler
    function setupScrollHandler()
    {
      const sliders = document.querySelectorAll('input[type="range"]');
      if (sliders && sliders.length > 0)
      {
        let scrollTimeout;
        
        // Add a class to disable sliders during scroll
        function disableSliders()
        {
          sliders.forEach(slider =>
          {
            if (slider && slider.classList)
            {
              slider.classList.add('scrolling');
            }
          });
        }
        
        // Remove the class with a small delay after scrolling stops
        function enableSliders()
        {
          clearTimeout(scrollTimeout);
          scrollTimeout = setTimeout(() =>
          {
            sliders.forEach(slider =>
            {
              if (slider && slider.classList)
              {
                slider.classList.remove('scrolling');
              }
            });
          }, 250); // Wait 250ms after scrolling stops before re-enabling
        }
        
        // Attach scroll event listener
        if (window.addEventListener)
        {
          window.addEventListener('scroll', () =>
          {
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
