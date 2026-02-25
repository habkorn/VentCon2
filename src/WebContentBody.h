#pragma once
#include <Arduino.h>

// ============================================================================
// Section 2: Body start (static content before input fields)
// ============================================================================

// HTML <div> elements are generic block-level containers used to group content
// for styling (via class/id) or layout purposes. They have no semantic meaning
// themselves but provide structure for CSS rules and JavaScript targeting.

// Element order matters: earlier elements render first, later elements stack
// on top (unless z-index overrides), and JS can only access parsed elements.

const char HTML_BODY_START[] PROGMEM = R"rawliteral(

</head>

<body>
  <div class="loader" id="loader">
    <div class="loader-content">
      <div class="logo-container">
        <img src="/Logo.svg" alt="VENTREX" class="logo" id="loaderLogo" onerror="this.style.display='none';var s=document.createElement('span');s.className='logo-fallback';s.textContent='VENTREX';this.parentNode.appendChild(s);">
      </div>
      <h1>
        <span class="brand-title" style="color: var(--ventrex-blue);" aria-label="VENTCON Pressure Control System">
          VENTCON Pressure Control System
        </span>
      </h1>
      <div class="loader-spinner"></div>
      <span class="loader-text" ">Loading...</span>
    </div>
  </div>


  <main>
    <section class="card">
      <!-- <h2 style="text-align: center;">Real Time Monitoring</h2> -->
      
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
          <div class="gauge-title">Valve Duty Cycle</div>
          <div class="gauge-value">
            <span id="pwm" style="color: var(--success);">--</span>
            <small>%</small>
            <span id="pwm-trend" class="trend-indicator trend-stable">▲</span>
          </div>
          <div class="gauge-bar">
            <div id="pwm-fill" class="gauge-fill" style="width:0%"></div>
          </div>
        </div>
      </div>
      
      <div class="chart-header">
        <div class="chart-header-left">
          <span class="chart-gear-btn" onclick="openChartSettings()" title="Chart Settings"></span>
          <h3>Live Chart</h3>
        </div>
        <div class="chart-header-right">
          <label for="chartToggle" class="toggle-label">Show Chart</label>
          <input type="checkbox" id="chartToggle" checked>
        </div>
      </div>
      
      <div class="chart-container" id="chartContainer">
        <canvas id="pressureChart"></canvas>
      </div>
    </section>   
   
    <!-- Snackbar Apply Changes button (hidden by default) -->
    <div id="saveSnackbar" class="snackbar">
      <span id="saveSnackbarText">Apply Changes</span>
    </div>
    
    <!-- Slider Settings Modal -->
    <div id="sliderSettingsModal" class="modal-overlay">
      <div class="modal-content">
        <div class="modal-header">
          <h3 id="modalTitle">Slider Settings</h3>
          <button class="modal-close" onclick="closeSliderModal()">&times;</button>
        </div>
        <div class="modal-body">
          <div class="modal-row">
            <label>Minimum value for slider</label>
            <input type="text" inputmode="decimal" id="modalMin" step="any">
          </div>
          <div class="modal-row">
            <label>Maximum value for slider</label>
            <input type="text" inputmode="decimal" id="modalMax" step="any">
          </div>
          <div class="modal-row">
            <label>Step for +/- Buttons</label>
            <input type="text" inputmode="decimal" id="modalStep" step="any">
          </div>
        </div>
        <div class="modal-footer">
          <button class="modal-btn modal-btn-cancel" onclick="closeSliderModal()">Cancel</button>
          <button class="modal-btn modal-btn-save" onclick="saveSliderSettings()">Save</button>
        </div>
      </div>
    </div>

    <!-- Chart Settings Modal -->
    <div id="chartSettingsModal" class="modal-overlay">
      <div class="modal-content">
        <div class="modal-header">
          <h3>Chart Settings</h3>
          <button class="modal-close" onclick="closeChartSettings()">&times;</button>
        </div>
        <div class="modal-body">
          <div class="modal-row">
            <label>Pressure axis min (bar)</label>
            <input type="text" inputmode="decimal" id="chartYMin" step="any">
          </div>
          <div class="modal-row">
            <label>Pressure axis max (bar)</label>
            <input type="text" inputmode="decimal" id="chartYMax" step="any">
          </div>
          <div class="modal-row">
            <label>Duty cycle axis min (%)</label>
            <input type="text" inputmode="decimal" id="chartPwmMin" step="any">
          </div>
          <div class="modal-row">
            <label>Duty cycle axis max (%)</label>
            <input type="text" inputmode="decimal" id="chartPwmMax" step="any">
          </div>
          <div class="modal-row">
            <label>Time window (seconds)</label>
            <input type="text" inputmode="numeric" id="chartTimeWindow" step="1">
          </div>
          <div class="modal-row">
            <label>Time grid interval (seconds)</label>
            <input type="text" inputmode="numeric" id="chartTimeGrid" step="1">
          </div>
        </div>
        <div class="modal-footer">
          <button class="modal-btn modal-btn-cancel" onclick="closeChartSettings()">Cancel</button>
          <button class="modal-btn modal-btn-save" onclick="saveChartSettings()">Save</button>
        </div>
      </div>
    </div>

)rawliteral";
