#pragma once
#include <Arduino.h>

// ============================================================================
// Section 4: Footer with version placeholder
// ============================================================================

const char HTML_FOOTER[] PROGMEM = R"rawliteral(
  <footer>

    <div class="logo-container">
      <img src="/Logo.svg" alt="VENTREX" class="logo" id="ventrexLogo" onerror="this.style.display='none';var s=document.createElement('span');s.className='logo-fallback';s.textContent='VENTREX';this.parentNode.appendChild(s);">
    </div>
    <h1>
      <span class="brand-title" aria-label="VENTCON Pressure Control System">
        VENTCON Pressure Control System
      </span>
    </h1>

    <p>v%VERSION% by HAB</p>
    <!-- Hidden Easter egg panel -->
    <div id="easterEgg" class="easter-egg">
      <h3>Developer Mode Activated! 🚀</h3>
      <p>Hello there, curious one!</p> 
      <p>You've found the secret developer panel.</p>
      <p>VENTCON Control System - Made with ❤️ by VENTREX</p>
      
      <div class="easter-egg-section">
        <h4>📊 System Information</h4>
        <div id="devInfo" class="dev-info"></div>
        <h4>💻 Serial Commands Reference</h4>
        <div class="cmd-ref">
          <div class="cmd-ref-header">
            <div class="cmd-ref-title">📋 VENTCON2 COMMAND REFERENCE</div>
            <div class="cmd-ref-subtitle">All commands are case-insensitive</div>
          </div>
          
          <div class="cmd-group">
            <div class="cmd-group-header">🎛️ PID CONTROL</div>
            <div class="cmd-group-body">
              <div class="cmd-grid">
                <span><strong>KP &lt;value&gt;</strong></span><span>Set proportional gain (e.g., KP 0.5)</span>
                <span><strong>KI &lt;value&gt;</strong></span><span>Set integral gain (e.g., KI 0.1)</span>
                <span><strong>KD &lt;value&gt;</strong></span><span>Set derivative gain (e.g., KD 0.01)</span>
                <span><strong>SP &lt;value&gt;</strong></span><span>Set pressure setpoint in bar (e.g., SP 3.0)</span>
                <span><strong>SAMPLE &lt;ms&gt;</strong></span><span>Set PID sample time, 1-1000ms (e.g., SAMPLE 10)</span>
                <span><strong>RESET</strong></span><span>Reset PID controller (clear windup & state)</span>
              </div>
            </div>
          </div>
          
          <div class="cmd-group">
            <div class="cmd-group-header">📊 SIGNAL PROCESSING</div>
            <div class="cmd-group-body">
              <div class="cmd-grid">
                <span><strong>FLT &lt;value&gt;</strong></span><span>Set filter strength, 0.0-1.0 (e.g., FLT 0.2)</span>
                <span><strong>AW ON/OFF</strong></span><span>Enable/disable anti-windup for deadband</span>
                <span><strong>HYST ON/OFF</strong></span><span>Enable/disable hysteresis compensation</span>
                <span><strong>HYSTAMT &lt;val&gt;</strong></span><span>Set hysteresis amount in % (e.g., HYSTAMT 5)</span>
              </div>
            </div>
          </div>
          
          <div class="cmd-group">
            <div class="cmd-group-header">⚡ PWM & VALVE CONTROL</div>
            <div class="cmd-group-body">
              <div class="cmd-grid">
                <span><strong>FREQ &lt;hz&gt;</strong></span><span>Set PWM frequency, 100-10000Hz (e.g., FREQ 1000)</span>
                <span><strong>RES &lt;bits&gt;</strong></span><span>Set PWM resolution, 1-16 bits (e.g., RES 8)</span>
                <span><strong>PWM &lt;percent&gt;</strong></span><span>Force PWM duty cycle, 0-100% (e.g., PWM 25)</span>
                <span><strong>RESUME</strong></span><span>Resume normal PID control after manual PWM</span>
                <span><strong>CONTROL FREQ</strong></span><span>Set control loop frequency, 10-1000Hz</span>
              </div>
            </div>
          </div>
          
          <div class="cmd-group">
            <div class="cmd-group-header">🎯 AUTO-TUNING</div>
            <div class="cmd-group-body">
              <div class="cmd-grid">
                <span><strong>AUTOTUNE</strong></span><span>Test AutoTuner and show detailed configuration</span>
                <span><strong>TUNE START</strong></span><span>Start PID auto-tuning process</span>
                <span><strong>TUNE STOP</strong></span><span>Cancel auto-tuning process</span>
                <span><strong>TUNE ACCEPT</strong></span><span>Accept calculated PID parameters</span>
                <span><strong>TUNE REJECT</strong></span><span>Reject and keep current PID parameters</span>
                <span><strong>TUNE SP &lt;bar&gt;</strong></span><span>Set auto-tune test setpoint, 0.5-10.0 bar</span>
                <span><strong>TUNE MIN &lt;pct&gt;</strong></span><span>Set auto-tune minimum PWM, 50-90%</span>
                <span><strong>TUNE MAX &lt;pct&gt;</strong></span><span>Set auto-tune maximum PWM, 60-95%</span>
                <span><strong>TUNE CYCLE &lt;ms&gt;</strong></span><span>Set min cycle time, 50-2000ms</span>
                <span><strong>TUNE RULE &lt;0-3&gt;</strong></span><span>Select tuning rule (see TUNE RULES)</span>
                <span><strong>TUNE AGGR &lt;val&gt;</strong></span><span>Set aggressiveness factor, 0.5-2.0</span>
                <span><strong>TUNE RULES</strong></span><span>Show available tuning rules with descriptions</span>
              </div>
            </div>
          </div>
          
          <div class="cmd-group">
            <div class="cmd-group-header">📡 NETWORK & WIFI</div>
            <div class="cmd-group-body">
              <div class="cmd-grid">
                <span><strong>SCAN WIFI</strong></span><span>Scan and list WiFi networks with signal strength</span>
                <span><strong>WIFI CHANNEL</strong></span><span>Set WiFi AP channel, 1-13 (interference avoidance)</span>
                <span><strong>PAGE ON/OFF</strong></span><span>Enable/disable web server processing</span>
              </div>
            </div>
          </div>
          
          <div class="cmd-group">
            <div class="cmd-group-header">📈 DATA & MONITORING</div>
            <div class="cmd-group-body">
              <div class="cmd-grid">
                <span><strong>STATUS</strong></span><span>Show comprehensive system status</span>
                <span><strong>SENSOR</strong></span><span>Test SensorManager and show detailed sensor readings</span>
                <span><strong>STARTCD</strong></span><span>Start continuous data output for plotting</span>
                <span><strong>STOPCD</strong></span><span>Stop continuous data output</span>
                <span><strong>MEM</strong></span><span>Show memory usage and system information</span>
                <span><strong>VER</strong></span><span>Display firmware version and build info</span>
              </div>
            </div>
          </div>
          
          <div class="cmd-group">
            <div class="cmd-group-header">💾 FILE SYSTEM & SETTINGS</div>
            <div class="cmd-group-body">
              <div class="cmd-grid">
                <span><strong>SAVE</strong></span><span>Force save current settings to flash memory</span>
                <span><strong>READ</strong></span><span>Read and display settings stored in flash</span>
                <span><strong>DIR</strong></span><span>List all files in flash memory with sizes</span>
              </div>
            </div>
          </div>
          
          <div class="cmd-group">
            <div class="cmd-group-header">🚀 QUICK REFERENCE</div>
            <div class="cmd-group-body">
              <div class="cmd-grid" style="margin-bottom: 8px;">
                <span><strong>HELP</strong></span><span>Show this command reference</span>
              </div>
              <div class="cmd-workflow">
                <div class="cmd-workflow-title">Example workflow:</div>
                <div class="cmd-workflow-steps">
                  STATUS → Check current system state<br>
                  TUNE START → Begin auto-tuning for optimal PID<br>
                  TUNE ACCEPT → Accept calculated parameters<br>
                  STARTCD → Monitor real-time data<br>
                  SAVE → Persist settings to flash
                </div>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </footer>


  <div class="reset-defaults-container">
    <button id="resetDefaultsBtn" class="reset-defaults-btn" title="Load default values from settings.json and reset PID">Reset to Default</button>
  </div>

)rawliteral";
