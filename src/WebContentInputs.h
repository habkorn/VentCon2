#pragma once
#include <Arduino.h>

// ============================================================================
// Section 3: Dynamic input fields with placeholders (~1.5KB)
// This is the ONLY section that needs String processing
// ============================================================================

const char HTML_INPUTS[] PROGMEM = R"rawliteral(
    <details class="card" open>
      <summary>Control Parameters</summary>
      <div class="collapsible-content">
        <div class="control-row">
          <label for="sp_slider"><span style="color:var(--accent)">SETPOINT</span> Outlet Pressure in bar(g)</label>
          <div class="control-slider">
            <button type="button" class="slider-btn decrement-btn" id="sp_decrement">-</button>
            <input type="range" id="sp_slider" min="%SP_MIN%" max="%SP_MAX%" step="%SP_STEP%" value="%SP%">
            <button type="button" class="slider-btn increment-btn" id="sp_increment">+</button>
            <input type="text" inputmode="decimal" id="sp_text" value="%SP%" step="%SP_STEP%">
            <button class="settings-gear" onclick="openSliderSettings('sp')" title="Configure slider limits">&#9965;</button>
          </div>
        </div>    
      
        <hr>
      
        <div class="section-title-row">
          <span class="section-title"><span>PID</span><div></div></span>
          <button id="resetPidBtn" class="reset-btn" title="Re-initialize PID with current settings">Reset PID</button>
        </div>

        <div class="control-row">
          <label for="kp_slider">Proportional Kp</label>
          <div class="control-slider">
            <button type="button" class="slider-btn decrement-btn" id="kp_decrement">-</button>
            <input type="range" id="kp_slider" min="%KP_MIN%" max="%KP_MAX%" step="%KP_STEP%" value="%KP%">
            <button type="button" class="slider-btn increment-btn" id="kp_increment">+</button>
            <input type="text" inputmode="decimal" id="kp_text" value="%KP%" step="%KP_STEP%">
            <button class="settings-gear" onclick="openSliderSettings('kp')" title="Configure slider limits">&#9965;</button>
          </div>
        </div>

        <div class="control-row">
          <label for="ki_slider">Integral Ki <span id="ti_display" class="time-const" title="Integration time constant Ti = Kp / Ki"></span></label>
          <div class="control-slider">
            <button type="button" class="slider-btn decrement-btn" id="ki_decrement">-</button>
            <input type="range" id="ki_slider" min="%KI_MIN%" max="%KI_MAX%" step="%KI_STEP%" value="%KI%">
            <button type="button" class="slider-btn increment-btn" id="ki_increment">+</button>
            <input type="text" inputmode="decimal" id="ki_text" value="%KI%" step="%KI_STEP%">
            <button class="settings-gear" onclick="openSliderSettings('ki')" title="Configure slider limits">&#9965;</button>
          </div>
        </div>

        <div class="control-row">
          <label for="kd_slider">Derivative Kd <span id="td_display" class="time-const" title="Derivative time constant Td = Kd / Kp"></span></label>
          <div class="control-slider">
            <button type="button" class="slider-btn decrement-btn" id="kd_decrement">-</button>
            <input type="range" id="kd_slider" min="%KD_MIN%" max="%KD_MAX%" step="%KD_STEP%" value="%KD%">
            <button type="button" class="slider-btn increment-btn" id="kd_increment">+</button>
            <input type="text" inputmode="decimal" id="kd_text" value="%KD%" step="%KD_STEP%">
            <button class="settings-gear" onclick="openSliderSettings('kd')" title="Configure slider limits">&#9965;</button>
          </div>
        </div>
      </div>
    </details>

    <details class="card">
      <summary>Auxiliary Settings</summary>
      <div class="collapsible-content">
        <div class="control-row">
          <label for="flt_slider">Low Pass Filter Strength on Pressure Sensor (α)</label>
          <div class="control-slider">
            <button type="button" class="slider-btn decrement-btn" id="flt_decrement">-</button>
            <input type="range" id="flt_slider" min="%FLT_MIN%" max="%FLT_MAX%" step="%FLT_STEP%" value="%FLT%">
            <button type="button" class="slider-btn increment-btn" id="flt_increment">+</button>
            <input type="text" inputmode="decimal" id="flt_text" value="%FLT%" step="%FLT_STEP%">
          </div>
        </div>

        <div class="control-row">
          <label for="freq_slider">Actuator PWM Frequency in Hz</label>
          <div class="control-slider">
            <button type="button" class="slider-btn decrement-btn" id="freq_decrement">-</button>
            <input type="range" id="freq_slider" min="%FREQ_MIN%" max="%FREQ_MAX%" step="%FREQ_STEP%" value="%FREQ%">
            <button type="button" class="slider-btn increment-btn" id="freq_increment">+</button>
            <input type="text" inputmode="numeric" id="freq_text" value="%FREQ%" step="%FREQ_STEP%">
          </div>
        </div>

        <div class="control-row">
          <label for="res_slider">Actuator PWM Resolution in bits</label>
          <div class="control-slider">
            <button type="button" class="slider-btn decrement-btn" id="res_decrement">-</button>
            <input type="range" id="res_slider" min="%RES_MIN%" max="%RES_MAX%" step="%RES_STEP%" value="%RES%">
            <button type="button" class="slider-btn increment-btn" id="res_increment">+</button>
            <input type="text" inputmode="numeric" id="res_text" value="%RES%" step="%RES_STEP%">
          </div>
        </div>

        <div class="control-row">
          <label for="pst_text">PID Sample Time in ms</label>
          <div class="input-with-unit">
            <input type="text" inputmode="numeric" id="pst_text" value="%PST%" min="%PST_MIN%" max="%PST_MAX%">
            <span class="unit">ms</span>
          </div>
        </div>
      </div>
    </details>

    <details class="card">
      <summary>Outlet Pressure Sensor Settings</summary>
      <div class="collapsible-content">
        <div class="sensor-columns">
          <div class="sensor-block">
            <div class="sensor-section-title">Pressure Range</div>
            <div class="sensor-group">
              <label for="sensor_minP">Min</label>
              <div class="input-with-unit">
                <input type="text" inputmode="decimal" id="sensor_minP" value="%SENSOR_MINP%">
                <span class="unit">bar(g)</span>
              </div>
            </div>
            <div class="sensor-group">
              <label for="sensor_maxP">Max</label>
              <div class="input-with-unit">
                <input type="text" inputmode="decimal" id="sensor_maxP" value="%SENSOR_MAXP%">
                <span class="unit">bar(g)</span>
              </div>
            </div>
          </div>
          <div class="sensor-block">
            <div class="sensor-section-title">Voltage Range</div>
            <div class="sensor-group">
              <label for="sensor_minV">Min</label>
              <div class="input-with-unit">
                <input type="text" inputmode="decimal" id="sensor_minV" value="%SENSOR_MINV%">
                <span class="unit">V</span>
              </div>
            </div>
            <div class="sensor-group">
              <label for="sensor_maxV">Max</label>
              <div class="input-with-unit">
                <input type="text" inputmode="decimal" id="sensor_maxV" value="%SENSOR_MAXV%">
                <span class="unit">V</span>
              </div>
            </div>
          </div>
        </div>
      </div>
    </details>

    <details class="card" open>
      <summary>System Information</summary>
      <div class="collapsible-content">
        <div class="control-row">
          <label>Network Status</label>
          <div class="hardware-info">
            <span class="status-indicator" id="network_indicator"></span>
            <span id="network_status" class="status-text">--</span>
          </div>
        </div>
      </div>
    </details>



  </main>
)rawliteral";
