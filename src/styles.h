#pragma once

// Define CSS as a constant string in PROGMEM to save RAM
const char CSS_STYLES[] PROGMEM = R"rawliteral(
<style>
:root 
{
  --primary: #2563eb;
  --primary-light: #60a5fa;
  --primary-dark: #1d4ed8;
  --secondary: #475569;
  --accent: #f59e0b;
  --danger: #ef4444;
  --success: #10b981;
  --background: #f8fafc;
  --card: #ffffff;
  --text: #1e293b;
  --border: #e2e8f0;
  --shadow: rgba(0, 0, 0, 0.05);
}

* 
{
  box-sizing: border-box;
  margin: 0;
  padding: 0;
}

body 
{
  font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
  color: var(--text);
  background-color: var(--background);
  line-height: 1.4;
  max-width: 1000px;
  margin: 0 auto;
  padding: 0 0.5rem 0.5rem 0.5rem;
  min-height: 100vh;
  display: flex;
  flex-direction: column;
}

main
{
  flex: 1;
}

header
{
  margin: 0 0 1rem 0;
}

/* Collapsible card styling */
details.card
{
  -webkit-tap-highlight-color: transparent;
}

details.card summary
{
  display: flex;
  justify-content: space-between;
  align-items: center;
  cursor: pointer;
  font-weight: 600;
  font-size: 1rem;
  color: var(--primary-dark);
  list-style: none;
  padding: 0.5rem 0;
}

details.card summary::-webkit-details-marker
{
  display: none;
}

details.card summary::after
{
  content: '\25BC';
  font-size: 0.75rem;
  transition: transform 0.2s ease;
}

details.card[open] summary::after
{
  transform: rotate(180deg);
}

details.card .collapsible-content
{
  padding-top: 0.5rem;
}

/* Modal Overlay and Content */
.modal-overlay
{
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  background: rgba(0, 0, 0, 0.5);
  z-index: 10000;
  display: none;
  align-items: center;
  justify-content: center;
}

.modal-content
{
  background: white;
  border-radius: 12px;
  box-shadow: 0 8px 32px rgba(0, 0, 0, 0.2);
  min-width: 280px;
  max-width: 90vw;
  animation: modalFadeIn 0.2s ease;
}

@keyframes modalFadeIn
{
  from { opacity: 0; transform: scale(0.95); }
  to { opacity: 1; transform: scale(1); }
}

.modal-header
{
  display: flex;
  justify-content: space-between;
  align-items: center;
  padding: 1rem;
  border-bottom: 1px solid var(--border);
}

.modal-header h3
{
  margin: 0;
  font-size: 1rem;
  color: var(--primary-dark);
}

.modal-close
{
  background: none;
  border: none;
  font-size: 1.5rem;
  cursor: pointer;
  color: var(--text-light);
  padding: 0;
  line-height: 1;
}

.modal-close:hover
{
  color: var(--text);
}

.modal-body
{
  padding: 1rem;
}

.modal-row
{
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 0.75rem;
}

.modal-row:last-child
{
  margin-bottom: 0;
}

.modal-row label
{
  font-weight: 500;
  color: var(--text);
}

.modal-row input
{
  width: 120px;
  padding: 0.5rem;
  border: 1px solid var(--border);
  border-radius: 6px;
  font-size: 0.9rem;
}

.modal-footer
{
  display: flex;
  justify-content: flex-end;
  gap: 0.5rem;
  padding: 1rem;
  border-top: 1px solid var(--border);
}

.modal-btn
{
  padding: 0.5rem 1rem;
  border-radius: 6px;
  font-weight: 500;
  cursor: pointer;
  border: none;
  font-size: 0.9rem;
}

.modal-btn-cancel
{
  background: var(--background-alt);
  color: var(--text);
}

.modal-btn-save
{
  background: var(--primary);
  color: white;
}

.modal-btn-save:hover
{
  background: var(--primary-dark);
}

/* Gear icon for slider settings */
.settings-gear
{
  background: none;
  border: none;
  cursor: pointer;
  font-size: 0.9rem;
  padding: 0;
  margin: 0;
  width: auto;
  min-width: 0;
  line-height: 1;
  flex-shrink: 0;
  color: var(--text-light);
  opacity: 0.6;
  transition: opacity 0.2s;
}

.settings-gear:hover
{
  opacity: 1;
}

h1, h2, h3, h4
{
  font-weight: 600;
  line-height: 1.2;
  margin-bottom: 0.5rem;
  font-size: 0.5rem;
}

h1
{
  font-size: 1.5rem;
  text-align: center;
  color: var(--primary-dark);
}

h2
{
  font-size: 1.25rem;
  color: var(--primary);
  border-bottom: 1px solid var(--border);
  padding-bottom: 0.25rem;
  margin-bottom: 0.75rem;
}

h3
{
  font-size: 1rem;
  color: var(--secondary);
  margin-bottom: 0.25rem;
}

.alert
{
  background-color: #fef9c3;
  border-left: 4px solid var(--accent);
  color: #854d0e;
  padding: 0.5rem 0.75rem;
  border-radius: 0.375rem;
  margin-bottom: 0.75rem;
  display: flex;
  align-items: center;
  justify-content: space-between;
}

.alert h3
{
  margin-bottom: 0;
  color: #854d0e;
  font-size: 0.9rem;
}

.alert p
{
  font-size: 0.8rem;
  margin: 0;
}

.card
{
  background-color: var(--card);
  border-radius: 0.5rem;
  box-shadow: 0 2px 4px var(--shadow);
  padding: 0.75rem;
  margin-bottom: 0.75rem;
}

.section-title
{
  display: flex;
  align-items: center;
  margin: 0.5rem 0;
}

.section-title span
{
  font-size: 0.8rem;
  font-weight: 600;
  color: var(--secondary);
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.section-title div
{
  flex-grow: 1;
  height: 1px;
  background: var(--border);
  margin-left: 0.5rem;
}

.control-row
{
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-bottom: 0.5rem;
  flex-wrap: nowrap;
  max-width: 100%;
  box-sizing: border-box;
}

@media (max-width: 640px)
{
  .control-row
  {
    flex-direction: column;
    align-items: stretch;
  }
}

.control-row label
{
  min-width: 8rem;
  font-weight: 500;
  font-size: 0.875rem;
}

.time-const
{
  font-weight: 400;
  font-size: 0.75rem;
  color: var(--text-secondary, #6b7280);
  white-space: nowrap;
}

.control-slider
{
  flex: 1;
  min-width: 0;
  display: flex;
  align-items: center;
  gap: 0.35rem;
  overflow: hidden;
}

/* Sensor settings: two-column grid with input+unit groups */
.sensor-grid
{
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 0.75rem;
}

@media (max-width: 640px)
{
  .sensor-grid { grid-template-columns: 1fr; }
}

.sensor-group label
{
  display: block;
  font-weight: 500;
  font-size: 0.8rem;
  margin-bottom: 0.2rem;
  color: var(--text);
}

.input-with-unit
{
  display: flex;
  align-items: center;
  gap: 0;
}

.input-with-unit input
{
  width: 5.5rem;
  padding: 0.3rem 0.4rem;
  border: 1px solid var(--border);
  border-radius: 0.25rem 0 0 0.25rem;
  font-size: 0.85rem;
  text-align: right;
  outline: none;
  transition: border-color 0.2s;
}

.input-with-unit input:focus
{
  border-color: var(--primary);
}

.input-with-unit .unit
{
  padding: 0.3rem 0.45rem;
  background: var(--background);
  border: 1px solid var(--border);
  border-left: none;
  border-radius: 0 0.25rem 0.25rem 0;
  font-size: 0.8rem;
  color: var(--text-secondary, #6b7280);
  white-space: nowrap;
  user-select: none;
}

.sensor-section-title
{
  font-size: 0.75rem;
  font-weight: 600;
  text-transform: uppercase;
  letter-spacing: 0.05em;
  color: var(--text-secondary, #6b7280);
  margin-bottom: 0.35rem;
  margin-top: 0.25rem;
}

.sensor-section-title:not(:first-child)
{
  margin-top: 0.85rem;
}

.slider-btn 
{
  width: 20px;
  height: 20px;
  min-width: 20px;
  min-height: 20px;
  max-width: 20px;
  max-height: 20px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: var(--primary-light);
  color: var(--primary-dark);
  border: 1px solid var(--primary);
  border-radius: 4px;
  font-size: 1rem;
  font-weight: bold;
  cursor: pointer;
  transition: background 0.2s, color 0.2s;
  user-select: none;
  padding: 0;
}

.slider-btn:active
{
  background: var(--primary-dark);
  color: #fff;
}

.slider-btn:disabled
{
  opacity: 0.5;
  cursor: not-allowed;
}

input[type="range"]
{
  flex: 1;
  height: 0.375rem;
  appearance: none;
  background-color: var(--border);
  border-radius: 0.25rem;
  outline: none;
  pointer-events: none; /* Make the track non-interactive */
}

input[type="range"]::-webkit-slider-thumb
{
  appearance: none;
  width: 1rem;
  height: 1rem;
  border-radius: 50%;
  background-color: var(--primary);
  cursor: pointer;
  pointer-events: auto; /* Make the thumb interactive */
}

/* Add a new class for temporarily disabling sliders during scroll */
input[type="range"].scrolling::-webkit-slider-thumb
{
  pointer-events: none !important; /* Disable during scrolling */
}

input[type="range"].scrolling::-moz-range-thumb
{
  pointer-events: none !important; /* Disable during scrolling for Firefox */
}

input[type="range"].scrolling::-ms-thumb
{
  pointer-events: none !important; /* Disable during scrolling for IE/Edge */
}

input[type="range"]::-moz-range-thumb
{
  width: 1rem;
  height: 1rem;
  border-radius: 50%;
  background-color: var(--primary);
  cursor: pointer;
  pointer-events: auto; /* Make the thumb interactive for Firefox */
  border: none;
}

input[type="range"]::-ms-thumb
{
  width: 1rem;
  height: 1rem;
  border-radius: 50%;
  background-color: var(--primary);
  cursor: pointer;
  pointer-events: auto; /* Make the thumb interactive for IE/Edge */
  border: none;
}

input[type="text"][inputmode]
{
  width: 4rem;
  padding: 0.25rem;
  border: 1px solid var(--border);
  border-radius: 0.25rem;
  font-size: 0.8rem;
  text-align: right;
}

/* Validation error state for any input */
.input-error
{
  border-color: var(--danger) !important;
  box-shadow: 0 0 0 2px rgba(239,68,68,0.2);
}

/* Validation error tooltip shown below input */
.input-error-msg
{
  color: var(--danger);
  font-size: 0.7rem;
  margin-top: 2px;
  display: none;
}

.input-error-msg.visible
{
  display: block;
}

.button-container
{
  display: flex;
  justify-content: flex-start;
  margin-top: 0.75rem;
}

button
{
  background-color: var(--primary);
  color: white;
  border: none;
  border-radius: 0.375rem;
  padding: 0.5rem 1rem;
  font-weight: 600;
  cursor: pointer;
  min-width: 150px;
  font-size: 0.875rem;
}

.status-value
{
  font-weight: 600;
  padding: 0.25rem 0.5rem;
  background-color: var(--background);
  border-radius: 0.25rem;
  min-width: 3.5rem;
  text-align: right;
  font-size: 0.875rem;
}

progress
{
  -webkit-appearance: none;
  appearance: none;
  width: 100%;
  height: 0.375rem;
  border-radius: 0.25rem;
}

.chart-container
{
  height: 200px;
  width: 100%;
  margin-top: 0.0rem;
}

.chart-header
{
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin: 0.5rem 0 0;
}

.chart-toggle
{
  display: flex;
  align-items: center;
  gap: 0.375rem;
}

.toggle-label
{
  font-size: 0.75rem;
  color: var(--secondary);
}

input[type="checkbox"]
{
  height: 0.875rem;
  width: 0.875rem;
  cursor: pointer;
}

.hardware-info
{
  display: flex;
  align-items: center;
  gap: 0.375rem;
}

.status-indicator
{
  display: inline-block;
  width: 0.625rem;
  height: 0.625rem;
  border-radius: 50%;
}

.status-text
{
  font-size: 0.8rem;
}

footer
{
  text-align: center;
  margin-top: auto;
  font-size: 0.75rem;
  color: var(--secondary);
  padding-bottom: 0.5rem;
}

span
{
  font-size: 0.875rem;
}

.gauge-container
{
  display: flex;
  justify-content: space-between;
  margin-bottom: 1rem;
}

.gauge
{
  flex: 1;
  display: flex;
  flex-direction: column;
  align-items: center;
  background: linear-gradient(180deg, rgba(255,255,255,0.9) 0%, rgba(240,240,240,0.8) 100%);
  border-radius: 10px;
  padding: 0.6rem;
  margin: 0 0.3rem;
  position: relative;
  box-shadow: inset 0 1px 3px rgba(0,0,0,0.1);
}

.gauge-title
{
  font-size: 0.75rem;
  font-weight: 600;
  color: var(--secondary);
  margin-bottom: 0.3rem;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.gauge-value
{
  font-size: 1.8rem;
  font-weight: 700;
  color: var(--primary-dark);
  margin: 0.1rem 0;
  line-height: 1;
  display: flex;
  align-items: baseline;
}

.gauge-value small
{
  font-size: 0.75rem;
  color: var(--secondary);
  margin-left: 0.2rem;
}

.gauge-bar
{
  width: 100%;
  height: 0.5rem;
  background-color: #e2e8f0;
  border-radius: 0.25rem;
  margin-top: 0.3rem;
  overflow: hidden;
  position: relative;
}

.gauge-fill
{
  height: 100%;
  background: linear-gradient(90deg, var(--primary-light) 0%, var(--primary) 100%);
  border-radius: 0.25rem;
  transition: width 0.3s ease-out;
  width: 0%;
}

.gauge-target
{
  position: absolute;
  height: 100%;
  width: 2px;
  background-color: var(--accent);
  top: 0;
  transition: left 0.3s ease-out;
}

.trend-indicator
{
  margin-left: 0.3rem;
  font-size: 1rem;
  transition: transform 0.3s ease;
}

.trend-up 
{
  color: var(--danger);
  transform: translateY(-2px);
}

.trend-down 
{
  color: var(--success);
  transform: translateY(2px) rotate(180deg);
}

.trend-stable 
{
  color: var(--secondary);
  opacity: 0.5;
}

/* Logo styling */
.logo-container
{
  text-align: center;
  margin-bottom: 0.5rem;
}

.logo
{
  height: 60px;
  max-width: 100%;
}

.logo-fallback
{
  font-size: 2rem;
  font-weight: 700;
  color: var(--primary);
  letter-spacing: 0.1em;
  display: inline-block;
  height: 60px;
  line-height: 60px;
}

@keyframes spin
{
  0% { transform: rotate(0deg); }
  100% { transform: rotate(360deg); }
}

.increment-btn
{
  background-color: #10b981 !important;
  color: #fff !important;
  border: none;
}
.decrement-btn
{
  background-color: #002f87 !important;
  color: #fff !important;
  border: none;
}

.reset-btn
{
  background-color: #f59e0b;
  color: white;
  border: none;
  border-radius: 4px;
  padding: 6px 12px;
  font-size: 0.8rem;
  font-weight: 600;
  cursor: pointer;
  transition: background-color 0.2s;
  margin-left: auto;
  margin-right: 12px;
}

.reset-btn:hover
{
  background-color: #d97706;
}

/* Reset to Default button */
.reset-defaults-container
{
  text-align: center;
  margin: 1rem 0;
}

.reset-defaults-btn
{
  background-color: #64748b;
  color: white;
  border: none;
  border-radius: 6px;
  padding: 10px 24px;
  font-size: 0.85rem;
  font-weight: 600;
  cursor: pointer;
  transition: background-color 0.2s;
}

.reset-defaults-btn:hover
{
  background-color: #475569;
}

.reset-defaults-btn:disabled
{
  opacity: 0.6;
  cursor: not-allowed;
}

/* Loader overlay */
.loader
{
  position: fixed;
  top: 0;
  left: 0;
  width: 100vw;
  height: 100vh;
  z-index: 9999;
  background: #fff;
  display: flex;
  align-items: center;
  justify-content: center;
}

.loader-content
{
  display: flex;
  flex-direction: column;
  align-items: center;
}

.loader-logo
{
  height: 30px;
  margin-bottom: 16px;
}

.loader-title
{
  font-size: 1.5rem;
  font-weight: 1000;
  margin-bottom: 16px;
  text-align: center;
  background: linear-gradient(90deg, #002f87 0%, #32c09d 100%);
  -webkit-background-clip: text;
  background-clip: text;
  color: transparent;
}

.loader-spinner
{
  border: 6px solid #f3f3f3;
  border-top: 6px solid var(--primary);
  border-radius: 50%;
  width: 48px;
  height: 48px;
  animation: spin 1s linear infinite;
}

.loader-text
{
  margin-top: 12px;
  color: var(--primary);
  font-weight: 600;
}

/* Brand title gradient */
.brand-title
{
  background: linear-gradient(90deg, #002f87 0%, #32c09d 100%);
  -webkit-background-clip: text;
  background-clip: text;
  color: transparent;
  display: inline-block;
  font-size: 0.75em;
  font-weight: 1000;
}

/* Snackbar */
.snackbar
{
  position: fixed;
  bottom: 24px;
  bottom: calc(24px + env(safe-area-inset-bottom, 0px));
  left: 50%;
  transform: translateX(-50%);
  background: var(--primary);
  color: white;
  padding: 12px 24px;
  border-radius: 24px;
  box-shadow: 0 4px 12px rgba(0, 0, 0, 0.15);
  font-weight: 600;
  z-index: 1000;
  display: none;
  cursor: pointer;
  transition: all 0.3s ease;
}

/* Section title with button */
.section-title-row
{
  display: flex;
  align-items: center;
  justify-content: space-between;
  margin-top: 1rem;
}

/* Easter egg panel styles */
.easter-egg
{
  display: none;
  margin-top: 20px;
  padding: 15px;
  background: #f0f8ff;
  border-radius: 8px;
  text-align: left;
}

.easter-egg h3
{
  color: #002f87;
  text-align: center;
}

.easter-egg p
{
  text-align: center;
}

.easter-egg h4
{
  color: #002f87;
  margin-bottom: 10px;
}

.easter-egg-section
{
  margin-top: 20px;
}

.dev-info
{
  display: grid;
  grid-template-columns: 1fr 1fr;
  gap: 10px;
  font-family: monospace;
  font-size: 12px;
  margin-bottom: 15px;
}

.dev-info-footer
{
  margin-top: 8px;
  font-size: 10px;
  color: #666;
}

.cmd-ref
{
  font-family: monospace;
  font-size: 11px;
  background: #fff;
  padding: 12px;
  border-radius: 6px;
  max-height: 400px;
  overflow-y: auto;
  border: 1px solid #ddd;
}

.cmd-ref-header
{
  text-align: center;
  font-weight: bold;
  margin-bottom: 15px;
  padding: 8px;
  background: #f8fafc;
  border-radius: 4px;
}

.cmd-ref-title
{
  color: #2563eb;
  font-size: 12px;
}

.cmd-ref-subtitle
{
  color: #64748b;
  font-size: 10px;
  margin-top: 2px;
}

.cmd-group
{
  border: 1px solid #e2e8f0;
  border-radius: 4px;
  margin-bottom: 12px;
  overflow: hidden;
}

.cmd-group:last-child
{
  margin-bottom: 0;
}

.cmd-group-header
{
  background: #f1f5f9;
  padding: 6px 8px;
  font-weight: bold;
  color: #334155;
  border-bottom: 1px solid #e2e8f0;
}

.cmd-group-body
{
  padding: 8px;
}

.cmd-grid
{
  display: grid;
  grid-template-columns: 120px 1fr;
  gap: 8px;
  font-size: 10px;
}

.cmd-workflow
{
  border-top: 1px solid #e2e8f0;
  padding-top: 8px;
  font-size: 10px;
}

.cmd-workflow-title
{
  font-weight: bold;
  margin-bottom: 4px;
  color: #2563eb;
}

.cmd-workflow-steps
{
  color: #64748b;
  line-height: 1.4;
}
</style>
)rawliteral";

