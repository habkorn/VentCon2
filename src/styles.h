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
  padding: 0.5rem;
}

header {
  margin-bottom: 1rem;
}

h1, h2, h3, h4 {
  font-weight: 600;
  line-height: 1.2;
  margin-bottom: 0.5rem;
  font-size: 0.5rem;
}

h1 {
  font-size: 1.5rem;
  text-align: center;
  color: var(--primary-dark);
}

h2 {
  font-size: 1.25rem;
  color: var(--primary);
  border-bottom: 1px solid var(--border);
  padding-bottom: 0.25rem;
  margin-bottom: 0.75rem;
}

h3 {
  font-size: 1rem;
  color: var(--secondary);
  margin-bottom: 0.25rem;
}

.alert {
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

.alert h3 {
  margin-bottom: 0;
  color: #854d0e;
  font-size: 0.9rem;
}

.alert p {
  font-size: 0.8rem;
  margin: 0;
}

.card {
  background-color: var(--card);
  border-radius: 0.5rem;
  box-shadow: 0 2px 4px var(--shadow);
  padding: 0.75rem;
  margin-bottom: 0.75rem;
}

.section-title {
  display: flex;
  align-items: center;
  margin: 0.5rem 0;
}

.section-title span {
  font-size: 0.8rem;
  font-weight: 600;
  color: var(--secondary);
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.section-title div {
  flex-grow: 1;
  height: 1px;
  background: var(--border);
  margin-left: 0.5rem;
}

.control-row {
  display: flex;
  align-items: center;
  gap: 0.5rem;
  margin-bottom: 0.5rem;
  flex-wrap: wrap;
}

@media (max-width: 640px) {
  .control-row {
    flex-direction: column;
    align-items: stretch;
  }
}

.control-row label {
  min-width: 8rem;
  font-weight: 500;
  font-size: 0.875rem;
}

.control-slider {
  flex: 1;
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

input[type="range"] {
  flex: 1;
  height: 0.375rem;
  appearance: none;
  background-color: var(--border);
  border-radius: 0.25rem;
  outline: none;
}

input[type="range"]::-webkit-slider-thumb {
  appearance: none;
  width: 1rem;
  height: 1rem;
  border-radius: 50%;
  background-color: var(--primary);
  cursor: pointer;
}

input[type="number"] {
  width: 4rem;
  padding: 0.25rem;
  border: 1px solid var(--border);
  border-radius: 0.25rem;
  font-size: 0.8rem;
  text-align: right;
}

.button-container {
  display: flex;
  justify-content: flex-start;
  margin-top: 0.75rem;
}

button {
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

.status-value {
  font-weight: 600;
  padding: 0.25rem 0.5rem;
  background-color: var(--background);
  border-radius: 0.25rem;
  min-width: 3.5rem;
  text-align: right;
  font-size: 0.875rem;
}

progress {
  -webkit-appearance: none;
  appearance: none;
  width: 100%;
  height: 0.375rem;
  border-radius: 0.25rem;
}

.chart-container {
  height: 180px;
  width: 100%;
  margin-top: 0.5rem;
}

.chart-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin: 0.5rem 0 0;
}

.chart-toggle {
  display: flex;
  align-items: center;
  gap: 0.375rem;
}

.toggle-label {
  font-size: 0.75rem;
  color: var(--secondary);
}

input[type="checkbox"] {
  height: 0.875rem;
  width: 0.875rem;
  cursor: pointer;
}

.hardware-info {
  display: flex;
  align-items: center;
  gap: 0.375rem;
}

.status-indicator {
  display: inline-block;
  width: 0.625rem;
  height: 0.625rem;
  border-radius: 50%;
}

.status-text {
  font-size: 0.8rem;
}

footer {
  text-align: center;
  margin-top: 0.75rem;
  font-size: 0.75rem;
  color: var(--secondary);
  padding-bottom: 0.5rem;
}

span {
  font-size: 0.875rem;
}

.gauge-container {
  display: flex;
  justify-content: space-between;
  margin-bottom: 1rem;
}

.gauge {
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

.gauge-title {
  font-size: 0.75rem;
  font-weight: 600;
  color: var(--secondary);
  margin-bottom: 0.3rem;
  text-transform: uppercase;
  letter-spacing: 0.05em;
}

.gauge-value {
  font-size: 1.8rem;
  font-weight: 700;
  color: var(--primary-dark);
  margin: 0.1rem 0;
  line-height: 1;
  display: flex;
  align-items: baseline;
}

.gauge-value small {
  font-size: 0.75rem;
  color: var(--secondary);
  margin-left: 0.2rem;
}

.gauge-bar {
  width: 100%;
  height: 0.5rem;
  background-color: #e2e8f0;
  border-radius: 0.25rem;
  margin-top: 0.3rem;
  overflow: hidden;
  position: relative;
}

.gauge-fill {
  height: 100%;
  background: linear-gradient(90deg, var(--primary-light) 0%, var(--primary) 100%);
  border-radius: 0.25rem;
  transition: width 0.3s ease-out;
  width: 0%;
}

.gauge-target {
  position: absolute;
  height: 100%;
  width: 2px;
  background-color: var(--accent);
  top: 0;
  transition: left 0.3s ease-out;
}

.trend-indicator {
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
.logo-container {
  text-align: center;
  margin-bottom: 0.5rem;
}

.logo {
  height: 60px;
  max-width: 100%;
}
</style>
)rawliteral";
