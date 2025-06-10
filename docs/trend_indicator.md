# Pressure Trend Indicator

## Overview

The pressure trend indicator is a visual element that shows whether the pressure is increasing, decreasing, or stable. It appears as a triangular arrow (▲) next to the pressure value and changes color based on the detected trend.

## Implementation Details

### HTML Structure
The indicator is implemented as a span element with specific CSS classes:

```html
<span id="pressure-trend" class="trend-indicator trend-stable">▲</span>
```

### JavaScript Logic

The trend detection logic:

1. **Data Collection**:
   - The system maintains an array of the last 3 pressure readings
   - Each new reading is added to the array, and oldest is removed when full

2. **Trend Calculation**:
   - The system compares the most recent value with the oldest value in the array
   - A threshold of 0.05 bar is applied to filter out noise and minor fluctuations

3. **State Determination**:
   - If difference > threshold: UP trend (arrow becomes green)
   - If difference < -threshold: DOWN trend (arrow becomes red)
   - If difference within threshold: STABLE trend (arrow becomes gray)

```javascript
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
```

### CSS Styling

The trend indicator has different colors based on the detected trend:

- Green (trend-up): Indicates increasing pressure
- Red (trend-down): Indicates decreasing pressure
- Gray (trend-stable): Indicates stable pressure

### Update Cycle

The trend indicator is updated with each new pressure reading from the server (every 300ms) by calling the `updatePressureTrend()` function within the data fetch cycle.

## Benefits

1. **Visual Feedback**: Provides immediate visual cue about pressure direction
2. **Noise Filtering**: Ignores minor fluctuations that don't represent true trends
3. **Temporal Awareness**: Uses multiple readings over time rather than just adjacent points
4. **Enhanced Monitoring**: Makes it easier to detect if the system is approaching stability

This implementation helps operators quickly assess system behavior without having to watch pressure values closely or monitor the chart.
