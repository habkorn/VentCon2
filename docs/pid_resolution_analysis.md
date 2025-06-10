# PWM Resolution Effects on PID Control Performance

## Resolution Impact on Control

The PWM resolution setting (`settings.pwm_res`) has several important effects on the PID control system:

### 1. Control Granularity

- **Higher Resolution (e.g., 15-bit = 32,768 steps)**
  - Allows for extremely fine control adjustments
  - Minimizes quantization errors in the control signal
  - Enables smoother transitions between valve positions
  - Particularly important for precise pressure regulation at steady state

- **Lower Resolution (e.g., 8-bit = 256 steps)**
  - Creates noticeable "steps" in the control output
  - May cause the valve to jump between discrete positions
  - Can introduce limit cycling (small oscillations around setpoint)

### 2. Response Characteristics

Looking at the code, when the resolution changes, the PID output limits are updated:

```cpp
// Update max value and PID limits
pwm_max_value = new_max_value;
pid.SetOutputLimits(0, pwm_max_value);
```

This means the PID controller's internal calculations are scaled to the full resolution range. This affects:

- **Error Quantization**: With lower resolution, each step represents a larger percentage change in output
- **Derivative Action**: Smaller step sizes (higher resolution) allow more precise derivative calculation
- **Integral Accumulation**: Higher resolution allows more gradual accumulation of the integral term

### 3. Real-World Performance Considerations

For the VentCon system specifically:

- **Valve Characteristics**: Since the valve operates in the 50%-90% range, having finer control within this range is beneficial
- **ESP32 Hardware**: The ESP32 supports high-resolution PWM, but extremely high resolutions might require lower PWM frequencies
- **Diminishing Returns**: Resolution beyond 12-bit (4,096 steps) may yield diminishing returns if:
  - The valve cannot physically respond to such tiny changes
  - The pressure sensor noise floor masks the effects of very small control adjustments

## Optimal Resolution Settings

Based on the system characteristics:

| Resolution | Steps | Comments |
|------------|-------|----------|
| 8-bit      | 256   | Too coarse for precise pressure control |
| 10-bit     | 1,024 | Minimum recommended for basic control |
| 12-bit     | 4,096 | Good balance for most applications |
| 15-bit     | 32,768| Current setting - excellent precision but may be overkill |
| 16-bit     | 65,536| Maximum supported - likely unnecessary |

**Recommendation**: The current 15-bit setting provides excellent precision. If you need to optimize PWM frequency or processing overhead, you could reduce to 12-bit without significantly impacting control performance in most cases.

## Interaction with Anti-Windup and Hysteresis

The PWM resolution interacts with other control features:

- **Anti-Windup**: Higher resolution means smaller steps between values, potentially making the anti-windup less necessary for minor adjustments
- **Hysteresis Compensation**: Higher resolution allows for more precise application of hysteresis compensation

## Conclusion

PWM resolution is an important tuning parameter that affects control precision. While higher resolution generally improves control performance, it should be balanced with other system requirements like PWM frequency and processing overhead.
