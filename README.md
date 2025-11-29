# DC Motor Brush Ripple Detector

A C function for embedded systems that detects brush commutation ripple from DC motor current and voltage measurements.

## Motor Specifications

- **Brush commutations per revolution**: 18
- **Overlapping brushes**: Yes (produces small pulses during commutation)
- **Sampling rate**: 10 kHz (100μs sample period)

## Function Overview

```c
float brush_ripple_detector(float motor_current, float motor_voltage);
```

### Inputs
- `motor_current`: Motor current measurement in Amperes (float32)
- `motor_voltage`: Motor voltage measurement in Volts (float32)

### Output
- Returns normalized ripple signal (0.0 to 1.0)
- Peaks in the output indicate brush commutation events

## Motor Parameters (Defined in Function)

All parameters are defined as `static const float` inside the function body:

- **Inductance (L)**: 0.5 mH (0.0005 H)
- **Resistance (R)**: 0.5 Ω
- **Torque constant (Kt)**: 0.05 Nm/A
- **Rotor inertia (J)**: 0.00001 kg·m²
- **Commutations per revolution**: 18

## How It Works

The function detects brush commutation by:

1. **Baseline Tracking**: Uses slow exponential filter to track DC components
2. **AC Extraction**: Removes baseline to isolate ripple components
3. **Derivative Detection**: Calculates current rate of change (di/dt)
4. **Inductive Kick Detection**: Computes L·di/dt to detect commutation transients
5. **Multi-signal Fusion**: Combines current AC, voltage AC, inductive, and derivative signals
6. **Filtering**: Applies fast exponential filter for smooth output
7. **Normalization**: Outputs 0.0-1.0 range for easy processing

## Usage Example

```c
#include "brush_ripple_detector.h"

void motor_control_loop(void)
{
    // Called every 100us (10 kHz)
    float current = read_motor_current();  // Read ADC
    float voltage = read_motor_voltage();  // Read ADC
    
    float ripple = brush_ripple_detector(current, voltage);
    
    // Use ripple signal for:
    // - Commutation event detection
    // - Speed estimation (18 pulses per revolution)
    // - Motor health monitoring
    // - Position estimation
    
    if (ripple > 0.7f) {
        // Commutation event detected
        commutation_counter++;
    }
}
```

## Features

- **Zero external dependencies**: Only uses standard `math.h`
- **Stateful operation**: Maintains filter states internally with static variables
- **Real-time capable**: Optimized for embedded systems
- **Single precision**: All calculations use float32 for efficiency
- **Self-contained**: All parameters defined in function body

## Customization

To adjust the motor parameters, modify the static constants in the function:

```c
static const float INDUCTANCE = 0.0005f;        // Your motor's inductance
static const float RESISTANCE = 0.5f;           // Your motor's resistance
static const float TORQUE_CONSTANT = 0.05f;     // Your motor's Kt
static const float ROTOR_INERTIA = 0.00001f;    // Your motor's inertia
```

To tune the detection sensitivity, adjust the filter parameters:

```c
static const float ALPHA_FAST = 0.3f;    // Increase for faster response
static const float ALPHA_SLOW = 0.05f;   // Decrease for slower baseline tracking
static const float RIPPLE_GAIN = 5.0f;   // Increase for more sensitivity
```

## Notes

- Function must be called at consistent 10 kHz rate (100μs period)
- First call initializes the filter and returns 0.0
- One function instance per motor (uses static variables)
- For multiple motors, create separate function instances or pass state structure
