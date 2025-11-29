/**
 * @file brush_ripple_detector.c
 * @brief DC Motor Brush Commutation Ripple Detection
 * 
 * Detects brush commutation events from current and voltage measurements
 * for a DC motor with 18 brush commutations per revolution.
 * Sampling rate: 10 kHz (100us sample period)
 */

#include <stdint.h>
#include <math.h>

/**
 * @brief Detects brush commutation ripple signal from motor measurements
 * 
 * @param motor_current Current measurement in Amperes
 * @param motor_voltage Voltage measurement in Volts
 * @return float32_t Normalized ripple signal (0.0 to 1.0, with peaks at commutation)
 */
float brush_ripple_detector(float motor_current, float motor_voltage)
{
    // Motor parameters (static, defined once)
    static const float INDUCTANCE = 0.0005f;        // Motor inductance [H] (0.5 mH)
    static const float RESISTANCE_NOMINAL = 0.5f;   // Nominal motor resistance [Ohm]
    static const float TORQUE_CONSTANT = 0.05f;     // Torque constant [Nm/A] (also back-EMF constant)
    static const float ROTOR_INERTIA = 0.00001f;    // Rotor inertia [kg*m^2]
    static const float COMMUTATIONS_PER_REV = 18.0f; // Brush commutations per revolution
    
    // Resistance estimation parameters
    static const float RESISTANCE_MIN = 0.1f;       // Minimum plausible resistance [Ohm]
    static const float RESISTANCE_MAX = 5.0f;       // Maximum plausible resistance [Ohm]
    static const float RESISTANCE_ALPHA = 0.01f;    // Slow adaptation rate for resistance
    static const float MIN_CURRENT_FOR_ESTIMATION = 0.1f; // Minimum current for valid estimation [A]
    
    // Sampling parameters
    static const float SAMPLE_TIME = 0.0001f;       // 100us sampling period [s]
    static const float TWO_PI = 6.28318530718f;     // 2*PI for angular calculations
    static const float ANGULAR_STEP = TWO_PI / COMMUTATIONS_PER_REV; // Radians per commutation
    
    // Filter parameters for ripple extraction
    static const float ALPHA_FAST = 0.3f;           // Fast filter coefficient for ripple
    static const float ALPHA_SLOW = 0.05f;          // Slow filter coefficient for baseline
    static const float RIPPLE_GAIN = 5.0f;          // Amplification of ripple signal
    
    // Validation parameters
    static const float MIN_PULSE_PERIOD = 0.0001f;  // Minimum expected pulse period [s] (max ~9400 RPM)
    static const float MAX_PULSE_PERIOD = 1.0f;     // Maximum expected pulse period [s] (min ~1 RPM)
    static const float ACCEL_TOLERANCE = 0.5f;      // Acceleration validation tolerance factor (±50%)
    static const float PERIOD_ALPHA = 0.1f;         // Filter for expected period smoothing
    static const float SYNTHETIC_PULSE_AMPLITUDE = 0.75f; // Amplitude for synthetic pulses
    static const float SYNTHETIC_PULSE_WIDTH = 0.002f;    // Width of synthetic pulse [s] (2ms)
    
    // Static variables to maintain state between calls
    static float current_prev = 0.0f;
    static float voltage_prev = 0.0f;
    static float current_baseline = 0.0f;
    static float voltage_baseline = 0.0f;
    static float current_filtered = 0.0f;
    static float voltage_filtered = 0.0f;
    static float ripple_output = 0.0f;
    static uint8_t initialized = 0;
    
    // Pulse validation state variables
    static float last_pulse_time = 0.0f;            // Time since last validated pulse [s]
    static float current_rpm = 0.0f;                // Current estimated RPM
    static float expected_pulse_period = 0.1f;      // Expected time between pulses [s]
    static float angular_velocity = 0.0f;           // Current angular velocity [rad/s]
    static float angular_velocity_measured = 0.0f;  // Angular velocity from pulse measurements [rad/s]
    static uint32_t sample_counter = 0;             // Sample counter for time tracking
    static uint8_t synthetic_pulse_active = 0;      // Flag indicating synthetic pulse generation
    static float synthetic_pulse_timer = 0.0f;      // Timer for synthetic pulse duration
    
    // Adaptive resistance estimation
    static float resistance_estimate = 0.5f;        // Estimated motor resistance [Ohm]
    static uint32_t valid_pulse_count = 0;          // Counter for valid pulses received
    static float last_valid_pulse_period = 0.1f;    // Last measured valid pulse period [s]
    
    // Initialize on first call
    if (!initialized) {
        current_baseline = motor_current;
        voltage_baseline = motor_voltage;
        current_prev = motor_current;
        voltage_prev = motor_voltage;
        sample_counter = 0;
        last_pulse_time = 0.0f;
        angular_velocity = 0.0f;
        angular_velocity_measured = 0.0f;
        current_rpm = 0.0f;
        synthetic_pulse_active = 0;
        synthetic_pulse_timer = 0.0f;
        resistance_estimate = RESISTANCE_NOMINAL;
        valid_pulse_count = 0;
        last_valid_pulse_period = 0.1f;
        initialized = 1;
        return 0.0f;
    }
    
    // Increment sample counter for time tracking
    sample_counter++;
    
    // Update synthetic pulse timer if active
    if (synthetic_pulse_active) {
        synthetic_pulse_timer += SAMPLE_TIME;
        if (synthetic_pulse_timer >= SYNTHETIC_PULSE_WIDTH) {
            synthetic_pulse_active = 0;
            synthetic_pulse_timer = 0.0f;
        }
    }
    
    // Calculate rate of change (derivative approximation)
    float current_derivative = (motor_current - current_prev) / SAMPLE_TIME;
    float voltage_derivative = (motor_voltage - voltage_prev) / SAMPLE_TIME;
    
    // Update baseline (slow moving average to track DC component)
    current_baseline = ALPHA_SLOW * motor_current + (1.0f - ALPHA_SLOW) * current_baseline;
    voltage_baseline = ALPHA_SLOW * motor_voltage + (1.0f - ALPHA_SLOW) * voltage_baseline;
    
    // Extract AC component (ripple) by removing baseline
    float current_ac = motor_current - current_baseline;
    float voltage_ac = motor_voltage - voltage_baseline;
    
    // Calculate expected voltage drop due to inductance during commutation
    // V_ripple = L * di/dt (inductive kick during commutation)
    float inductive_ripple = INDUCTANCE * fabsf(current_derivative);
    
    // Calculate resistive component ripple using adaptive resistance estimate
    float resistive_ripple = fabsf(current_ac) * resistance_estimate;
    
    // Combine current and voltage ripple indicators
    // Weight both the AC components and the derivative-based detection
    float combined_ripple = fabsf(current_ac) * 0.4f + 
                           fabsf(voltage_ac) * 0.3f + 
                           inductive_ripple * 0.2f +
                           fabsf(current_derivative) * 0.1f;
    
    // Apply fast exponential filter to smooth the ripple signal
    ripple_output = ALPHA_FAST * combined_ripple * RIPPLE_GAIN + 
                    (1.0f - ALPHA_FAST) * ripple_output;
    
    // Normalize output to 0.0 - 1.0 range
    // Clip to prevent overflow
    if (ripple_output > 1.0f) {
        ripple_output = 1.0f;
    } else if (ripple_output < 0.0f) {
        ripple_output = 0.0f;
    }
    
    // ===== PULSE VALIDATION USING MOTOR DYNAMICS =====
    
    // Use measured angular velocity if available, otherwise use model-based estimate
    if (valid_pulse_count > 2 && angular_velocity_measured > 0.1f) {
        // Blend measured velocity with model for smooth tracking
        angular_velocity = 0.7f * angular_velocity_measured + 0.3f * angular_velocity;
    }
    
    // Calculate motor torque from current: T = Kt * I
    float motor_torque = TORQUE_CONSTANT * motor_current;
    
    // Calculate angular acceleration from torque and inertia: alpha = T / J
    float angular_acceleration = motor_torque / ROTOR_INERTIA;
    
    // Update angular velocity based on acceleration
    angular_velocity += angular_acceleration * SAMPLE_TIME;
    
    // Prevent negative velocity (assuming unidirectional motor)
    if (angular_velocity < 0.0f) {
        angular_velocity = 0.0f;
    }
    
    // Calculate current RPM from angular velocity
    // RPM = (angular_velocity * 60) / (2 * PI)
    current_rpm = (angular_velocity * 60.0f) / TWO_PI;
    
    // Calculate expected pulse period from angular velocity
    // Period = angular_step / angular_velocity
    float calculated_period = (angular_velocity > 0.1f) ? 
                              (ANGULAR_STEP / angular_velocity) : MAX_PULSE_PERIOD;
    
    // Smooth the expected period estimate
    expected_pulse_period = PERIOD_ALPHA * calculated_period + 
                           (1.0f - PERIOD_ALPHA) * expected_pulse_period;
    
    // Clamp expected period to reasonable range
    if (expected_pulse_period < MIN_PULSE_PERIOD) {
        expected_pulse_period = MIN_PULSE_PERIOD;
    } else if (expected_pulse_period > MAX_PULSE_PERIOD) {
        expected_pulse_period = MAX_PULSE_PERIOD;
    }
    
    // Track time since last pulse
    last_pulse_time += SAMPLE_TIME;
    
    // ===== CHECK FOR MISSED PULSE (FALSE NEGATIVE PREVENTION) =====
    
    // If expected pulse period has passed and no pulse detected, generate synthetic pulse
    float missed_pulse_threshold = expected_pulse_period * (1.0f + ACCEL_TOLERANCE);
    
    if (last_pulse_time > missed_pulse_threshold && 
        current_rpm > 5.0f &&  // Only generate synthetic pulses when motor is running
        !synthetic_pulse_active && 
        ripple_output < 0.5f) {  // No natural pulse currently detected
        
        // Missed pulse detected - generate synthetic pulse to maintain continuity
        synthetic_pulse_active = 1;
        synthetic_pulse_timer = 0.0f;
        
        // Fine-tune angular velocity based on missed pulse timing
        // This helps the model stay synchronized even with missed detections
        if (last_pulse_time > 0.0f) {
            float implied_velocity = ANGULAR_STEP / last_pulse_time;
            angular_velocity = 0.7f * angular_velocity + 0.3f * implied_velocity;
        }
        
        last_pulse_time = 0.0f;  // Reset pulse timer
    }
    
    // Validate pulse timing based on motor dynamics
    float validation_factor = 1.0f;
    
    if (ripple_output > 0.6f) {  // Potential pulse detected
        
        // Check if pulse timing is consistent with expected period
        float period_lower_bound = expected_pulse_period * (1.0f - ACCEL_TOLERANCE);
        float period_upper_bound = expected_pulse_period * (1.0f + ACCEL_TOLERANCE);
        
        // Validate pulse timing
        if (last_pulse_time >= period_lower_bound && last_pulse_time <= period_upper_bound) {
            // Valid pulse - timing matches expected period based on motor dynamics
            validation_factor = 1.0f;
            
            // Reset pulse timer for next pulse
            if (ripple_output > 0.8f) {  // Strong pulse confirmed
                
                // Calculate measured angular velocity from actual pulse period
                if (last_pulse_time > 0.0f) {
                    float measured_velocity = ANGULAR_STEP / last_pulse_time;
                    
                    // Update measured angular velocity with filtering
                    angular_velocity_measured = 0.8f * measured_velocity + 0.2f * angular_velocity_measured;
                    
                    // Store for resistance estimation
                    last_valid_pulse_period = last_pulse_time;
                    valid_pulse_count++;
                    
                    // ===== ADAPTIVE RESISTANCE ESTIMATION =====
                    // Only estimate resistance after collecting enough valid pulses
                    if (valid_pulse_count > 10 && fabsf(motor_current) > MIN_CURRENT_FOR_ESTIMATION) {
                        
                        // Calculate back-EMF from measured angular velocity
                        // Back-EMF: Vb = Kt * omega
                        float back_emf = TORQUE_CONSTANT * angular_velocity_measured;
                        
                        // Calculate voltage drop across resistance
                        // V_motor = I * R + Vb  =>  R = (V - Vb) / I
                        float voltage_drop = motor_voltage - back_emf;
                        
                        // Estimate resistance
                        float resistance_new = voltage_drop / motor_current;
                        
                        // Validate estimate is within plausible range
                        if (resistance_new >= RESISTANCE_MIN && resistance_new <= RESISTANCE_MAX) {
                            // Slowly adapt resistance estimate
                            resistance_estimate = RESISTANCE_ALPHA * resistance_new + 
                                                 (1.0f - RESISTANCE_ALPHA) * resistance_estimate;
                        }
                    }
                    
                    // Update angular velocity with measured value
                    angular_velocity = 0.9f * angular_velocity + 0.1f * measured_velocity;
                }
                
                last_pulse_time = 0.0f;
            }
            
        } else if (last_pulse_time < period_lower_bound) {
            // Pulse came too early - check if acceleration justifies it
            float required_accel = (ANGULAR_STEP / last_pulse_time - angular_velocity) / last_pulse_time;
            float actual_accel_capability = (TORQUE_CONSTANT * motor_current) / ROTOR_INERTIA;
            
            if (fabsf(required_accel) <= fabsf(actual_accel_capability) * (1.0f + ACCEL_TOLERANCE)) {
                // Acceleration is sufficient to explain early pulse
                validation_factor = 0.8f;
            } else {
                // Likely a false detection - suppress
                validation_factor = 0.2f;
            }
            
        } else {
            // Pulse came late - could be deceleration or missed pulse
            validation_factor = 0.7f;
        }
        
        // For very low speeds, relax validation (startup condition)
        if (current_rpm < 10.0f) {
            validation_factor = (validation_factor + 1.0f) * 0.5f;  // Average with full validation
        }
        
    } else {
        // No pulse detected, maintain current state
        validation_factor = 1.0f;
    }
    
    // Apply validation factor to ripple output
    float validated_ripple = ripple_output * validation_factor;
    
    // ===== INJECT SYNTHETIC PULSE IF ACTIVE =====
    
    if (synthetic_pulse_active) {
        // Generate triangular pulse shape for smooth synthetic pulse
        float pulse_progress = synthetic_pulse_timer / SYNTHETIC_PULSE_WIDTH;
        float synthetic_pulse_value;
        
        if (pulse_progress < 0.5f) {
            // Rising edge: 0 to peak
            synthetic_pulse_value = SYNTHETIC_PULSE_AMPLITUDE * (pulse_progress * 2.0f);
        } else {
            // Falling edge: peak to 0
            synthetic_pulse_value = SYNTHETIC_PULSE_AMPLITUDE * ((1.0f - pulse_progress) * 2.0f);
        }
        
        // Combine synthetic pulse with validated ripple (take maximum)
        if (synthetic_pulse_value > validated_ripple) {
            validated_ripple = synthetic_pulse_value;
        }
    }
    
    // Final clipping
    if (validated_ripple > 1.0f) {
        validated_ripple = 1.0f;
    } else if (validated_ripple < 0.0f) {
        validated_ripple = 0.0f;
    }
    
    // Update previous values for next iteration
    current_prev = motor_current;
    voltage_prev = motor_voltage;
    
    return validated_ripple;
}
