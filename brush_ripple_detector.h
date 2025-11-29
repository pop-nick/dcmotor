/**
 * @file brush_ripple_detector.h
 * @brief DC Motor Brush Commutation Ripple Detection Header
 */

#ifndef BRUSH_RIPPLE_DETECTOR_H
#define BRUSH_RIPPLE_DETECTOR_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Detects brush commutation ripple signal from motor measurements
 * 
 * This function processes current and voltage measurements from a DC motor
 * with 18 brush commutations per revolution to extract the ripple signal
 * caused by brush commutation events.
 * 
 * @param motor_current Current measurement in Amperes [A]
 * @param motor_voltage Voltage measurement in Volts [V]
 * @return float Normalized ripple signal (0.0 to 1.0, peaks indicate commutation)
 * 
 * @note Call this function at 10 kHz (every 100us) for proper operation
 * @note The function maintains internal state; one instance per motor
 */
float brush_ripple_detector(float motor_current, float motor_voltage);

#ifdef __cplusplus
}
#endif

#endif // BRUSH_RIPPLE_DETECTOR_H
