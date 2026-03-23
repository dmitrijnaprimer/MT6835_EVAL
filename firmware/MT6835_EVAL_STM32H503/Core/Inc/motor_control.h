/**
 * @file motor_control.h
 * @brief Stepper motor driver interface for TMC2225 via TIM2.
 *
 * Controls a stepper motor through the TMC2225 driver using timer-generated
 * step pulses. Supports continuous rotation at a given RPM and fixed step-count
 * moves. The motor uses 400 full steps per revolution with 32 microsteps,
 * giving 12800 microsteps per revolution.
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/** @brief Initialize motor control hardware and set all outputs to safe state.
 */
void MC_Init(void);

/**
 * @brief Start continuous rotation at the specified speed.
 * @param rpm       Target speed in revolutions per minute.
 * @param clockwise True for CW rotation, false for CCW.
 */
void MC_MoveAtRPM(float rpm, bool clockwise);

/**
 * @brief Move a fixed number of microsteps and stop.
 * @param steps     Number of microsteps to move (at 32 microstep mode).
 * @param clockwise True for CW, false for CCW.
 */
void MC_MoveSteps(int32_t steps, bool clockwise);

/**
 * @brief Set the TMC2225 microstep mode via the MS pin.
 * @param use_32_steps True for 32 microsteps, false for 4 microsteps.
 * @note Currently a no-op; microstep mode is set directly in the command
 * handler.
 */
void MC_SetMicrostepMode(bool use_32_steps);

/** @brief Emergency stop — disable the driver and halt all step generation. */
void MC_Stop(void);

/** @brief Return true if the motor driver is currently generating steps. */
bool MC_IsRunning(void);

/** @brief Return true if the TMC2225 enable pin is asserted (driver active). */
bool MC_IsEnabled(void);

/** @brief Return true if a step-count move has completed. */
bool MC_IsStepMoveComplete(void);

/** @brief Return the current target RPM. */
float MC_GetTargetRPM(void);

/**
 * @brief Store the target RPM for subsequent move commands.
 * @param rpm Speed in RPM (0-6000).
 */
void MC_SetTargetRPM(int rpm);

/**
 * @brief Notify the motor controller that a movement has started.
 * @note Clears the homed flag since the shaft position has changed.
 */
void MC_NotifyMovementStarted(void);

/**
 * @brief TIM2 interrupt callback for step pulse generation.
 * @note Called from the HAL timer interrupt at twice the step frequency.
 *       Toggles the STEP pin on each call to generate rising and falling edges.
 */
void MC_StepTimerCallback(void);

#endif /* MOTOR_CONTROL_H */