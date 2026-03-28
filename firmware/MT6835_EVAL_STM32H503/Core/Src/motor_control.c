/**
 * @file motor_control.c
 * @brief Stepper motor driver using TIM2 for step pulse generation.
 *
 * Controls a TMC2225 stepper driver via STEP/DIR/EN pins.
 * Supports continuous rotation and fixed-step moves.
 * TIM2 runs at 250 MHz, toggles the STEP pin in the ISR.
 */

#include "motor_control.h"
#include "main.h"
#include "stm32h5xx_hal.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;

#define TIM2_CLOCK_FREQ_HZ (250000000UL)
#define MC_MICROSTEPS_CONTINUOUS 32
#define MC_MICROSTEPS_STEPS 32
#define MC_STEPS_PER_REV_FULL 400
#define MC_TOTAL_STEPS_PER_REV                                                 \
  (MC_STEPS_PER_REV_FULL * MC_MICROSTEPS_CONTINUOUS)

static bool motor_running = false;
static int current_target_rpm = 0;
static bool current_direction = true;
static bool motor_enabled = false;
static volatile int32_t remaining_steps = 0;
static volatile int32_t total_requested_steps = 0;
static volatile bool step_pin_state = false;
static volatile bool step_move_complete = false;

extern bool homed;

static void ConfigureStepTimer(int rpm, bool clockwise);
static void ConfigureStepTimerForSteps(int rpm, bool clockwise, int32_t steps);
static void MC_SetEnableState(bool enabled);

/** @brief Enable or disable the TMC2225 driver (EN pin is active low). */
static void MC_SetEnableState(bool enabled) {
  if (enabled) {
    HAL_GPIO_WritePin(TMC2225_EN_GPIO_Port, TMC2225_EN_Pin, GPIO_PIN_RESET);
  } else {
    HAL_GPIO_WritePin(TMC2225_EN_GPIO_Port, TMC2225_EN_Pin, GPIO_PIN_SET);
  }
  motor_running = enabled;
}

/** @brief Configure TIM2 for continuous rotation at the given RPM. */
static void ConfigureStepTimer(int rpm, bool clockwise) {
  HAL_GPIO_WritePin(TMC2225_DIR_GPIO_Port, TMC2225_DIR_Pin,
                    clockwise ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if (rpm <= 0) {
    HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    remaining_steps = 0;
    total_requested_steps = 0;
    step_pin_state = false;
    step_move_complete = false;
    HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_RESET);
    MC_SetEnableState(false);
    return;
  }

  MC_NotifyMovementStarted();

  float step_freq_hz = ((float)rpm / 60.0f) * MC_TOTAL_STEPS_PER_REV;
  uint32_t timer_period_ticks =
      (uint32_t)(TIM2_CLOCK_FREQ_HZ / (step_freq_hz * 2));
  if (timer_period_ticks < 1)
    timer_period_ticks = 1;

  __HAL_TIM_SET_AUTORELOAD(&htim2, timer_period_ticks - 1);
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);
  MC_SetEnableState(true);
  remaining_steps = 0;
  total_requested_steps = 0;
  step_move_complete = false;
  step_pin_state = false;
  HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_RESET);
}

/** @brief Configure TIM2 for a fixed number of steps, then auto-stop. */
static void ConfigureStepTimerForSteps(int rpm, bool clockwise, int32_t steps) {
  HAL_GPIO_WritePin(TMC2225_DIR_GPIO_Port, TMC2225_DIR_Pin,
                    clockwise ? GPIO_PIN_SET : GPIO_PIN_RESET);

  if (rpm <= 0 || steps <= 0) {
    HAL_TIM_Base_Stop_IT(&htim2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    remaining_steps = 0;
    total_requested_steps = 0;
    step_pin_state = false;
    step_move_complete = false;
    HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_RESET);
    MC_SetEnableState(false);
    return;
  }

  MC_NotifyMovementStarted();

  float step_freq_hz = ((float)rpm / 60.0f) * MC_TOTAL_STEPS_PER_REV;
  uint32_t timer_period_ticks =
      (uint32_t)(TIM2_CLOCK_FREQ_HZ / (step_freq_hz * 2));
  if (timer_period_ticks < 1)
    timer_period_ticks = 1;

  __HAL_TIM_SET_AUTORELOAD(&htim2, timer_period_ticks - 1);
  __HAL_TIM_SET_COUNTER(&htim2, 0);

  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);
  MC_SetEnableState(true);
  remaining_steps = steps * 2;
  total_requested_steps = steps;
  step_move_complete = false;
  step_pin_state = false;
  HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_RESET);
}

/** @brief TIM2 ISR callback. Toggles STEP pin, counts remaining steps. */
void MC_StepTimerCallback(void) {
  if (remaining_steps > 0) {
    if (!step_pin_state) {
      HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_SET);
      step_pin_state = true;
    } else {
      HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin,
                        GPIO_PIN_RESET);
      step_pin_state = false;
      remaining_steps -= 2;

      if (remaining_steps <= 0) {
        HAL_TIM_Base_Stop_IT(&htim2);
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin,
                          GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
        remaining_steps = 0;
        step_move_complete = true;
      }
    }
  } else if (remaining_steps == 0 && total_requested_steps == 0) {
    // Continuous mode
    if (!step_pin_state) {
      HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_SET);
      step_pin_state = true;
    } else {
      HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin,
                        GPIO_PIN_RESET);
      step_pin_state = false;
    }
  }
}

/** @brief Initialize motor GPIO and timer state. */
void MC_Init(void) {
  HAL_GPIO_WritePin(LED_4_GPIO_Port, LED_4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_5_GPIO_Port, LED_5_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TMC2225_EN_GPIO_Port, TMC2225_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(TMC2225_DIR_GPIO_Port, TMC2225_DIR_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                    GPIO_PIN_SET);
  HAL_Delay(10);

  motor_running = false;
  current_target_rpm = 0;
  current_direction = true;
  motor_enabled = false;
  remaining_steps = 0;
  total_requested_steps = 0;
  step_pin_state = false;
  step_move_complete = false;

  HAL_TIM_Base_Stop_IT(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
}

/** @brief Start continuous rotation at the given RPM. */
void MC_MoveAtRPM(float rpm, bool clockwise) {
  int rpm_int = (int)rpm;
  if (rpm_int <= 0) {
    MC_Stop();
    return;
  }
  ConfigureStepTimer(rpm_int, clockwise);
  current_target_rpm = rpm_int;
  current_direction = clockwise;
}

/** @brief Move a fixed number of microsteps, then stop. */
void MC_MoveSteps(int32_t steps, bool clockwise) {
  if (steps <= 0 || current_target_rpm <= 0) {
    MC_Stop();
    return;
  }
  ConfigureStepTimerForSteps(current_target_rpm, clockwise, steps);
  motor_running = true;
}

/** @brief Emergency stop. Halts timer and disables driver. */
void MC_Stop(void) {
  HAL_TIM_Base_Stop_IT(&htim2);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_RESET);
  MC_SetEnableState(false);
  remaining_steps = 0;
  total_requested_steps = 0;
  step_pin_state = false;
  step_move_complete = false;
}

bool MC_IsRunning(void) { return motor_running; }
bool MC_IsEnabled(void) {
  return (HAL_GPIO_ReadPin(TMC2225_EN_GPIO_Port, TMC2225_EN_Pin) ==
          GPIO_PIN_RESET);
}
bool MC_IsStepMoveComplete(void) { return step_move_complete; }
float MC_GetTargetRPM(void) { return (float)current_target_rpm; }
void MC_SetTargetRPM(int rpm) {
  if (rpm >= 0)
    current_target_rpm = rpm;
}
void MC_SetMicrostepMode(bool use_32_steps) {}

/** @brief Clear the homed flag when motor starts moving. */
void MC_NotifyMovementStarted(void) {
  extern bool homed;
  if (homed)
    homed = false;
}