/**
 * @file motor_control.c
 * @brief TMC2225 stepper motor driver using TIM2 for step pulse generation.
 *
 * Drives a 400-step motor with 32 microsteps (12800 usteps/rev) through the
 * TMC2225 driver IC. TIM2 generates step pulses at the calculated frequency
 * for the requested RPM. Supports continuous rotation and fixed step-count
 * moves with automatic stop on completion.
 */

#include "motor_control.h"
#include "main.h"
#include "stm32h5xx_hal.h"
#include <math.h>

extern TIM_HandleTypeDef htim2;

#define TIM2_CLOCK_FREQ_HZ (250000000UL) /**< TIM2 input clock (APB1). */
#define MC_MICROSTEPS_CONTINUOUS 32      /**< Microstep mode for moves. */
#define MC_MICROSTEPS_STEPS 32
#define MC_STEPS_PER_REV_FULL 400 /**< Full steps per revolution. */
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

/**
 * @brief Set the TMC2225 enable pin state.
 * @param enabled True to enable driver (EN LOW), false to disable (EN HIGH).
 */
static void MC_SetEnableState(bool enabled) {
  HAL_GPIO_WritePin(TMC2225_EN_GPIO_Port, TMC2225_EN_Pin,
                    enabled ? GPIO_PIN_RESET : GPIO_PIN_SET);
  motor_running = enabled;
}

/**
 * @brief Configure TIM2 for continuous rotation at the given RPM.
 * @param rpm       Target speed (0 to stop).
 * @param clockwise Direction.
 */
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
  uint32_t period = (uint32_t)(TIM2_CLOCK_FREQ_HZ / (step_freq_hz * 2));
  if (period < 1)
    period = 1;

  __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
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

/**
 * @brief Configure TIM2 for a fixed step-count move.
 * @param rpm       Step frequency derived from this RPM.
 * @param clockwise Direction.
 * @param steps     Number of microsteps to execute.
 */
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
  uint32_t period = (uint32_t)(TIM2_CLOCK_FREQ_HZ / (step_freq_hz * 2));
  if (period < 1)
    period = 1;

  __HAL_TIM_SET_AUTORELOAD(&htim2, period - 1);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  HAL_GPIO_WritePin(LED_1_GPIO_Port, LED_1_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start_IT(&htim2);
  MC_SetEnableState(true);
  remaining_steps = steps * 2; /* x2: each tick toggles the pin once. */
  total_requested_steps = steps;
  step_move_complete = false;
  step_pin_state = false;
  HAL_GPIO_WritePin(TMC2225_STEP_GPIO_Port, TMC2225_STEP_Pin, GPIO_PIN_RESET);
}

/**
 * @brief TIM2 period-elapsed ISR callback for step pulse generation.
 *
 * Called at twice the step frequency. Each call toggles the STEP pin.
 * In step-count mode, decrements the remaining count on each falling edge
 * and stops when complete. In continuous mode, toggles indefinitely.
 */
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

/** @brief Initialize motor control: set all GPIOs to safe state, stop timer. */
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

void MC_MoveSteps(int32_t steps, bool clockwise) {
  if (steps <= 0 || current_target_rpm <= 0) {
    MC_Stop();
    return;
  }
  ConfigureStepTimerForSteps(current_target_rpm, clockwise, steps);
  motor_running = true;
}

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

/** @brief Microstep mode is set directly via GPIO in the command handler. */
void MC_SetMicrostepMode(bool use_32_steps) { (void)use_32_steps; }

void MC_NotifyMovementStarted(void) {
  extern bool homed;
  if (homed)
    homed = false;
}