#include "uartcmd_handler.h"
#include "encoders.h"
#include "main.h"
#include "motor_control.h"
#include "stm32h5xx_hal.h"
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

extern UART_HandleTypeDef huart1;

static int stored_target_rpm = 0;
static int32_t stored_target_steps = 0;
bool homed = false;

static void SendResponse(const char *msg) {
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

static void SendResponseF(const char *fmt, ...) {
  char buf[300];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  SendResponse(buf);
}

static void HandleSetSpeedCmd(char *param);
static void HandleSetStepsCmd(char *param);
static void HandleMoveCWCmd(void);
static void HandleMoveCCWCmd(void);
static void HandleMoveCWStepsCmd(void);
static void HandleMoveCCWStepsCmd(void);
static void HandleStopCmd(void);
static void HandleStatusCmd(void);
static void HandleHomeCmd(void);
static void HandleHomeLIRZeroCmd(void);
static void HandleLIRBitsCmd(char *param);
static void HandleLIRDebugCmd(void);
static void HandleMT6835CalEnCmd(void);
static void HandleMT6835CalDisCmd(void);
static void HandleMT6835ZeroCmd(void);
static void HandleMT6835ReadRegsCmd(void);
static void HandleSetAutocalRpmCmd(char *param);
static void HandleLoadNLCDataCmd(char *param);
static void HandleLoadNLCCmd(char *param);
static void HandleMT6835ReadNLCCmd(void);
static void HandleMT6835ClearNLCCmd(void);
static void HandleMT6835EnableNLCCmd(void);
static void HandleMT6835DisableNLCCmd(void);
static void HandleMT6835ProgramEEPROMCmd(void);

void UART_CMD_ProcessCommand(char *cmd_str) {
  char *token = strtok(cmd_str, " \r\n");
  if (!token)
    return;

  if (strcmp(token, "CONNECT") == 0) {
    SendResponse("OK:Connected to MT6835 Eval Board\n");
  } else if (strcmp(token, "STOP_MOTOR") == 0) {
    HandleStopCmd();
  } else if (strcmp(token, "SET_SPEED") == 0) {
    token = strtok(NULL, " \r\n");
    if (token)
      HandleSetSpeedCmd(token);
  } else if (strcmp(token, "SET_STEPS") == 0) {
    token = strtok(NULL, " \r\n");
    if (token)
      HandleSetStepsCmd(token);
  } else if (strcmp(token, "MOVE_CW") == 0) {
    HandleMoveCWCmd();
  } else if (strcmp(token, "MOVE_CCW") == 0) {
    HandleMoveCCWCmd();
  } else if (strcmp(token, "MOVE_CW_STEPS") == 0) {
    HandleMoveCWStepsCmd();
  } else if (strcmp(token, "MOVE_CCW_STEPS") == 0) {
    HandleMoveCCWStepsCmd();
  } else if (strcmp(token, "STATUS") == 0) {
    HandleStatusCmd();
  } else if (strcmp(token, "HOME_MOTOR") == 0) {
    HandleHomeCmd();
  } else if (strcmp(token, "HOME_LIR_ZERO") == 0) {
    HandleHomeLIRZeroCmd();
  } else if (strcmp(token, "LIR_BITS") == 0) {
    token = strtok(NULL, " \r\n");
    if (token)
      HandleLIRBitsCmd(token);
  } else if (strcmp(token, "LIR_DEBUG") == 0) {
    HandleLIRDebugCmd();
  } else if (strcmp(token, "MT6835_CAL_ENABLE") == 0) {
    HandleMT6835CalEnCmd();
  } else if (strcmp(token, "MT6835_CAL_DISABLE") == 0) {
    HandleMT6835CalDisCmd();
  } else if (strcmp(token, "SET_ZERO_MT6835") == 0) {
    HandleMT6835ZeroCmd();
  } else if (strcmp(token, "MT6835_READ_REGISTERS") == 0) {
    HandleMT6835ReadRegsCmd();
  } else if (strcmp(token, "SET_AUTOCAL_RPM") == 0) {
    token = strtok(NULL, " \r\n");
    if (token)
      HandleSetAutocalRpmCmd(token);
  } else if (strcmp(token, "LOAD_NLC_DATA") == 0) {
    token = strtok(NULL, " \r\n");
    if (token)
      HandleLoadNLCDataCmd(token);
  } else if (strcmp(token, "LOAD_NLC") == 0) {
    token = strtok(NULL, " \r\n");
    if (token)
      HandleLoadNLCCmd(token);
  } else if (strcmp(token, "MT6835_READ_ZERO") == 0) {
    uint16_t zp = ENC_GetMT6835ZeroPos();
    float zp_deg = (float)zp * 360.0f / 4096.0f;
    SendResponseF("OK:ZERO_POS=%u (%.3f deg)\n", zp, zp_deg);
  } else if (strcmp(token, "MT6835_NLC_DUMP") == 0) {
    char buf[400];
    ENC_ReadNLCTable(buf, sizeof(buf));
    SendResponse("NLC_RAW_DUMP:");
    SendResponse(buf);
    SendResponse("\n");
  } else if (strcmp(token, "MT6835_READ_NLC") == 0) {
    HandleMT6835ReadNLCCmd();
  } else if (strcmp(token, "MT6835_CLEAR_NLC") == 0) {
    HandleMT6835ClearNLCCmd();
  } else if (strcmp(token, "MT6835_ENABLE_NLC") == 0) {
    HandleMT6835EnableNLCCmd();
  } else if (strcmp(token, "MT6835_DISABLE_NLC") == 0) {
    HandleMT6835DisableNLCCmd();
  } else if (strcmp(token, "MT6835_PROGRAM_EEPROM") == 0) {
    HandleMT6835ProgramEEPROMCmd();
  } else if (strcmp(token, "TMC2225_MS_4") == 0) {
    HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                      GPIO_PIN_RESET);
    SendResponse("OK:Microsteps=4\n");
  } else if (strcmp(token, "TMC2225_MS_32") == 0) {
    HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                      GPIO_PIN_SET);
    SendResponse("OK:Microsteps=32\n");
  } else {
    SendResponseF("ERR:Unknown cmd '%s'\n", token);
  }
}

static void WaitForMoveComplete(uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();
  while (!MC_IsStepMoveComplete()) {
    if ((HAL_GetTick() - start) > timeout_ms)
      break;
    HAL_Delay(5);
  }
}

static void HandleHomeLIRZeroCmd(void) {
  // Enable motor, 32 microsteps
  HAL_GPIO_WritePin(TMC2225_EN_GPIO_Port, TMC2225_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                    GPIO_PIN_SET);
  HAL_Delay(10);

  uint8_t bits = ENC_GetLIRBits();
  uint32_t max_counts = 1UL << bits;
  int32_t half_counts = (int32_t)(max_counts / 2);

  // Reset LIR offset to get absolute position
  ENC_SetLIRBits(bits);
  HAL_Delay(50);

  uint32_t pos0 = ENC_ReadLIRRaw();
  SendResponseF("INFO:LIR abs pos=%lu/%lu, calibrating direction...\n",
                (unsigned long)pos0, (unsigned long)max_counts);

  // Move 200 microsteps CW to determine direction mapping
  MC_SetTargetRPM(5);
  MC_MoveSteps(200, true);
  WaitForMoveComplete(5000);
  HAL_Delay(300);

  // Reset offset again and read new position
  ENC_SetLIRBits(bits);
  HAL_Delay(50);
  uint32_t pos1 = ENC_ReadLIRRaw();

  // Calculate LIR counts per CW motor step
  int32_t delta = (int32_t)pos1 - (int32_t)pos0;
  if (delta > half_counts)
    delta -= (int32_t)max_counts;
  if (delta < -half_counts)
    delta += (int32_t)max_counts;

  float counts_per_step = (float)delta / 200.0f;

  if (fabsf(counts_per_step) < 0.5f) {
    SendResponse("ERR:LIR not responding to motor movement\n");
    MC_Stop();
    return;
  }

  SendResponseF("INFO:Direction: %.1f LIR counts/motor step\n",
                counts_per_step);

  // Calculate steps to reach LIR raw position 0 from current pos1
  int32_t to_zero = -(int32_t)pos1;
  if (to_zero > half_counts)
    to_zero -= (int32_t)max_counts;
  if (to_zero < -half_counts)
    to_zero += (int32_t)max_counts;

  int32_t steps_needed = (int32_t)((float)to_zero / counts_per_step);
  bool go_cw = (steps_needed > 0);
  int32_t abs_steps = (steps_needed > 0) ? steps_needed : -steps_needed;

  if (abs_steps > 1) {
    SendResponseF("INFO:Coarse move: %ld steps %s\n", (long)abs_steps,
                  go_cw ? "CW" : "CCW");
    MC_SetTargetRPM(5);
    MC_MoveSteps(abs_steps, go_cw);
    WaitForMoveComplete(30000);
    HAL_Delay(500);
  }

  // Fine tune: read again, correct
  ENC_SetLIRBits(bits);
  HAL_Delay(50);
  uint32_t pos2 = ENC_ReadLIRRaw();

  int32_t remaining = -(int32_t)pos2;
  if (remaining > half_counts)
    remaining -= (int32_t)max_counts;
  if (remaining < -half_counts)
    remaining += (int32_t)max_counts;

  int32_t fine_steps = (int32_t)((float)remaining / counts_per_step);
  bool fine_cw = (fine_steps > 0);
  int32_t fine_abs = (fine_steps > 0) ? fine_steps : -fine_steps;

  if (fine_abs > 1) {
    SendResponseF("INFO:Fine move: %ld steps %s\n", (long)fine_abs,
                  fine_cw ? "CW" : "CCW");
    MC_SetTargetRPM(1);
    MC_MoveSteps(fine_abs, fine_cw);
    WaitForMoveComplete(30000);
    HAL_Delay(500);
  }

  // Set zeros at this position
  ENC_SetLIRBits(bits); // reset offset one more time
  HAL_Delay(50);
  ENC_ReadLIRRaw();
  ENC_ReadMT6835Raw();
  ENC_SetLIRZero();
  bool mt_ack = ENC_SetMT6835Zero();
  homed = true;

  uint32_t lir_final = ENC_ReadLIRRaw();
  uint32_t mt_final = ENC_ReadMT6835Raw();
  float lir_deg = (float)lir_final / (float)max_counts * 360.0f;
  float mt_deg = (float)mt_final / 2097152.0f * 360.0f;

  SendResponseF("OK:Homed to LIR zero. LIR=%.6f MT=%.5f (ACK=%s)\n", lir_deg,
                mt_deg, mt_ack ? "OK" : "FAIL");
}

static void HandleSetSpeedCmd(char *param) {
  int rpm = atoi(param);
  if (rpm >= 0 && rpm <= 6000) {
    stored_target_rpm = rpm;
    MC_SetTargetRPM(rpm);
    SendResponseF("OK:Speed=%d RPM\n", rpm);
  } else {
    SendResponse("ERR:RPM out of range 0-6000\n");
  }
}

static void HandleSetStepsCmd(char *param) {
  int32_t steps = atol(param);
  if (steps >= 0) {
    stored_target_steps = steps;
    SendResponseF("OK:Steps=%ld\n", (long)steps);
  } else {
    SendResponse("ERR:Steps must be >= 0\n");
  }
}

static void HandleMoveCWCmd(void) {
  if (stored_target_rpm <= 0) {
    SendResponse("ERR:Set speed first\n");
    return;
  }
  HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                    GPIO_PIN_SET);
  MC_MoveAtRPM((float)stored_target_rpm, true);
  SendResponseF("OK:CW @ %d RPM\n", stored_target_rpm);
}

static void HandleMoveCCWCmd(void) {
  if (stored_target_rpm <= 0) {
    SendResponse("ERR:Set speed first\n");
    return;
  }
  HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                    GPIO_PIN_SET);
  MC_MoveAtRPM((float)stored_target_rpm, false);
  SendResponseF("OK:CCW @ %d RPM\n", stored_target_rpm);
}

static void HandleMoveCWStepsCmd(void) {
  if (stored_target_steps <= 0) {
    SendResponse("ERR:Set steps first\n");
    return;
  }
  if (stored_target_rpm <= 0) {
    SendResponse("ERR:Set speed first\n");
    return;
  }
  HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                    GPIO_PIN_SET);
  MC_MoveSteps(stored_target_steps, true);
  SendResponseF("OK:%ld steps CW\n", (long)stored_target_steps);
}

static void HandleMoveCCWStepsCmd(void) {
  if (stored_target_steps <= 0) {
    SendResponse("ERR:Set steps first\n");
    return;
  }
  if (stored_target_rpm <= 0) {
    SendResponse("ERR:Set speed first\n");
    return;
  }
  HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                    GPIO_PIN_SET);
  MC_MoveSteps(stored_target_steps, false);
  SendResponseF("OK:%ld steps CCW\n", (long)stored_target_steps);
}

static void HandleStopCmd(void) {
  MC_Stop();
  SendResponse("OK:Stopped\n");
}

static void HandleStatusCmd(void) {
  uint32_t lir_raw = ENC_ReadLIRRaw();
  uint32_t mt_raw = ENC_ReadMT6835Raw();
  ENC_MeasureMT6835PWM();

  uint8_t cal_status = ENC_GetMT6835CalStatus();
  const char *cal_str;
  switch (cal_status) {
  case 0:
    cal_str = "None";
    break;
  case 1:
    cal_str = "Running";
    break;
  case 2:
    cal_str = "Failed";
    break;
  case 3:
    cal_str = "OK";
    break;
  default:
    cal_str = "Unknown";
    break;
  }

  int pwm_int = (int)ENC_GetMT6835PWMDutyCyclePercent();
  bool cal_en = (HAL_GPIO_ReadPin(MT6835_CAL_EN_GPIO_Port, MT6835_CAL_EN_Pin) ==
                 GPIO_PIN_SET);
  bool motor_en = MC_IsEnabled();
  bool ms32 = (HAL_GPIO_ReadPin(TMC2225_MICROSTEPS_GPIO_Port,
                                TMC2225_MICROSTEPS_Pin) == GPIO_PIN_SET);

  uint16_t zero_pos = ENC_GetMT6835ZeroPos();

  // Reconstruct physical angle before ZERO_POS subtraction
  // ZERO_POS is 12-bit (0-4095), angle is 21-bit, scale = 2^21/2^12 = 512
  uint32_t zero_pos_21bit = (uint32_t)zero_pos * 512u;
  uint32_t mt_physical = (mt_raw + zero_pos_21bit) & 0x1FFFFF;

  SendResponseF("STATUS_LIR-DA237T_POS:%lu,"
                "STATUS_MT6835_POS:%lu,"
                "STATUS_MT6835_PHYSICAL:%lu,"
                "STATUS_MT6835_ZERO_POS:%u,"
                "STATUS_MT6835_PWM:%d,"
                "STATUS_MT6835_CAL_EN:%s,"
                "STATUS_TMC2225_EN:%s,"
                "STATUS_TMC2225_MICROSTEPS:%s,"
                "STATUS_MT6835_USER_CAL:%s,"
                "STATUS_HOME:%s\n",
                (unsigned long)lir_raw, (unsigned long)mt_raw,
                (unsigned long)mt_physical, (unsigned int)zero_pos, pwm_int,
                cal_en ? "True" : "False", motor_en ? "True" : "False",
                ms32 ? "32" : "4", cal_str, homed ? "True" : "False");
}

static void HandleHomeCmd(void) {
  HAL_GPIO_WritePin(TMC2225_EN_GPIO_Port, TMC2225_EN_Pin, GPIO_PIN_RESET);
  HAL_Delay(10);
  ENC_ReadLIRRaw();
  ENC_ReadMT6835Raw();
  ENC_SetLIRZero();
  bool mt_ack = ENC_SetMT6835Zero();
  homed = true;
  uint32_t lir = ENC_ReadLIRRaw();
  uint32_t mt = ENC_ReadMT6835Raw();
  float lir_deg = (float)lir / (float)(1UL << ENC_GetLIRBits()) * 360.0f;
  float mt_deg = (float)mt / 2097152.0f * 360.0f;
  SendResponseF("OK:Homed LIR=%.6f MT=%.5f (ACK=%s)\n", lir_deg, mt_deg,
                mt_ack ? "OK" : "FAIL");
}

static void HandleLIRBitsCmd(char *param) {
  int bits = atoi(param);
  if (bits >= 21 && bits <= 23) {
    ENC_SetLIRBits((uint8_t)bits);
    SendResponseF("OK:LIR=%d bits\n", bits);
  } else {
    SendResponse("ERR:LIR bits 21-23 only\n");
  }
}

static void HandleLIRDebugCmd(void) {
  char debug_str[512];
  ENC_GetLIRDebugInfo(debug_str, sizeof(debug_str));
  SendResponse("INFO:LIR BiSS-C:\n");
  SendResponse(debug_str);
  SendResponse("\n");
}

static void HandleMT6835CalEnCmd(void) {
  HAL_GPIO_WritePin(MT6835_CAL_EN_GPIO_Port, MT6835_CAL_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
  SendResponse("OK:CAL_EN=HIGH\n");
}

static void HandleMT6835CalDisCmd(void) {
  HAL_GPIO_WritePin(MT6835_CAL_EN_GPIO_Port, MT6835_CAL_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
  SendResponse("OK:CAL_EN=LOW\n");
}

static void HandleMT6835ZeroCmd(void) {
  bool ack = ENC_SetMT6835Zero();
  uint32_t mt = ENC_ReadMT6835Raw();
  float mt_deg = (float)mt / 2097152.0f * 360.0f;
  SendResponseF("OK:MT6835 zero ACK=%s pos=%.5f\n", ack ? "OK" : "FAIL",
                mt_deg);
}

static void HandleMT6835ReadRegsCmd(void) {
  char buf[400];
  ENC_GetMT6835RegisterDump(buf, sizeof(buf));
  SendResponse("MT_REG_DUMP:");
  SendResponse(buf);
  SendResponse("\n");
}

static void HandleSetAutocalRpmCmd(char *param) {
  int rpm = atoi(param);
  ENC_ConfigureAutoCalRPM(rpm);
  uint8_t w, r;
  ENC_GetAutoCalDebugInfo(&w, &r);
  SendResponseF("OK:AutoCal RPM=%d W=0x%02X R=0x%02X\n", rpm, w, r);
}

static void HandleLoadNLCDataCmd(char *param) {
  size_t len = strlen(param);
  if (len != 384) {
    SendResponseF("ERR:NLC need 384 hex chars, got %d\n", (int)len);
    return;
  }
  bool ok = ENC_WriteNLCTable(param);
  SendResponseF("OK:NLC RAM %s\n", ok ? "OK" : "FAIL");
}

static void HandleLoadNLCCmd(char *param) {
  size_t len = strlen(param);
  if (len != 384) {
    SendResponseF("ERR:NLC need 384 hex chars, got %d\n", (int)len);
    return;
  }
  SendResponse("INFO:Writing NLC...\n");
  bool write_ok = ENC_WriteNLCTable(param);
  if (!write_ok) {
    SendResponse("ERR:NLC write failed\n");
    return;
  }
  SendResponse("INFO:Programming EEPROM (6s)...\n");
  bool prog_ok = ENC_ProgramMT6835EEPROM();
  SendResponseF("OK:NLC %s\n", prog_ok ? "programmed" : "EEPROM failed");
}

static void HandleMT6835ReadNLCCmd(void) {
  char buf[400];
  ENC_ReadNLCTable(buf, sizeof(buf));
  SendResponse("NLC_DUMP:");
  SendResponse(buf);
  SendResponse("\n");
}

static void HandleMT6835ClearNLCCmd(void) {
  SendResponse("INFO:Clearing NLC + EEPROM...\n");
  bool ok = ENC_ClearNLCTable();
  SendResponseF("OK:NLC clear %s\n", ok ? "OK" : "EEPROM fail");
}

static void HandleMT6835EnableNLCCmd(void) {
  bool ok = ENC_SetNLCEnabled(true);
  SendResponseF("OK:NLC %s (RAM, use PROGRAM_EEPROM to persist)\n",
                ok ? "enabled" : "failed");
}

static void HandleMT6835DisableNLCCmd(void) {
  bool ok = ENC_SetNLCEnabled(false);
  SendResponseF("OK:NLC %s (RAM, use PROGRAM_EEPROM to persist)\n",
                ok ? "disabled" : "failed");
}

static void HandleMT6835ProgramEEPROMCmd(void) {
  SendResponse("INFO:Programming EEPROM (6s)...\n");
  bool ok = ENC_ProgramMT6835EEPROM();
  SendResponseF("OK:EEPROM %s\n", ok ? "programmed" : "failed");
}

bool UART_CMD_GetHomedState(void) { return homed; }
void UART_CMD_SendStatusUpdate(void) { HandleStatusCmd(); }