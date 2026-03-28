/**
 * @file uartcmd_handler.c
 * @brief UART command processor for MT6835 evaluation board.
 *
 * Parses newline-terminated ASCII commands from the host PC and dispatches
 * them to the appropriate encoder, motor, or calibration handler. Responses
 * are sent back as single-line ASCII strings prefixed with OK:, ERR:, INFO:,
 * or STATUS_ for structured telemetry.
 */

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

/** @brief Target RPM stored by SET_SPEED, used by MOVE_CW/CCW commands. */
static int stored_target_rpm = 0;

/** @brief Target step count stored by SET_STEPS, used by MOVE_CW/CCW_STEPS. */
static int32_t stored_target_steps = 0;

/** @brief True after a successful HOME command. Cleared on any motor movement.
 */
bool homed = false;

/* ------------------------------------------------------------------ */
/* UART helpers                                                       */
/* ------------------------------------------------------------------ */

/** @brief Send a null-terminated string over UART1. */
static void SendResponse(const char *msg) {
  HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}

/** @brief Send a printf-formatted string over UART1 (max 300 chars). */
static void SendResponseF(const char *fmt, ...) {
  char buf[300];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);
  SendResponse(buf);
}

/* ------------------------------------------------------------------ */
/* Forward declarations                                               */
/* ------------------------------------------------------------------ */

static void HandleSetSpeedCmd(char *param);
static void HandleSetStepsCmd(char *param);
static void HandleMoveCWCmd(void);
static void HandleMoveCCWCmd(void);
static void HandleMoveCWStepsCmd(void);
static void HandleMoveCCWStepsCmd(void);
static void HandleStopCmd(void);
static void HandleStatusCmd(void);
static void HandleHomeCmd(void);
static void HandleSetMT6835ZeroCmd(void);
static void HandleLIRBitsCmd(char *param);
static void HandleLIRDebugCmd(void);
static void HandleMT6835CalEnCmd(void);
static void HandleMT6835CalDisCmd(void);
static void HandleMT6835ReadRegsCmd(void);
static void HandleSetAutocalRpmCmd(char *param);
static void HandleLoadNLCCmd(char *param);
static void HandleMT6835ReadNLCCmd(void);
static void HandleMT6835ClearNLCCmd(void);
static void HandleMT6835EnableNLCCmd(void);
static void HandleMT6835DisableNLCCmd(void);
static void HandleMT6835ProgramEEPROMCmd(void);

/* ------------------------------------------------------------------ */
/* Command dispatcher                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Parse and execute a single command string.
 * @param cmd_str Null-terminated command (will be modified by strtok).
 */
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
  } else if (strcmp(token, "HOME") == 0) {
    HandleHomeCmd();
  } else if (strcmp(token, "SET_ZERO_MT6835") == 0) {
    HandleSetMT6835ZeroCmd();
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
  } else if (strcmp(token, "MT6835_READ_REGISTERS") == 0) {
    HandleMT6835ReadRegsCmd();
  } else if (strcmp(token, "SET_AUTOCAL_RPM") == 0) {
    token = strtok(NULL, " \r\n");
    if (token)
      HandleSetAutocalRpmCmd(token);
  } else if (strcmp(token, "LOAD_NLC") == 0) {
    token = strtok(NULL, " \r\n");
    if (token)
      HandleLoadNLCCmd(token);
  } else if (strcmp(token, "MT6835_READ_ZERO") == 0) {
    uint16_t zp = ENC_GetMT6835ZeroPos();
    SendResponseF("OK:ZERO_POS=%u\n", zp);
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
  } else if (strcmp(token, "MT6835_SET_HYST") == 0) {
    token = strtok(NULL, " \r\n");
    if (token) {
      int val = atoi(token);
      if (val >= 0 && val <= 7) {
        uint8_t reg = ENC_ReadMT6835Register(MT6835_REG_DIR_HYST);
        reg = (reg & 0xF8) | (val & 0x07);
        ENC_WriteMT6835Register(MT6835_REG_DIR_HYST, reg);
        SendResponseF("OK:HYST=%d (reg=0x%02X)\n", val, reg);
      } else {
        SendResponse("ERR:HYST 0-7 only\n");
      }
    }
  } else if (strcmp(token, "MT6835_SET_BW") == 0) {
    token = strtok(NULL, " \r\n");
    if (token) {
      int val = atoi(token);
      if (val >= 0 && val <= 7) {
        uint8_t reg = ENC_ReadMT6835Register(MT6835_REG_BW);
        reg = (reg & 0xF8) | (val & 0x07);
        ENC_WriteMT6835Register(MT6835_REG_BW, reg);
        SendResponseF("OK:BW=%d (reg=0x%02X)\n", val, reg);
      } else {
        SendResponse("ERR:BW 0-7 only\n");
      }
    }
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

/* ------------------------------------------------------------------ */
/* Motor helpers                                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief Block until the current step-move completes or timeout expires.
 * @param timeout_ms Maximum wait time in milliseconds.
 */
static void WaitForMoveComplete(uint32_t timeout_ms) {
  uint32_t start = HAL_GetTick();
  while (!MC_IsStepMoveComplete()) {
    if ((HAL_GetTick() - start) > timeout_ms)
      break;
    HAL_Delay(5);
  }
}

/* ------------------------------------------------------------------ */
/* HOME — move shaft to LIR absolute zero position                    */
/* ------------------------------------------------------------------ */

/**
 * @brief Move the shaft to the LIR-DA237T absolute zero position.
 *
 * Uses a coarse-then-fine approach: reads the LIR absolute position, calculates
 * direction and distance to raw position zero, moves there, and sets the LIR
 * software offset so subsequent reads start from 0 degrees.
 *
 * Does NOT change the MT6835 ZERO_POS register. Use SET_ZERO_MT6835 separately
 * during commissioning — changing ZERO_POS invalidates the NLC table alignment.
 */
static void HandleHomeCmd(void) {
  HAL_GPIO_WritePin(TMC2225_EN_GPIO_Port, TMC2225_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(TMC2225_MICROSTEPS_GPIO_Port, TMC2225_MICROSTEPS_Pin,
                    GPIO_PIN_SET);
  HAL_Delay(10);

  uint8_t bits = ENC_GetLIRBits();
  uint32_t max_counts = 1UL << bits;
  int32_t half_counts = (int32_t)(max_counts / 2);

  /* Reset LIR offset to read absolute physical position. */
  ENC_SetLIRBits(bits);
  HAL_Delay(50);

  uint32_t pos0 = ENC_ReadLIRRaw();
  SendResponseF("INFO:LIR abs pos=%u/%u\n", (unsigned int)pos0,
                (unsigned int)max_counts);

  /* Move 200 microsteps CW to determine direction mapping. */
  MC_SetTargetRPM(5);
  MC_MoveSteps(200, true);
  WaitForMoveComplete(5000);
  HAL_Delay(300);

  ENC_SetLIRBits(bits);
  HAL_Delay(50);
  uint32_t pos1 = ENC_ReadLIRRaw();

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

  /* Coarse move to LIR raw zero. */
  int32_t to_zero = -(int32_t)pos1;
  if (to_zero > half_counts)
    to_zero -= (int32_t)max_counts;
  if (to_zero < -half_counts)
    to_zero += (int32_t)max_counts;

  int32_t steps_needed = (int32_t)((float)to_zero / counts_per_step);
  bool go_cw = (steps_needed > 0);
  int32_t abs_steps = (steps_needed > 0) ? steps_needed : -steps_needed;

  if (abs_steps > 1) {
    MC_SetTargetRPM(5);
    MC_MoveSteps(abs_steps, go_cw);
    WaitForMoveComplete(30000);
    HAL_Delay(500);
  }

  /* Fine correction pass. */
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
    MC_SetTargetRPM(1);
    MC_MoveSteps(fine_abs, fine_cw);
    WaitForMoveComplete(30000);
    HAL_Delay(500);
  }

  /* Set LIR software zero at this position. MT6835 zero is NOT touched. */
  ENC_SetLIRBits(bits);
  HAL_Delay(50);
  /* Read final positions. LIR reports true physical position (no software
   * offset). */
  ENC_ReadLIRRaw();
  homed = true;

  uint32_t lir_final = ENC_ReadLIRRaw();
  uint32_t mt_final = ENC_ReadMT6835Raw();

  SendResponseF("OK:Homed LIR=%u MT=%u\n", (unsigned int)lir_final,
                (unsigned int)mt_final);
}

/* ------------------------------------------------------------------ */
/* Motor commands                                                     */
/* ------------------------------------------------------------------ */

/** @brief Set the target speed for subsequent motor commands (0-6000 RPM). */
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

/** @brief Set the step count for MOVE_CW_STEPS / MOVE_CCW_STEPS. */
static void HandleSetStepsCmd(char *param) {
  int32_t steps = atol(param);
  if (steps >= 0) {
    stored_target_steps = steps;
    SendResponseF("OK:Steps=%d\n", (int)steps);
  } else {
    SendResponse("ERR:Steps must be >= 0\n");
  }
}

/** @brief Start continuous clockwise rotation at stored RPM. */
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

/** @brief Start continuous counter-clockwise rotation at stored RPM. */
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

/** @brief Move the stored number of steps clockwise. */
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
  SendResponseF("OK:%d steps CW\n", (int)stored_target_steps);
}

/** @brief Move the stored number of steps counter-clockwise. */
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
  SendResponseF("OK:%d steps CCW\n", (int)stored_target_steps);
}

/** @brief Emergency stop — disable the motor driver immediately. */
static void HandleStopCmd(void) {
  MC_Stop();
  SendResponse("OK:Stopped\n");
}

/* ------------------------------------------------------------------ */
/* Status                                                             */
/* ------------------------------------------------------------------ */

/**
 * @brief Read all sensor values and send a structured status line.
 *
 * The status line contains comma-separated key:value pairs consumed by the
 * Python GUI for live display and data collection.
 */
static void HandleStatusCmd(void) {
  uint32_t lir_raw = ENC_ReadLIRRaw();
  uint32_t mt_raw = ENC_ReadMT6835Raw();

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

  bool motor_en = MC_IsEnabled();
  uint16_t zero_pos = ENC_GetMT6835ZeroPos();

  /* Reconstruct pre-ZERO_POS physical angle (12-bit to 21-bit scaling). */
  uint32_t zero_pos_21bit = (uint32_t)zero_pos * 512u;
  uint32_t mt_physical = (mt_raw + zero_pos_21bit) & 0x1FFFFF;

  uint8_t reg_0d = ENC_ReadMT6835Register(MT6835_REG_DIR_HYST);
  uint8_t hyst_val = reg_0d & 0x07;
  uint8_t reg_11 = ENC_ReadMT6835Register(MT6835_REG_BW);
  uint8_t bw_val = reg_11 & 0x07;

  SendResponseF("STATUS_LIR-DA237T_POS:%u,"
                "STATUS_MT6835_POS:%u,"
                "STATUS_MT6835_RAW:%u,"
                "STATUS_MT6835_ZERO_POS:%u,"
                "STATUS_TMC2225_EN:%s,"
                "STATUS_MT6835_USER_CAL:%s,"
                "STATUS_HOME:%s,"
                "STATUS_MT6835_HYST:%u,"
                "STATUS_MT6835_BW:%u\n",
                (unsigned int)lir_raw, (unsigned int)mt_raw,
                (unsigned int)mt_physical, (unsigned int)zero_pos,
                motor_en ? "True" : "False", cal_str, homed ? "True" : "False",
                (unsigned int)hyst_val, (unsigned int)bw_val);
}

/* ------------------------------------------------------------------ */
/* MT6835 configuration commands                                      */
/* ------------------------------------------------------------------ */

/**
 * @brief Set current shaft position as MT6835 zero (ZERO_POS register).
 *
 * This is a commissioning-time operation. Changing ZERO_POS shifts the NLC
 * lookup table grid, so any existing NLC calibration becomes invalid.
 */
static void HandleSetMT6835ZeroCmd(void) {
  bool ack = ENC_SetMT6835Zero();
  uint32_t mt = ENC_ReadMT6835Raw();
  uint16_t zp = ENC_GetMT6835ZeroPos();
  SendResponseF("OK:MT6835 zero ACK=%s MT=%u ZERO_POS=%u\n",
                ack ? "OK" : "FAIL", (unsigned int)mt, zp);
}

/** @brief Set LIR-DA237T BiSS-C resolution (21, 22, or 23 bits). */
static void HandleLIRBitsCmd(char *param) {
  int bits = atoi(param);
  if (bits >= 21 && bits <= 23) {
    ENC_SetLIRBits((uint8_t)bits);
    SendResponseF("OK:LIR=%d bits\n", bits);
  } else {
    SendResponse("ERR:LIR bits 21-23 only\n");
  }
}

/** @brief Print LIR-DA237T BiSS-C debug information (raw SPI frame, CRC). */
static void HandleLIRDebugCmd(void) {
  char debug_str[512];
  ENC_GetLIRDebugInfo(debug_str, sizeof(debug_str));
  SendResponse("INFO:LIR BiSS-C:\n");
  SendResponse(debug_str);
  SendResponse("\n");
}

/** @brief Assert CAL_EN pin to start MT6835 user auto-calibration. */
static void HandleMT6835CalEnCmd(void) {
  HAL_GPIO_WritePin(MT6835_CAL_EN_GPIO_Port, MT6835_CAL_EN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_SET);
  SendResponse("OK:CAL_EN=HIGH\n");
}

/** @brief De-assert CAL_EN pin to end MT6835 user auto-calibration. */
static void HandleMT6835CalDisCmd(void) {
  HAL_GPIO_WritePin(MT6835_CAL_EN_GPIO_Port, MT6835_CAL_EN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_2_GPIO_Port, LED_2_Pin, GPIO_PIN_RESET);
  SendResponse("OK:CAL_EN=LOW\n");
}

/** @brief Dump all MT6835 configuration registers (0x001-0x012). */
static void HandleMT6835ReadRegsCmd(void) {
  char buf[400];
  ENC_GetMT6835RegisterDump(buf, sizeof(buf));
  SendResponse("MT_REG_DUMP:");
  SendResponse(buf);
  SendResponse("\n");
}

/**
 * @brief Configure the AUTOCAL_FREQ register for the given RPM.
 * @param param ASCII decimal RPM string.
 */
static void HandleSetAutocalRpmCmd(char *param) {
  int rpm = atoi(param);
  ENC_ConfigureAutoCalRPM(rpm);
  SendResponseF("OK:AutoCal RPM=%d\n", rpm);
}

/* ------------------------------------------------------------------ */
/* NLC table commands                                                 */
/* ------------------------------------------------------------------ */

/**
 * @brief Write an NLC table to MT6835 registers and program to EEPROM.
 * @param param 384-character hex string (192 bytes packed MSB-first).
 *
 * This writes the NLC data to registers 0x013-0x0D2, enables NLC_EN,
 * and programs the entire register map into EEPROM (~6 seconds).
 */
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

/** @brief Read back the NLC table from MT6835 registers (for verification). */
static void HandleMT6835ReadNLCCmd(void) {
  char buf[400];
  ENC_ReadNLCTable(buf, sizeof(buf));
  SendResponse("NLC_DUMP:");
  SendResponse(buf);
  SendResponse("\n");
}

/** @brief Clear the NLC table, disable NLC_EN, and program EEPROM. */
static void HandleMT6835ClearNLCCmd(void) {
  SendResponse("INFO:Clearing NLC + EEPROM...\n");
  bool ok = ENC_ClearNLCTable();
  SendResponseF("OK:NLC clear %s\n", ok ? "OK" : "EEPROM fail");
}

/** @brief Enable NLC correction in RAM (not persisted until EEPROM program). */
static void HandleMT6835EnableNLCCmd(void) {
  bool ok = ENC_SetNLCEnabled(true);
  SendResponseF("OK:NLC %s\n", ok ? "enabled" : "failed");
}

/** @brief Disable NLC correction in RAM (not persisted until EEPROM program).
 */
static void HandleMT6835DisableNLCCmd(void) {
  bool ok = ENC_SetNLCEnabled(false);
  SendResponseF("OK:NLC %s\n", ok ? "disabled" : "failed");
}

/** @brief Program all MT6835 registers to EEPROM (~6 second blocking
 * operation). */
static void HandleMT6835ProgramEEPROMCmd(void) {
  SendResponse("INFO:Programming EEPROM (6s)...\n");
  bool ok = ENC_ProgramMT6835EEPROM();
  SendResponseF("OK:EEPROM %s\n", ok ? "programmed" : "failed");
}

/* ------------------------------------------------------------------ */
/* Public interface                                                   */
/* ------------------------------------------------------------------ */

bool UART_CMD_GetHomedState(void) { return homed; }

void UART_CMD_SendStatusUpdate(void) { HandleStatusCmd(); }