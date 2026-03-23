/**
 * @file uartcmd_handler.h
 * @brief UART command parser and dispatcher for the MT6835 evaluation board.
 *
 * Receives newline-terminated ASCII commands from the host PC over UART1
 * and routes them to encoder, motor, or calibration handlers. Responses
 * use structured prefixes (OK:, ERR:, INFO:, STATUS_) for machine parsing.
 */

#ifndef UARTCMD_HANDLER_H
#define UARTCMD_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Parse and execute a single command string.
 * @param cmd_str Null-terminated command line (modified internally by strtok).
 */
void UART_CMD_ProcessCommand(char *cmd_str);

/**
 * @brief Send a full STATUS response over UART (same as the STATUS command).
 * @note Can be called programmatically to push an unsolicited status update.
 */
void UART_CMD_SendStatusUpdate(void);

/**
 * @brief Return the current homed state.
 * @return True if the shaft has been homed since the last motor movement.
 */
bool UART_CMD_GetHomedState(void);

#endif /* UARTCMD_HANDLER_H */