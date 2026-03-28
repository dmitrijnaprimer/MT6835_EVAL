/**
 * @file uartcmd_handler.h
 * @brief UART command parser for the MT6835 evaluation board.
 *
 * Receives ASCII command strings from the host (newline-terminated),
 * dispatches to the appropriate handler, and sends responses.
 */

#ifndef UARTCMD_HANDLER_H
#define UARTCMD_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Parse and execute a UART command string.
 *
 * The string is tokenized by spaces. The first token is the command name.
 * Modifies the input buffer (strtok).
 *
 * @param cmd_str Null-terminated command string (will be modified).
 */
void UART_CMD_ProcessCommand(char *cmd_str);

/**
 * @brief Send a full STATUS response over UART.
 *
 * Reads both encoders, motor state, HYST, BW, and emits a single
 * comma-separated STATUS line.
 */
void UART_CMD_SendStatusUpdate(void);

/**
 * @brief Return the current homed state.
 * @return True if the shaft has been homed and hasn't moved since.
 */
bool UART_CMD_GetHomedState(void);

#endif /* UARTCMD_HANDLER_H */