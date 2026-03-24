/**
 * @file encoders.h
 * @brief Driver interface for MT6835 magnetic encoder and LIR-DA237T optical
 * encoder.
 *
 * MT6835: 21-bit AMR angle encoder, communicates via SPI1 (mode 3).
 * LIR-DA237T: 23-bit optical reference encoder, communicates via SPI2 (BiSS-C).
 */

#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @name MT6835 SPI command opcodes (upper nibble of byte 0). */
/** @{ */
#define MT6835_CMD_READ 0x3        /**< Read one register. */
#define MT6835_CMD_WRITE 0x6       /**< Write one register. */
#define MT6835_CMD_BURST_READ 0xA  /**< Burst read angle (0x003-0x006). */
#define MT6835_CMD_AUTO_ZERO 0x5   /**< Set current position as zero. */
#define MT6835_CMD_PROG_EEPROM 0xC /**< Program registers to EEPROM (~6s). */
/** @} */

/** @name MT6835 register addresses. */
/** @{ */
#define MT6835_REG_ANGLE_H 0x003   /**< ANGLE[20:13]. */
#define MT6835_REG_ANGLE_M 0x004   /**< ANGLE[12:5]. */
#define MT6835_REG_ANGLE_L 0x005   /**< ANGLE[4:0] | STATUS[2:0]. */
#define MT6835_REG_CRC 0x006       /**< CRC8 (polynomial X^8+X^2+X+1). */
#define MT6835_REG_ABZ_RES_H 0x007 /**< ABZ_RES[13:6]. */
#define MT6835_REG_ABZ_RES_L 0x008 /**< ABZ_RES[5:0] | ABZ_OFF | AB_SWAP. */
#define MT6835_REG_ZERO_H 0x009    /**< ZERO_POS[11:4]. */
#define MT6835_REG_ZERO_L 0x00A    /**< ZERO_POS[3:0] | Z_EDGE | Z_PUL_WID. */
#define MT6835_REG_Z_UVW 0x00B /**< Z_PHASE | UVW_MUX | UVW_OFF | UVW_RES. */
#define MT6835_REG_PWM_NLC                                                     \
  0x00C /**< NLC_EN(bit5) | PWM_FQ | PWM_POL | PWM_SEL. */
#define MT6835_REG_DIR_HYST 0x00D   /**< ROT_DIR(bit3) | HYST[2:0]. */
#define MT6835_REG_AUTOCAL 0x00E    /**< GPIO_DS | AUTOCAL_FREQ[2:0]. */
#define MT6835_REG_BW 0x011         /**< BW[2:0] system bandwidth. */
#define MT6835_REG_NLC_START 0x013  /**< First NLC table byte (192 total). */
#define MT6835_REG_NLC_END 0x0D2    /**< Last NLC table byte. */
#define MT6835_REG_CAL_STATUS 0x113 /**< Calibration status [7:6]. */
/** @} */

#define MT6835_NLC_TABLE_SIZE 192 /**< NLC table: 192 bytes (256 x 6-bit). */
#define MT6835_ACK_SUCCESS 0x55   /**< SPI acknowledge byte. */

/* ---- Initialization ---- */

/** @brief Initialize both encoders. Call after SPI peripheral init. */
void ENC_Init(void);

/* ---- MT6835 angle reading ---- */

/**
 * @brief Burst-read the MT6835 21-bit angle with CRC8 verification.
 * @return 21-bit angle (0-2097151). Last valid value on CRC error.
 */
uint32_t ENC_ReadMT6835Raw(void);

/** @brief Return last valid 21-bit angle without SPI transaction. */
uint32_t ENC_GetMT6835Raw(void);

/** @brief Return last angle in degrees (0.0-360.0). */
float ENC_GetMT6835Degrees(void);

/** @brief True if last SPI read had CRC mismatch. */
bool ENC_GetMT6835CRCError(void);

/* ---- MT6835 register access ---- */

/**
 * @brief Write one byte to an MT6835 register.
 * @param address 12-bit register address.
 * @param data    Byte to write.
 */
void ENC_WriteMT6835Register(uint16_t address, uint8_t data);

/**
 * @brief Read one byte from an MT6835 register.
 * @param address 12-bit register address.
 * @return Register value.
 */
uint8_t ENC_ReadMT6835Register(uint16_t address);

/* ---- MT6835 commands ---- */

/**
 * @brief Set current shaft position as MT6835 zero (ZERO_POS register).
 * @return True if chip acknowledged (0x55).
 * @warning Changing ZERO_POS invalidates NLC table alignment.
 */
bool ENC_SetMT6835Zero(void);

/**
 * @brief Program all registers to EEPROM. Blocks ~6.5 seconds.
 * @return True if chip acknowledged.
 */
bool ENC_ProgramMT6835EEPROM(void);

/**
 * @brief Read ZERO_POS[11:0] from registers 0x009-0x00A.
 * @return 12-bit value (0-4095). 1 LSB = 0.088 degrees.
 */
uint16_t ENC_GetMT6835ZeroPos(void);

/* ---- MT6835 auto-calibration ---- */

/**
 * @brief Read calibration status from register 0x113[7:6].
 * @return 0=none, 1=running, 2=failed, 3=success.
 */
uint8_t ENC_GetMT6835CalStatus(void);

/**
 * @brief Configure AUTOCAL_FREQ for the given rotation speed.
 * @param rpm Calibration speed in RPM.
 */
void ENC_ConfigureAutoCalRPM(int rpm);

/**
 * @brief Get autocal register write/readback values for debugging.
 * @param written  Value written to AUTOCAL register.
 * @param readback Value read back after write.
 */
void ENC_GetAutoCalDebugInfo(uint8_t *written, uint8_t *readback);

/* ---- MT6835 NLC table ---- */

/**
 * @brief Write 192-byte NLC table to registers and enable NLC_EN.
 * @param hex_string 384-char hex string (MSB-first packed 6-bit values).
 * @return True on success. Does NOT program EEPROM.
 */
bool ENC_WriteNLCTable(const char *hex_string);

/**
 * @brief Read NLC table from registers into a hex string.
 * @param buffer  Output buffer (min 385 chars).
 * @param max_len Buffer size.
 */
void ENC_ReadNLCTable(char *buffer, size_t max_len);

/**
 * @brief Clear NLC table, disable NLC_EN, and program EEPROM.
 * @return True if EEPROM programming succeeded.
 */
bool ENC_ClearNLCTable(void);

/**
 * @brief Enable or disable NLC in RAM (not persisted until EEPROM program).
 * @param enabled True to set NLC_EN bit.
 * @return True if readback confirms requested state.
 */
bool ENC_SetNLCEnabled(bool enabled);

/* ---- MT6835 PWM ---- */

/**
 * @brief Measure MT6835 PWM duty cycle by busy-loop timing.
 * @note Blocks ~1-2ms. Result stored for ENC_GetMT6835PWMDutyCyclePercent().
 */
void ENC_MeasureMT6835PWM(void);

/** @brief Return last measured PWM duty cycle (0.0-100.0%). */
float ENC_GetMT6835PWMDutyCyclePercent(void);

/* ---- MT6835 debug ---- */

/**
 * @brief Format SPI debug info (raw bytes, CRC, status).
 * @param debug_str      Output buffer.
 * @param debug_str_size Buffer size.
 */
void ENC_GetMT6835DebugInfo(char *debug_str, size_t debug_str_size);

/**
 * @brief Dump registers 0x001-0x012 as "ADDR=VAL,..." string.
 * @param buffer  Output buffer.
 * @param max_len Buffer size.
 */
void ENC_GetMT6835RegisterDump(char *buffer, size_t max_len);

/* ---- LIR-DA237T optical encoder (BiSS-C via SPI2) ---- */

/**
 * @brief Set LIR BiSS-C resolution. Resets cached position to zero.
 * @param bits 21, 22, or 23. Invalid values default to 23.
 */
void ENC_SetLIRBits(uint8_t bits);

/** @brief Return current LIR resolution in bits. */
uint8_t ENC_GetLIRBits(void);

/**
 * @brief Read LIR position via BiSS-C protocol.
 *
 * Receives 64-bit SPI frame, finds start pattern (0-1-0), extracts
 * position data, verifies CRC6. Returns absolute physical angle.
 *
 * @return Position in counts (0 to 2^bits - 1).
 */
uint32_t ENC_ReadLIRRaw(void);

/** @brief Return last LIR angle in degrees (0.0-360.0). */
float ENC_GetLIRDegrees(void);

/**
 * @brief Format LIR BiSS-C debug info (raw frame, CRC, bit sequence).
 * @param debug_str      Output buffer.
 * @param debug_str_size Buffer size.
 */
void ENC_GetLIRDebugInfo(char *debug_str, size_t debug_str_size);

/** @brief PWM input capture callback placeholder (unused). */
void ENC_MT6835_PWM_IC_Callback(void);

#endif /* ENCODERS_H */