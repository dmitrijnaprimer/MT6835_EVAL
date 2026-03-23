/**
 * @file encoders.h
 * @brief Driver interface for MT6835 magnetic encoder and LIR-DA237T optical
 * encoder.
 *
 * Provides SPI communication with the MT6835 21-bit AMR angle encoder (SPI1)
 * and BiSS-C communication with the LIR-DA237T optical reference encoder
 * (SPI2). Includes angle reading, zero-position management, auto-calibration
 * support, NLC table read/write, and PWM duty cycle measurement.
 */

#ifndef ENCODERS_H
#define ENCODERS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @name MT6835 SPI command opcodes (4-bit, sent as upper nibble of byte 0). */
/**@{*/
#define MT6835_CMD_READ 0x3  /**< Read one register byte. */
#define MT6835_CMD_WRITE 0x6 /**< Write one register byte. */
#define MT6835_CMD_BURST_READ                                                  \
  0xA                            /**< Burst read angle registers 0x003-0x006. */
#define MT6835_CMD_AUTO_ZERO 0x5 /**< Auto-set current position as zero. */
#define MT6835_CMD_PROG_EEPROM 0xC /**< Program register map to EEPROM (~6s).  \
                                    */
/**@}*/

/** @name MT6835 register addresses. */
/**@{*/
#define MT6835_REG_ANGLE_H 0x003   /**< ANGLE[20:13]. */
#define MT6835_REG_ANGLE_M 0x004   /**< ANGLE[12:5]. */
#define MT6835_REG_ANGLE_L 0x005   /**< ANGLE[4:0] | STATUS[2:0]. */
#define MT6835_REG_CRC 0x006       /**< CRC8 over ANGLE + STATUS (24 bits). */
#define MT6835_REG_ABZ_RES_H 0x007 /**< ABZ_RES[13:6]. */
#define MT6835_REG_ABZ_RES_L 0x008 /**< ABZ_RES[5:0] | ABZ_OFF | AB_SWAP. */
#define MT6835_REG_ZERO_H 0x009    /**< ZERO_POS[11:4]. */
#define MT6835_REG_ZERO_L 0x00A /**< ZERO_POS[3:0] | Z_EDGE | Z_PUL_WID[2:0].  \
                                 */
#define MT6835_REG_Z_UVW 0x00B  /**< Z_PHASE, UVW_MUX, UVW_OFF, UVW_RES. */
#define MT6835_REG_PWM_NLC                                                     \
  0x00C /**< NLC_EN (bit5), PWM_FQ, PWM_POL, PWM_SEL. */
#define MT6835_REG_DIR_HYST 0x00D  /**< ROT_DIR, HYST[2:0]. */
#define MT6835_REG_AUTOCAL 0x00E   /**< GPIO_DS, AUTOCAL_FREQ[2:0]. */
#define MT6835_REG_BW 0x011        /**< BW[2:0] system bandwidth. */
#define MT6835_REG_NLC_START 0x013 /**< First NLC table register (192 bytes).  \
                                    */
#define MT6835_REG_NLC_END 0x0D2   /**< Last NLC table register. */
#define MT6835_REG_CAL_STATUS                                                  \
  0x113 /**< Cal status [7:6]: 0=none,1=run,2=fail,3=ok. */
/**@}*/

#define MT6835_NLC_TABLE_SIZE                                                  \
  192 /**< NLC table size in bytes (256 x 6-bit entries). */
#define MT6835_ACK_SUCCESS 0x55 /**< SPI acknowledge for successful commands.  \
                                 */

/** @brief Initialize both encoder interfaces. Call after SPI peripheral init.
 */
void ENC_Init(void);

/** @brief Burst-read the MT6835 21-bit angle with CRC verification. */
uint32_t ENC_ReadMT6835Raw(void);

/** @brief Return the last successfully read 21-bit angle (no SPI transaction).
 */
uint32_t ENC_GetMT6835Raw(void);

/** @brief Return the last read MT6835 angle in degrees (0-360). */
float ENC_GetMT6835Degrees(void);

/** @brief Return true if the last SPI angle read had a CRC mismatch. */
bool ENC_GetMT6835CRCError(void);

/**
 * @brief Write one byte to an MT6835 register.
 * @param address 12-bit register address.
 * @param data    Value to write.
 */
void ENC_WriteMT6835Register(uint16_t address, uint8_t data);

/**
 * @brief Read one byte from an MT6835 register.
 * @param address 12-bit register address.
 * @return Register value.
 */
uint8_t ENC_ReadMT6835Register(uint16_t address);

/**
 * @brief Set current position as MT6835 zero (writes ZERO_POS register).
 * @return True if the chip acknowledged (0x55).
 * @note Changing ZERO_POS invalidates NLC table alignment.
 */
bool ENC_SetMT6835Zero(void);

/**
 * @brief Program the MT6835 register map into EEPROM.
 * @return True if acknowledged. Blocks ~6.5 seconds.
 */
bool ENC_ProgramMT6835EEPROM(void);

/**
 * @brief Read the 12-bit ZERO_POS value from registers 0x009-0x00A.
 * @return 0-4095, representing 0-360 deg in 0.088 deg steps.
 */
uint16_t ENC_GetMT6835ZeroPos(void);

/**
 * @brief Read calibration status register.
 * @return 0=none, 1=running, 2=failed, 3=success.
 */
uint8_t ENC_GetMT6835CalStatus(void);

/**
 * @brief Configure the AUTOCAL_FREQ register for a given rotation speed.
 * @param rpm Calibration speed (25-6400 RPM).
 */
void ENC_ConfigureAutoCalRPM(int rpm);

/**
 * @brief Get the last autocal register write/readback values for debugging.
 * @param written  Value that was written.
 * @param readback Value read back.
 */
void ENC_GetAutoCalDebugInfo(uint8_t *written, uint8_t *readback);

/**
 * @brief Write the 192-byte NLC table and enable NLC_EN bit.
 * @param hex_string 384-character hex string (MSB-first packed, 6-bit 2sC
 * values).
 * @return True on success. Does NOT program EEPROM.
 */
bool ENC_WriteNLCTable(const char *hex_string);

/**
 * @brief Read the NLC table from registers into a hex string.
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
 * @brief Enable or disable NLC in the RAM register (not persisted).
 * @param enabled True to enable NLC_EN bit.
 * @return True if readback confirms the requested state.
 */
bool ENC_SetNLCEnabled(bool enabled);

/**
 * @brief Measure MT6835 PWM duty cycle using busy-loop timing.
 * @note Blocks for ~1-2ms (one PWM period). Call only when needed.
 */
void ENC_MeasureMT6835PWM(void);

/** @brief Return the last measured PWM duty cycle (0-100%). */
float ENC_GetMT6835PWMDutyCyclePercent(void);

/**
 * @brief Format MT6835 SPI debug info (raw bytes, CRC, status).
 * @param debug_str      Output buffer.
 * @param debug_str_size Buffer size.
 */
void ENC_GetMT6835DebugInfo(char *debug_str, size_t debug_str_size);

/**
 * @brief Dump config registers 0x001-0x012 as "ADDR=VAL,..." string.
 * @param buffer  Output buffer.
 * @param max_len Buffer size.
 */
void ENC_GetMT6835RegisterDump(char *buffer, size_t max_len);

/**
 * @brief Set the LIR-DA237T BiSS-C resolution and reset software zero.
 * @param bits 21, 22, or 23.
 */
void ENC_SetLIRBits(uint8_t bits);

/** @brief Return the current LIR bit resolution. */
uint8_t ENC_GetLIRBits(void);

/** @brief Set the current LIR position as software zero reference. */
void ENC_SetLIRZero(void);

/** @brief Configure LIR direction inversion. */
void ENC_SetLIRInvertDirection(bool invert);

/** @brief Return the LIR direction inversion state. */
bool ENC_GetLIRInvertDirection(void);

/** @brief Read the LIR position via BiSS-C (SPI2), apply software zero offset.
 */
uint32_t ENC_ReadLIRRaw(void);

/** @brief Return the last read LIR angle in degrees. */
float ENC_GetLIRDegrees(void);

/**
 * @brief Format LIR BiSS-C debug info (raw frame, CRC, bit sequence).
 * @param debug_str      Output buffer.
 * @param debug_str_size Buffer size.
 */
void ENC_GetLIRDebugInfo(char *debug_str, size_t debug_str_size);

/** @brief Input capture callback placeholder for MT6835 PWM (unused). */
void ENC_MT6835_PWM_IC_Callback(void);

#endif /* ENCODERS_H */