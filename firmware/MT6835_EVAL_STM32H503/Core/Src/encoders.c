/**
 * @file encoders.c
 * @brief Driver for MT6835 (SPI1) and LIR-DA237T (SPI2, BiSS-C).
 */

#include "encoders.h"
#include "main.h"
#include "stm32h5xx_hal.h"
#include "string.h"
#include <stdio.h>

extern SPI_HandleTypeDef hspi1;
extern SPI_HandleTypeDef hspi2;

#define MT6835_CRC8_POLY 0x07

#define LIR_NUM_BITS_DEFAULT 23
#define LIR_MIN_BITS 21
#define LIR_MAX_BITS 23

/* ---- LIR state ---- */
static volatile uint32_t lir_raw_cached = 0;
static volatile float lir_degrees_cached = 0.0f;
static uint8_t lir_num_bits = LIR_NUM_BITS_DEFAULT;

/* ---- MT6835 state ---- */
static volatile uint32_t mt6835_raw_cached = 0;
static volatile uint32_t mt6835_raw_physical = 0;
static volatile float mt6835_degrees_cached = 0.0f;
static float mt6835_pwm_duty_percent = 0.0f;
static uint8_t mt6835_status_reg = 0;
static bool mt6835_crc_error = false;

/* ---- Debug ---- */
static uint8_t mt6835_debug_rx_buf[6] = {0};
static uint8_t mt6835_debug_calc_crc = 0;
static uint8_t mt6835_autocal_written = 0;
static uint8_t mt6835_autocal_readback = 0;

static uint64_t debug_spi_combined_word = 0;
static uint8_t debug_start_found_at = 0xFF;
static uint32_t debug_extracted_data = 0;
static uint8_t debug_calculated_crc = 0;
static uint8_t debug_received_crc = 0;
static uint8_t debug_crc_ok = 0;
static char debug_raw_sequence_str[66];

/* ================================================================== */
/* Helpers                                                            */
/* ================================================================== */

/** @brief Convert hex character to 4-bit value. */
static uint8_t hex_char_to_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return 0;
}

/** @brief Convert two hex characters to a byte. */
static uint8_t hex_to_byte(const char *hex) {
  return (hex_char_to_nibble(hex[0]) << 4) | hex_char_to_nibble(hex[1]);
}

/** @brief CRC8 for MT6835 (poly X^8+X^2+X+1, init 0x00). */
static uint8_t calculate_crc8_mt6835(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (int i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ MT6835_CRC8_POLY;
      else crc <<= 1;
    }
  }
  return crc;
}

/** @brief CRC6 for BiSS-C protocol. */
static uint8_t calculate_crc6_bissc(uint64_t frame_word, uint8_t data_msb_pos,
                                    uint8_t num_data_bits) {
  uint8_t crc = 0x00;
  int total_bits = num_data_bits + 2;
  for (int i = 0; i < total_bits; i++) {
    int bit_pos = data_msb_pos - i;
    if (bit_pos < 0) break;
    uint8_t current_bit = (frame_word >> bit_pos) & 0x01;
    uint8_t msb = (crc >> 5) & 0x01;
    crc <<= 1;
    if (msb ^ current_bit) crc ^= 0x03;
    crc &= 0x3F;
  }
  return crc;
}

/* ================================================================== */
/* MT6835 PWM measurement                                             */
/* ================================================================== */

/** @brief Measure PWM duty cycle by busy-loop. Blocks ~1-2ms. */
void ENC_MeasureMT6835PWM(void) {
  uint32_t high_count = 0, low_count = 0, timeout;

  timeout = 100000;
  while (HAL_GPIO_ReadPin(MT6835_PWM_GPIO_Port, MT6835_PWM_Pin) == GPIO_PIN_SET && timeout--);
  if (timeout == 0) { mt6835_pwm_duty_percent = 100.0f; return; }

  timeout = 100000;
  while (HAL_GPIO_ReadPin(MT6835_PWM_GPIO_Port, MT6835_PWM_Pin) == GPIO_PIN_RESET && timeout--);
  if (timeout == 0) { mt6835_pwm_duty_percent = 0.0f; return; }

  timeout = 2000000;
  while (HAL_GPIO_ReadPin(MT6835_PWM_GPIO_Port, MT6835_PWM_Pin) == GPIO_PIN_SET && timeout--) high_count++;

  timeout = 2000000;
  while (HAL_GPIO_ReadPin(MT6835_PWM_GPIO_Port, MT6835_PWM_Pin) == GPIO_PIN_RESET && timeout--) low_count++;

  uint32_t total = high_count + low_count;
  mt6835_pwm_duty_percent = (total > 0) ? ((float)high_count / (float)total) * 100.0f : 0.0f;
}

/* ================================================================== */
/* MT6835 SPI register access                                         */
/* ================================================================== */

/** @brief Read one register byte via SPI1. */
uint8_t ENC_ReadMT6835Register(uint16_t address) {
  uint8_t tx_buf[3], rx_buf[3] = {0};
  tx_buf[0] = (MT6835_CMD_READ << 4) | ((address >> 8) & 0x0F);
  tx_buf[1] = address & 0xFF;
  tx_buf[2] = 0x00;
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 3, 10);
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_SET);
  return rx_buf[2];
}

/** @brief Write one register byte via SPI1. */
void ENC_WriteMT6835Register(uint16_t address, uint8_t data) {
  uint8_t tx_buf[3];
  tx_buf[0] = (MT6835_CMD_WRITE << 4) | ((address >> 8) & 0x0F);
  tx_buf[1] = address & 0xFF;
  tx_buf[2] = data;
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi1, tx_buf, 3, 10);
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_SET);
  for (volatile int i = 0; i < 100; i++);
}

/* ================================================================== */
/* MT6835 angle reading                                               */
/* ================================================================== */

/**
 * @brief Burst-read 21-bit angle with CRC8 check.
 *
 * Reads registers 0x003-0x006. On CRC failure the cached value is unchanged.
 * The returned angle has ZERO_POS already subtracted by the chip internally.
 */
uint32_t ENC_ReadMT6835Raw(void) {
  uint8_t tx_buf[6] = {0}, rx_buf[6] = {0};
  tx_buf[0] = (MT6835_CMD_BURST_READ << 4) | ((MT6835_REG_ANGLE_H >> 8) & 0x0F);
  tx_buf[1] = MT6835_REG_ANGLE_H & 0xFF;

  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 6, 10);
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_SET);

  memcpy(mt6835_debug_rx_buf, rx_buf, 6);

  uint32_t angle = ((uint32_t)rx_buf[2] << 13) |
                   ((uint32_t)rx_buf[3] << 5) |
                   ((uint32_t)(rx_buf[4] >> 3));
  mt6835_status_reg = rx_buf[4] & 0x07;

  uint8_t crc_data[3] = {rx_buf[2], rx_buf[3], rx_buf[4]};
  uint8_t calc_crc = calculate_crc8_mt6835(crc_data, 3);
  mt6835_debug_calc_crc = calc_crc;

  if (calc_crc == rx_buf[5]) {
    mt6835_crc_error = false;
    mt6835_raw_physical = angle;
    mt6835_raw_cached = angle;
    mt6835_degrees_cached = ((float)mt6835_raw_cached / 2097152.0f) * 360.0f;
  } else {
    mt6835_crc_error = true;
  }
  return mt6835_raw_cached;
}

/** @brief Read the 12-bit ZERO_POS register. */
uint16_t ENC_GetMT6835ZeroPos(void) {
  uint8_t high = ENC_ReadMT6835Register(MT6835_REG_ZERO_H);
  uint8_t low = ENC_ReadMT6835Register(MT6835_REG_ZERO_L);
  return ((uint16_t)high << 4) | ((low >> 4) & 0x0F);
}

/* ================================================================== */
/* MT6835 commands with ACK                                           */
/* ================================================================== */

/** @brief Set current position as zero. Re-reads angle afterward. */
bool ENC_SetMT6835Zero(void) {
  uint8_t tx_buf[3] = {0}, rx_buf[3] = {0};
  tx_buf[0] = (MT6835_CMD_AUTO_ZERO << 4);
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 3, 10);
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_SET);
  HAL_Delay(2);
  bool ok = (rx_buf[2] == MT6835_ACK_SUCCESS);
  ENC_ReadMT6835Raw();
  return ok;
}

/** @brief Program all registers to EEPROM. Blocks ~6.5s on success. */
bool ENC_ProgramMT6835EEPROM(void) {
  uint8_t tx_buf[3] = {0}, rx_buf[3] = {0};
  tx_buf[0] = (MT6835_CMD_PROG_EEPROM << 4);
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 3, 10);
  HAL_GPIO_WritePin(MT6835_CSN_GPIO_Port, MT6835_CSN_Pin, GPIO_PIN_SET);
  bool ok = (rx_buf[2] == MT6835_ACK_SUCCESS);
  if (ok) HAL_Delay(6500);
  return ok;
}

/* ================================================================== */
/* MT6835 auto-calibration                                            */
/* ================================================================== */

uint8_t ENC_GetMT6835CalStatus(void) {
  return (ENC_ReadMT6835Register(MT6835_REG_CAL_STATUS) >> 6) & 0x03;
}

/** @brief Map RPM to AUTOCAL_FREQ and write register 0x00E. */
void ENC_ConfigureAutoCalRPM(int rpm) {
  uint8_t freq_val;
  if (rpm >= 3200)      freq_val = 0x0;
  else if (rpm >= 1600) freq_val = 0x1;
  else if (rpm >= 800)  freq_val = 0x2;
  else if (rpm >= 400)  freq_val = 0x3;
  else if (rpm >= 200)  freq_val = 0x4;
  else if (rpm >= 100)  freq_val = 0x5;
  else if (rpm >= 50)   freq_val = 0x6;
  else                  freq_val = 0x7;

  uint8_t reg = ENC_ReadMT6835Register(MT6835_REG_AUTOCAL);
  uint8_t new_val = (reg & 0x8F) | ((freq_val & 0x07) << 4);
  ENC_WriteMT6835Register(MT6835_REG_AUTOCAL, new_val);
  mt6835_autocal_written = new_val;
  HAL_Delay(2);
  mt6835_autocal_readback = ENC_ReadMT6835Register(MT6835_REG_AUTOCAL);
}

void ENC_GetAutoCalDebugInfo(uint8_t *written, uint8_t *readback) {
  *written = mt6835_autocal_written;
  *readback = mt6835_autocal_readback;
}

/* ================================================================== */
/* MT6835 NLC table                                                   */
/* ================================================================== */

/** @brief Write 192-byte NLC table from hex string and enable NLC_EN. */
bool ENC_WriteNLCTable(const char *hex_string) {
  if (!hex_string || strlen(hex_string) < MT6835_NLC_TABLE_SIZE * 2)
    return false;

  for (int i = 0; i < MT6835_NLC_TABLE_SIZE; i++) {
    ENC_WriteMT6835Register(MT6835_REG_NLC_START + i, hex_to_byte(&hex_string[i * 2]));
    for (volatile int k = 0; k < 50; k++);
  }

  uint8_t reg = ENC_ReadMT6835Register(MT6835_REG_PWM_NLC);
  ENC_WriteMT6835Register(MT6835_REG_PWM_NLC, reg | 0x20);
  return true;
}

/** @brief Read NLC table into uppercase hex string. */
void ENC_ReadNLCTable(char *buffer, size_t max_len) {
  if (!buffer || max_len < (MT6835_NLC_TABLE_SIZE * 2 + 1)) return;
  buffer[0] = '\0';
  char temp[4];
  for (int i = 0; i < MT6835_NLC_TABLE_SIZE; i++) {
    snprintf(temp, sizeof(temp), "%02X", ENC_ReadMT6835Register(MT6835_REG_NLC_START + i));
    strcat(buffer, temp);
  }
}

/** @brief Clear NLC, disable NLC_EN, program EEPROM. */
bool ENC_ClearNLCTable(void) {
  for (int i = 0; i < MT6835_NLC_TABLE_SIZE; i++) {
    ENC_WriteMT6835Register(MT6835_REG_NLC_START + i, 0x00);
    for (volatile int k = 0; k < 50; k++);
  }
  uint8_t reg = ENC_ReadMT6835Register(MT6835_REG_PWM_NLC);
  ENC_WriteMT6835Register(MT6835_REG_PWM_NLC, reg & ~0x20);
  return ENC_ProgramMT6835EEPROM();
}

/** @brief Set NLC_EN bit in RAM. Verify by readback. */
bool ENC_SetNLCEnabled(bool enabled) {
  uint8_t reg = ENC_ReadMT6835Register(MT6835_REG_PWM_NLC);
  if (enabled) reg |= 0x20; else reg &= ~0x20;
  ENC_WriteMT6835Register(MT6835_REG_PWM_NLC, reg);
  uint8_t rb = ENC_ReadMT6835Register(MT6835_REG_PWM_NLC);
  return ((rb & 0x20) != 0) == enabled;
}

/* ================================================================== */
/* MT6835 getters                                                     */
/* ================================================================== */

uint32_t ENC_GetMT6835Raw(void) { return mt6835_raw_cached; }
float ENC_GetMT6835Degrees(void) { return mt6835_degrees_cached; }
float ENC_GetMT6835PWMDutyCyclePercent(void) { return mt6835_pwm_duty_percent; }
bool ENC_GetMT6835CRCError(void) { return mt6835_crc_error; }
void ENC_MT6835_PWM_IC_Callback(void) {}

/* ================================================================== */
/* MT6835 debug                                                       */
/* ================================================================== */

/** @brief Format last burst-read raw bytes and CRC status. */
void ENC_GetMT6835DebugInfo(char *debug_str, size_t debug_str_size) {
  snprintf(debug_str, debug_str_size,
      "MT_SPI:[%02X %02X %02X %02X %02X %02X] CRC_C:%02X CRC_ERR:%d STAT:%d",
      mt6835_debug_rx_buf[0], mt6835_debug_rx_buf[1], mt6835_debug_rx_buf[2],
      mt6835_debug_rx_buf[3], mt6835_debug_rx_buf[4], mt6835_debug_rx_buf[5],
      mt6835_debug_calc_crc, mt6835_crc_error ? 1 : 0, mt6835_status_reg);
}

/** @brief Dump registers 0x001-0x012 as comma-separated ADDR=VAL pairs. */
void ENC_GetMT6835RegisterDump(char *buffer, size_t max_len) {
  if (!buffer || max_len == 0) return;
  buffer[0] = '\0';
  char temp[16];
  size_t pos = 0;
  for (uint16_t addr = 0x001; addr <= 0x012; addr++) {
    int w = snprintf(temp, sizeof(temp), "%03X=%02X,", addr, ENC_ReadMT6835Register(addr));
    if (pos + w < max_len - 1) { strcat(buffer, temp); pos += w; }
    else break;
  }
  if (pos > 0 && buffer[pos - 1] == ',') buffer[pos - 1] = '\0';
}

/* ================================================================== */
/* LIR-DA237T (BiSS-C via SPI2)                                      */
/* ================================================================== */

/** @brief Set resolution and reset cached position. */
void ENC_SetLIRBits(uint8_t bits) {
  lir_num_bits = (bits >= LIR_MIN_BITS && bits <= LIR_MAX_BITS) ? bits : LIR_NUM_BITS_DEFAULT;
  lir_raw_cached = 0;
  lir_degrees_cached = 0.0f;
}

uint8_t ENC_GetLIRBits(void) { return lir_num_bits; }
float ENC_GetLIRDegrees(void) { return lir_degrees_cached; }

/**
 * @brief Read LIR absolute physical angle via BiSS-C.
 *
 * Receives 64-bit frame, finds start pattern (0->1->0), extracts position,
 * verifies CRC6 (direct or inverted match). Returns absolute physical counts.
 */
uint32_t ENC_ReadLIRRaw(void) {
  uint8_t nbits = lir_num_bits;

  debug_start_found_at = 0xFF;
  debug_extracted_data = 0;
  debug_calculated_crc = 0;
  debug_received_crc = 0;
  debug_crc_ok = 0;
  debug_spi_combined_word = 0;
  memset(debug_raw_sequence_str, 0, sizeof(debug_raw_sequence_str));

  uint32_t rx_buffer[2] = {0, 0};
  HAL_StatusTypeDef result = HAL_SPI_Receive(&hspi2, (uint8_t *)rx_buffer, 2, HAL_MAX_DELAY);

  if (result == HAL_OK) {
    uint64_t frame = ((uint64_t)rx_buffer[0] << 32) | rx_buffer[1];
    debug_spi_combined_word = frame;

    for (int i = 63; i >= 0; i--) {
      int ci = 63 - i;
      if (ci < 65) debug_raw_sequence_str[ci] = ((frame >> i) & 1) ? '1' : '0';
    }
    debug_raw_sequence_str[64] = '\0';

    int min_start = nbits + 9;
    for (int sb = 62; sb >= min_start; sb--) {
      uint8_t pre  = (frame >> (sb + 1)) & 1;
      uint8_t strt = (frame >> sb) & 1;
      uint8_t cds  = (frame >> (sb - 1)) & 1;

      if (pre == 0 && strt == 1 && cds == 0) {
        int dmsb = sb - 2;
        int dlsb = dmsb - (nbits - 1);
        uint32_t data = (uint32_t)((frame >> dlsb) & ((1ULL << nbits) - 1));
        uint8_t rx_crc = (uint8_t)((frame >> (dlsb - 8)) & 0x3F);

        debug_start_found_at = sb;
        debug_extracted_data = data;
        debug_received_crc = rx_crc;

        uint8_t calc = calculate_crc6_bissc(frame, dmsb, nbits);
        debug_calculated_crc = calc;

        if (calc == (uint8_t)(~rx_crc & 0x3F))
          debug_crc_ok = 2;
        else if (calc == rx_crc && (data != 0 || calc != 0))
          debug_crc_ok = 1;

        if (debug_crc_ok > 0)
          lir_raw_cached = data;
        break;
      }
    }
  }

  lir_degrees_cached = (float)((double)lir_raw_cached / (double)(1ULL << nbits) * 360.0);
  return lir_raw_cached;
}

/** @brief Format BiSS-C debug info. */
void ENC_GetLIRDebugInfo(char *debug_str, size_t debug_str_size) {
  uint32_t wh = (uint32_t)(debug_spi_combined_word >> 32);
  uint32_t wl = (uint32_t)(debug_spi_combined_word & 0xFFFFFFFF);
  snprintf(debug_str, debug_str_size,
      "SPI:0x%08lX%08lX ST:%d RAW:0x%lX CRC_C:0x%02X CRC_R:0x%02X OK:%d BITS:%d\nSEQ:%s",
      (unsigned long)wh, (unsigned long)wl, debug_start_found_at,
      (unsigned long)debug_extracted_data, debug_calculated_crc,
      debug_received_crc, debug_crc_ok, lir_num_bits, debug_raw_sequence_str);
}

/* ================================================================== */
/* Initialization                                                     */
/* ================================================================== */

/** @brief Initialize both encoders. Wait for MT6835 power-up (64ms spec). */
void ENC_Init(void) {
  HAL_Delay(100);
  lir_num_bits = LIR_NUM_BITS_DEFAULT;
  lir_raw_cached = 0;
  lir_degrees_cached = 0.0f;
  mt6835_raw_cached = 0;
  mt6835_raw_physical = 0;
  mt6835_degrees_cached = 0.0f;
  mt6835_pwm_duty_percent = 0.0f;
  mt6835_crc_error = false;
  ENC_ReadMT6835Raw();
}