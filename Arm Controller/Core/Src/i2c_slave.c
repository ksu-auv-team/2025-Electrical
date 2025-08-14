/*
 * i2c_slave.c
 */
#include "i2c_slave.h"
#include "main.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c2;
extern TIM_HandleTypeDef htim2;

// ----- Servo constraints (match main.c) -----
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000

static inline uint16_t clamp_u16(int v, int lo, int hi) {
  return (uint16_t)((v < lo) ? lo : (v > hi ? hi : v));
}
static inline uint16_t angle_to_us(uint8_t deg) {
  if (deg > 180) deg = 180;
  return (uint16_t)(SERVO_MIN_US + ((uint32_t)(SERVO_MAX_US - SERVO_MIN_US) * deg) / 180U);
}
static inline void servo_write_us(uint16_t us) {
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, clamp_u16(us, SERVO_MIN_US, SERVO_MAX_US));
}

// ---- Simple register window the master can write/read ----
// R0: angle (0-180). R1: reserved. R2..R3: pulse width (LSB, MSB)
#define I2C_REG_COUNT  32
static uint8_t I2C_REGISTERS[I2C_REG_COUNT] = {0};

// ---- RX/TX scratch ----
static uint8_t i2c_rx[32];
static uint8_t i2c_tx[32];

static volatile uint8_t rx_count    = 0;  // number of RX chunks completed
static volatile uint8_t rx_expected = 0;  // payload length after [reg][len]
static volatile uint8_t have_new_cmd = 0;

static inline void i2c_listen(I2C_HandleTypeDef *hi2c) {
  HAL_I2C_EnableListen_IT(hi2c);
}

// ----------------------- Foreground processor -----------------------
// Call this once per loop from main()
void process_i2c_cmd(void)
{
  if (!have_new_cmd) return;

  // Snapshot message atomically
  uint8_t reg, len;
  uint8_t data[32];
  __disable_irq();
  have_new_cmd = 0;
  reg = i2c_rx[0];
  len = (rx_count >= 2) ? i2c_rx[1] : 0;
  if (len > sizeof(data)) len = sizeof(data);
  memcpy(data, &i2c_rx[2], len);
  __enable_irq();

  // Apply to register window
  for (uint8_t i = 0; i < len; i++) {
    uint8_t idx = (uint8_t)(reg + i);
    if (idx < I2C_REG_COUNT) I2C_REGISTERS[idx] = data[i];
  }

  // ---- Act on registers ----
  if (reg == 0 && len >= 1) {
    // R0: angle in degrees
    uint8_t deg = I2C_REGISTERS[0];
    servo_write_us(angle_to_us(deg));
  } else if (reg <= 2 && (reg + len) >= 4) {
    // R2..R3: pulse width in microseconds (LSB, MSB)
    uint16_t us = (uint16_t)I2C_REGISTERS[2] | ((uint16_t)I2C_REGISTERS[3] << 8);
    servo_write_us(us);
  }

  // Debug print
  printf("I2C RX: reg=0x%02X len=%u data:", reg, (unsigned)len);
  for (uint8_t i = 0; i < len; i++) printf(" %02X", data[i]);
  printf("\r\n");
}

// ----------------------- I2C IRQ callbacks -------------------------
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                          uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
  (void)AddrMatchCode;

  if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
    // Master -> Slave (WRITE): expect [reg][len][payload]
    rx_count = 0;
    rx_expected = 0;
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[0], 1, I2C_NEXT_FRAME); // reg
  } else {
    // Master <- Slave (READ): return small status window
    // Status bytes: angle (R0), reserved (R1), pulse LSB (R2), pulse MSB (R3)
    // Ensure R2..R3 reflect current CCR timebase (µs)
    uint16_t ccr = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_1);
    I2C_REGISTERS[2] = (uint8_t)(ccr & 0xFF);
    I2C_REGISTERS[3] = (uint8_t)(ccr >> 8);

    i2c_tx[0] = I2C_REGISTERS[0];
    i2c_tx[1] = I2C_REGISTERS[1];
    i2c_tx[2] = I2C_REGISTERS[2];
    i2c_tx[3] = I2C_REGISTERS[3];
    HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx, 4, I2C_LAST_FRAME);
  }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
  rx_count++;

  if (rx_count == 1) {
    // got [reg]; now [len]
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[1], 1, I2C_NEXT_FRAME);
    return;
  }

  if (rx_count == 2) {
    // got [reg][len]; now payload
    rx_expected = i2c_rx[1];
    if (rx_expected > (sizeof(i2c_rx) - 2)) rx_expected = sizeof(i2c_rx) - 2;

    if (rx_expected == 0) { have_new_cmd = 1; return; }
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[2], rx_expected, I2C_LAST_FRAME);
    return;
  }

  // payload finished
  have_new_cmd = 1;
  i2c_listen(hi2c);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c) { i2c_listen(hi2c); }
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)  { i2c_listen(hi2c); }
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)       { (void)HAL_I2C_GetError(hi2c); i2c_listen(hi2c); }
