#include "i2c_slave.h"
#include "main.h"
#include <string.h>
#include <stdio.h>   // for printf (CDC/UART/ITM retarget)

#define I2C_REG_COUNT 10

static uint8_t i2c_rx[32];
static uint8_t i2c_tx[32];

static volatile uint8_t rx_stage = 0;     // 0=expect reg, 1=expect len, 2=expect payload
static volatile uint8_t rx_len   = 0;     // payload length
static volatile uint8_t have_new_cmd = 0;

extern I2C_HandleTypeDef hi2c2;

// Exported register window used by the app logic
uint8_t I2C_REGISTERS[I2C_REG_COUNT] = {0};

static inline void i2c_listen(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c);
}

// ---------------- Application-side action (no recursion) ----------------
extern TIM_HandleTypeDef htim2;
extern unsigned char tim1Degrees;
extern unsigned char direction;
extern float tim1PWM;
extern float pulseWidth;

static void process_data(void)
{
    // Map registers -> servo/logic
    int angle = I2C_REGISTERS[0];
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    direction = I2C_REGISTERS[1];

    pulseWidth = 500.0f + ((float)angle * 2000.0f) / 180.0f;
    tim1PWM = pulseWidth;

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, (uint32_t)tim1PWM);

    tim1Degrees = (unsigned char)angle;
}

// -------------------------- I2C slave callbacks --------------------------
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c,
                          uint8_t TransferDirection,
                          uint16_t AddrMatchCode)
{
    (void)AddrMatchCode;

    if (TransferDirection == I2C_DIRECTION_TRANSMIT) {
        // Master -> Slave (WRITE): expect [reg] first
        rx_stage = 0;
        rx_len   = 0;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[0], 1, I2C_NEXT_FRAME);
    } else {
        // Master <- Slave (READ): return a small window
        i2c_tx[0] = I2C_REGISTERS[0];
        i2c_tx[1] = I2C_REGISTERS[1];
        i2c_tx[2] = I2C_REGISTERS[2];
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, i2c_tx, 3, I2C_LAST_FRAME);
    }
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (rx_stage == 0) {
        rx_stage = 1;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[1], 1, I2C_NEXT_FRAME);
        return;
    }

    if (rx_stage == 1) {
        rx_len = i2c_rx[1];
        if (rx_len > (sizeof(i2c_rx) - 2)) rx_len = sizeof(i2c_rx) - 2;

        rx_stage = 2;
        if (rx_len == 0) {
            have_new_cmd = 1;
            return;
        }
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &i2c_rx[2], rx_len, I2C_LAST_FRAME);
        return;
    }

    have_new_cmd = 1;
    i2c_listen(hi2c);
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_listen(hi2c);
}

void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
    i2c_listen(hi2c);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    (void)HAL_I2C_GetError(hi2c);
    i2c_listen(hi2c);
}

// -------------------------- Foreground processor --------------------------
void process_i2c_cmd(void)
{
    if (!have_new_cmd) return;

    // Snapshot the message safely
    uint8_t reg, len;
    uint8_t data[32];

    __disable_irq();
    have_new_cmd = 0;
    reg = i2c_rx[0];
    len = rx_len;
    if (len > sizeof(data)) len = sizeof(data);
    memcpy(data, &i2c_rx[2], len);
    __enable_irq();

    // Apply into our register window
    for (uint8_t i = 0; i < len; i++) {
        uint8_t idx = (uint8_t)(reg + i);
        if (idx < I2C_REG_COUNT) {
            I2C_REGISTERS[idx] = data[i];
        }
    }

    // Debug print of the exact message
    printf("I2C RX: reg=0x%02X len=%u data:", reg, (unsigned)len);
    for (uint8_t i = 0; i < len; i++) {
        printf(" %02X", data[i]);
    }
    printf("\r\n");

    // Call process data to send servo cmds
    process_data();
}
