# I2C Protocol Library

A flexible and universal I2C communication protocol library for STM32 controllers.

## Files

- `i2c_protocol.h` - Header file with all type definitions, function prototypes, and configuration constants
- `i2c_protocol.c` - Implementation file with all functions

## Usage

### 1. Include the library in your STM32 project

Copy both files to your project and include the header:

```c
#include "i2c_protocol.h"
```

### 2. Quick Setup Options

#### Option A: Use existing I2C handle (recommended if you already have I2C configured)

```c
I2C_HandleTypeDef hi2c1;  // Your existing STM32 I2C handle
I2C_Handle_t i2c_handle;

// Initialize with your existing handle
I2C_InitWithHandle(&hi2c1, &i2c_handle, 0x02, I2C_ROLE_SLAVE);
//                 handle   i2c_handle  device_id   role
```

#### Option B: Auto-configure from GPIO pins (library handles I2C setup)

```c
I2C_Handle_t i2c_handle;

// Initialize with GPIO pins - library configures I2C for you
I2C_InitWithGPIO(&i2c_handle, 0x02, I2C_ROLE_SLAVE,
                I2C1, GPIOB, GPIO_PIN_7,  // I2C instance, SDA port, SDA pin
                GPIOB, GPIO_PIN_6, 0x42);  // SCL port, SCL pin, own address
```

### 3. Full Custom Configuration

For complete control over all parameters:

```c
I2C_Config_t config = {
    .hi2c = &hi2c1,               // Use existing handle, OR
    .hw_config = &my_hw_config,   // Hardware config for auto-init
    .gpio_config = &my_gpio_config, // GPIO config for auto-init
    .device_id = 0x02,            // Your device ID (1-239)
    .role = I2C_ROLE_SLAVE,
    .timeout_ms = 1000,
    .enable_interrupts = true,
    .auto_init_hardware = false   // Set true to auto-configure I2C
};

I2C_Handle_t i2c_handle;
I2C_Init(&i2c_handle, &config);
```

### 4. Define Your Own Device IDs and Commands

The library doesn't restrict device IDs or commands - you define them in your project:

```c
// Define your own device IDs (1-239 are available)
#define MY_ARM_CONTROLLER_ID        0x10
#define MY_TORPEDO_CONTROLLER_ID    0x11
#define MY_MAIN_COMPUTER_ID         0x01

// Define your own commands (0x0000-0xFEFF are available)
#define CMD_SET_MOTOR_SPEED         0x1001
#define CMD_GET_STATUS             0x1002
#define CMD_EMERGENCY_STOP         0x1003
#define CMD_SET_SERVO_ANGLE        0x1004
```

### 5. Sending Messages

```c
I2C_Message_t message;
uint8_t motor_data[] = {50, 75, 100}; // Example motor speeds

// Create message with your custom IDs and commands
I2C_CreateMessage(&message, 
                 MY_ARM_CONTROLLER_ID,      // Source
                 MY_TORPEDO_CONTROLLER_ID,  // Target
                 CMD_SET_MOTOR_SPEED,       // Your custom command
                 motor_data,                // Payload
                 sizeof(motor_data));       // Payload length

// Send message (master mode)
I2C_SendMessage(&i2c_handle, &message, 0x11);  // Target I2C address
```

### 6. Receiving Messages

```c
I2C_Message_t received_message;

// Check if message is ready (non-blocking)
if (I2C_MessageReady(&i2c_handle)) {
    I2C_Status_t status = I2C_ReceiveMessage(&i2c_handle, &received_message, 1000);
    if (status == I2C_STATUS_OK) {
        // Process received message with your custom commands
        switch (received_message.command) {
            case CMD_SET_MOTOR_SPEED:
                // Handle motor control
                break;
            case CMD_GET_STATUS:
                // Handle status request
                break;
        }
    }
}
```

### 7. Interrupt Integration

Add these to your STM32 HAL I2C callbacks:

```c
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        I2C_RxCompleteCallback(&i2c_handle);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        I2C_TxCompleteCallback(&i2c_handle);
    }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == &hi2c1) {
        I2C_ErrorCallback(&i2c_handle);
    }
}
```

## Message Format

All messages follow this format:
```
[src_id][target_id][command_high][command_low][payload_length][payload...][0xFF]
```

- `src_id`: Source device ID (1 byte)
- `target_id`: Target device ID (1 byte) 
- `command`: Command or data type (2 bytes, big-endian)
- `payload_length`: Length of payload data (1 byte)
- `payload`: Variable length data (0-64 bytes)
- `0xFF`: End of message marker (1 byte)

## Flexible ID and Command System

### Device IDs
- **Available**: 0x01 - 0xEF (1-239) - Use any you want!
- **Reserved**: 0xF0 - 0xFE (240-254) - Library internal use
- **Broadcast**: 0xFF (255) - Send to all devices

### Commands
- **Available**: 0x0000 - 0xFEFF (0-65279) - Define your own!
- **Reserved**: 0xFF00 - 0xFFFF (65280-65535) - Library internal use

## Project-Specific Configuration Examples

Each project can define their own configuration header:

```c
// torpedo_controller_config.h
#define TORPEDO_DEVICE_ID           0x03
#define TORPEDO_I2C_ADDRESS         0x43

#define CMD_SET_THRUST             0x1001
#define CMD_SET_RUDDER_ANGLE       0x1002
#define CMD_FIRE_TORPEDO           0x1003
```

```c
// arm_controller_config.h  
#define ARM_DEVICE_ID              0x02
#define ARM_I2C_ADDRESS            0x42

#define CMD_MOVE_JOINT             0x2001
#define CMD_GRAB_OBJECT            0x2002
#define CMD_GET_JOINT_POSITION     0x2003
```

## Error Handling

```c
I2C_Status_t status = I2C_SendMessage(&i2c_handle, &message, target_addr);
if (status != I2C_STATUS_OK) {
    // Handle error
    I2C_Statistics_t stats = I2C_GetStatistics(&i2c_handle);
    printf("Errors: %lu, Timeouts: %lu\n", stats.errors, stats.timeouts);
}
```