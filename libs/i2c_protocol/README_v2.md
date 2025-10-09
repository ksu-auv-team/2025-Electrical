# I2C Protocol Library v2.0 - Message Handling Only

A simplified I2C communication protocol library that focuses purely on message formatting, parsing, and validation. Hardware setup (pins, I2C configuration) is handled by individual projects.

## Key Changes from v1.0

**Removed Hardware Dependencies**
- No STM32 HAL dependencies
- No GPIO configuration
- No I2C peripheral setup
- Projects handle their own hardware setup

**Focused on Message Processing**
- Message creation and validation
- Serialization for transmission
- Parsing of received data
- Checksum calculation and validation

**Simplified Integration**
- Just include the library files
- Call functions to process messages
- No complex initialization required

## Files

- `i2c_protocol_v2.h` - Header with all functions and types
- `i2c_protocol_v2.c` - Implementation  
- `README_v2.md` - This documentation

## Message Format

```
[src_id][target_id][cmd_high][cmd_low][payload_len][payload...][0xFF]
```

- **src_id**: Source device ID (1 byte)
- **target_id**: Target device ID (1 byte)  
- **command**: Command type (2 bytes, big-endian)
- **payload_length**: Payload size (1 byte, 0-64)
- **payload**: Data bytes (0-64 bytes)
- **0xFF**: End marker (1 byte)

## Usage Examples

### 1. Creating a Message

```c
#include "i2c_protocol_v2.h"

I2C_Protocol_Message_t message;
uint8_t motor_data[] = {50, 75, 100};

// Create message
I2C_Protocol_Status_t status = I2C_Protocol_CreateMessage(
    &message,
    0x01,                    // Source: our device ID
    0x02,                    // Target: motor controller  
    0x1001,                  // Command: set motor speeds
    motor_data,              // Payload
    sizeof(motor_data)       // Payload length
);

if (status == I2C_PROTOCOL_OK) {
    // Message created successfully
}
```

### 2. Serializing for Transmission

```c
uint8_t tx_buffer[32];
uint16_t bytes_to_send;

// Convert message to bytes for I2C transmission
status = I2C_Protocol_SerializeMessage(&message, tx_buffer, sizeof(tx_buffer), &bytes_to_send);

if (status == I2C_PROTOCOL_OK) {
    // Send tx_buffer[0..bytes_to_send-1] via your I2C hardware
    // Example with STM32 HAL:
    HAL_I2C_Master_Transmit(&hi2c1, target_address, tx_buffer, bytes_to_send, 1000);
}
```

### 3. Parsing Received Data

```c
uint8_t rx_buffer[32];  // Data received from I2C
uint16_t rx_length = 8; // Number of bytes received
I2C_Protocol_Message_t received_msg;

// Parse received bytes into message structure
status = I2C_Protocol_ParseMessage(rx_buffer, rx_length, &received_msg);

if (status == I2C_PROTOCOL_OK) {
    // Check if message is for us
    if (I2C_Protocol_IsMessageForDevice(&received_msg, MY_DEVICE_ID)) {
        // Process the message
        switch (received_msg.command) {
            case 0x1001:
                // Handle motor control command
                break;
            case 0x1002:
                // Handle status request
                break;
        }
    }
}
```

### 4. Complete I2C Slave Example

```c
// Your I2C slave callback (called by HAL)
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1) {
        // Parse received data
        I2C_Protocol_Message_t received_msg;
        I2C_Protocol_Status_t status = I2C_Protocol_ParseMessage(
            i2c_rx_buffer, 
            i2c_rx_length, 
            &received_msg
        );
        
        if (status == I2C_PROTOCOL_OK && 
            I2C_Protocol_IsMessageForDevice(&received_msg, MY_DEVICE_ID)) {
            
            // Process message in your application
            ProcessReceivedMessage(&received_msg);
        }
        
        // Restart I2C listening
        HAL_I2C_EnableListen_IT(hi2c);
    }
}

void ProcessReceivedMessage(const I2C_Protocol_Message_t *msg)
{
    switch (msg->command) {
        case CMD_SET_LED:
            if (msg->payload_length > 0) {
                HAL_GPIO_WritePin(LED_PORT, LED_PIN, msg->payload[0]);
            }
            break;
            
        case CMD_GET_STATUS:
            SendStatusResponse(msg->src_id);
            break;
    }
}
```

## Integration Steps

### 1. Add Library to Your Project
```bash
# Copy library files to your project
cp i2c_protocol_v2.h your_project/Core/Inc/
cp i2c_protocol_v2.c your_project/Core/Src/
```

### 2. Include in Your Code
```c
#include "i2c_protocol_v2.h"
```

### 3. Handle I2C Hardware Yourself
```c
// Set up I2C hardware as needed for your project
// Configure pins, clock, addressing, etc.
// Enable interrupts if desired

// In I2C receive callback:
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // Use library to parse received data
    I2C_Protocol_ParseMessage(rx_buffer, rx_length, &message);
    // Process message as needed
}
```

### 4. Define Your Protocol
```c
// Define your device IDs
#define MY_DEVICE_ID            0x01
#define MOTOR_CONTROLLER_ID     0x02
#define SENSOR_BOARD_ID         0x03

// Define your commands  
#define CMD_SET_MOTOR_SPEED     0x1001
#define CMD_GET_SENSOR_DATA     0x1002
#define CMD_EMERGENCY_STOP      0x1003
```

## Benefits of v2.0

**No Hardware Dependencies** - Works with any STM32 family  
**Simpler Integration** - Just process messages, handle hardware yourself  
**More Flexible** - Each project configures I2C as needed  
**Easier Testing** - Can test message handling without hardware  
**Better Separation** - Protocol logic separate from hardware layer  

## Migration from v1.0

If you're using the old library:

1. **Replace hardware setup** calls with your own I2C configuration
2. **Replace `I2C_SendMessage()`** with `I2C_Protocol_SerializeMessage()` + HAL call
3. **Replace `I2C_ReceiveMessage()`** with `I2C_Protocol_ParseMessage()`
4. **Update includes** to use `i2c_protocol_v2.h`

## Example Project Structure

```
your_project/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   └── i2c_protocol_v2.h      ← Library header
│   └── Src/
│       ├── main.c                 ← Your I2C setup + message handling
│       └── i2c_protocol_v2.c      ← Library implementation
├── your_project.ioc               ← Configure I2C pins here
└── README.md                      ← Document your specific setup
```

This simplified approach gives you full control over I2C hardware while providing robust message handling!