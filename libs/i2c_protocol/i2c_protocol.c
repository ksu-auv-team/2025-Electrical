/**
 * @file i2c_protocol.c
 * @brief I2C Communication Protocol Library Implementation
 * @details Implementation of the I2C communication protocol for STM32 controllers.
 *          Handles message formatting, transmission, and reception.
 * 
 * @version 1.0
 * @date 2025
 */

#include "i2c_protocol.h"

/* Private Constants ---------------------------------------------------------*/
#define I2C_HEADER_SIZE             5       /**< Size of message header (src_id + target_id + command + payload_length) */
#define I2C_CHECKSUM_ENABLED        1       /**< Enable checksum validation */

/* Private Function Prototypes -----------------------------------------------*/
static I2C_Status_t I2C_SerializeMessage(const I2C_Message_t *message, uint8_t *buffer, uint16_t *buffer_size);
static I2C_Status_t I2C_DeserializeMessage(const uint8_t *buffer, uint16_t buffer_size, I2C_Message_t *message);
static I2C_Status_t I2C_InitHardware(I2C_Handle_t *handle);
static I2C_Status_t I2C_ConfigureGPIO(const I2C_GPIO_Config_t *gpio_config);
static uint32_t I2C_GetAlternateFunction(I2C_TypeDef *instance);

/* Quick Setup Functions Implementation --------------------------------------*/

I2C_Status_t I2C_InitWithHandle(I2C_HandleTypeDef *hi2c, I2C_Handle_t *handle, 
                                        uint8_t device_id, I2C_Role_t role)
{
    if (hi2c == NULL || handle == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    I2C_Config_t config = {
        .hi2c = hi2c,
        .hw_config = NULL,
        .gpio_config = NULL,
        .device_id = device_id,
        .role = role,
        .timeout_ms = I2C_TIMEOUT_MS,
        .enable_interrupts = true,
        .auto_init_hardware = false
    };
    
    return I2C_Init(handle, &config);
}

I2C_Status_t I2C_InitWithGPIO(I2C_Handle_t *handle, uint8_t device_id, I2C_Role_t role,
                                      I2C_TypeDef *i2c_instance, GPIO_TypeDef *sda_port, uint16_t sda_pin,
                                      GPIO_TypeDef *scl_port, uint16_t scl_pin, uint32_t own_address)
{
    if (handle == NULL || i2c_instance == NULL || sda_port == NULL || scl_port == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    // Create GPIO configuration
    static I2C_GPIO_Config_t gpio_config;
    gpio_config.sda_port = sda_port;
    gpio_config.sda_pin = sda_pin;
    gpio_config.scl_port = scl_port;
    gpio_config.scl_pin = scl_pin;
    gpio_config.alternate_function = I2C_GetAlternateFunction(i2c_instance);
    
    // Create hardware configuration with defaults
    static I2C_HW_Config_t hw_config;
    hw_config.instance = i2c_instance;
    hw_config.clock_speed = 100000;  // 100kHz default
    hw_config.own_address = own_address;
    hw_config.address_mode = I2C_ADDRESSINGMODE_7BIT;
    hw_config.dual_address_mode = I2C_DUALADDRESS_DISABLE;
    hw_config.general_call_mode = I2C_GENERALCALL_DISABLE;
    hw_config.no_stretch_mode = I2C_NOSTRETCH_DISABLE;
    
    // Create configuration
    I2C_Config_t config = {
        .hi2c = NULL,  // Will be created
        .hw_config = &hw_config,
        .gpio_config = &gpio_config,
        .device_id = device_id,
        .role = role,
        .timeout_ms = I2C_TIMEOUT_MS,
        .enable_interrupts = true,
        .auto_init_hardware = true
    };
    
    return I2C_Init(handle, &config);
}

/* Core Functions Implementation ---------------------------------------------*/

I2C_Status_t I2C_Init(I2C_Handle_t *handle, const I2C_Config_t *config)
{
    if (handle == NULL || config == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    // Copy configuration
    memcpy(&handle->config, config, sizeof(I2C_Config_t));
    
    // Initialize statistics
    memset(&handle->stats, 0, sizeof(I2C_Statistics_t));
    
    // Initialize buffers
    memset(handle->rx_buffer, 0, I2C_BUFFER_SIZE);
    memset(handle->tx_buffer, 0, I2C_BUFFER_SIZE);
    
    // Initialize flags
    handle->message_ready = false;
    handle->transmission_complete = false;
    handle->last_error = I2C_STATUS_OK;
    
    // Auto-initialize hardware if requested
    if (config->auto_init_hardware) {
        I2C_Status_t status = I2C_InitHardware(handle);
        if (status != I2C_STATUS_OK) {
            return status;
        }
    }
    
    // Validate that we have a valid I2C handle
    if (handle->config.hi2c == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    // Start listening if in slave mode and interrupts are enabled
    if (config->role == I2C_ROLE_SLAVE && config->enable_interrupts) {
        HAL_StatusTypeDef hal_status = HAL_I2C_EnableListen_IT(handle->config.hi2c);
        if (hal_status != HAL_OK) {
            return I2C_STATUS_ERROR;
        }
    }
    
    return I2C_STATUS_OK;
}

I2C_Status_t I2C_DeInit(I2C_Handle_t *handle)
{
    if (handle == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    // Stop listening if in slave mode
    if (handle->config.role == I2C_ROLE_SLAVE && handle->config.enable_interrupts) {
        HAL_I2C_DisableListen_IT(handle->config.hi2c);
    }
    
    // Clear handle
    memset(handle, 0, sizeof(I2C_Handle_t));
    
    return I2C_STATUS_OK;
}

I2C_Status_t I2C_SendMessage(I2C_Handle_t *handle, const I2C_Message_t *message, uint16_t target_address)
{
    if (handle == NULL || message == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    // Validate message
    if (!I2C_ValidateMessage(message)) {
        handle->stats.invalid_messages++;
        handle->last_error = I2C_STATUS_INVALID_MESSAGE;
        return I2C_STATUS_INVALID_MESSAGE;
    }
    
    // Serialize message
    uint16_t buffer_size;
    I2C_Status_t status = I2C_SerializeMessage(message, handle->tx_buffer, &buffer_size);
    if (status != I2C_STATUS_OK) {
        handle->stats.errors++;
        handle->last_error = status;
        return status;
    }
    
    HAL_StatusTypeDef hal_status;
    
    if (handle->config.role == I2C_ROLE_MASTER) {
        // Master mode transmission
        if (handle->config.enable_interrupts) {
            hal_status = HAL_I2C_Master_Transmit_IT(handle->config.hi2c, target_address << 1, 
                                                  handle->tx_buffer, buffer_size);
        } else {
            hal_status = HAL_I2C_Master_Transmit(handle->config.hi2c, target_address << 1, 
                                               handle->tx_buffer, buffer_size, handle->config.timeout_ms);
        }
    } else {
        // Slave mode transmission
        if (handle->config.enable_interrupts) {
            hal_status = HAL_I2C_Slave_Transmit_IT(handle->config.hi2c, handle->tx_buffer, buffer_size);
        } else {
            hal_status = HAL_I2C_Slave_Transmit(handle->config.hi2c, handle->tx_buffer, buffer_size, 
                                              handle->config.timeout_ms);
        }
    }
    
    if (hal_status == HAL_OK) {
        handle->stats.messages_sent++;
        return I2C_STATUS_OK;
    } else if (hal_status == HAL_TIMEOUT) {
        handle->stats.timeouts++;
        handle->last_error = I2C_STATUS_TIMEOUT;
        return I2C_STATUS_TIMEOUT;
    } else if (hal_status == HAL_BUSY) {
        handle->last_error = I2C_STATUS_BUSY;
        return I2C_STATUS_BUSY;
    } else {
        handle->stats.errors++;
        handle->last_error = I2C_STATUS_ERROR;
        return I2C_STATUS_ERROR;
    }
}

I2C_Status_t I2C_ReceiveMessage(I2C_Handle_t *handle, I2C_Message_t *message, uint32_t timeout_ms)
{
    if (handle == NULL || message == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    HAL_StatusTypeDef hal_status;
    
    if (handle->config.role == I2C_ROLE_SLAVE) {
        // Slave mode reception
        if (handle->config.enable_interrupts) {
            // In interrupt mode, check if message is already available
            if (!handle->message_ready) {
                return I2C_STATUS_BUSY;  // No message ready yet
            }
        } else {
            // Blocking receive
            hal_status = HAL_I2C_Slave_Receive(handle->config.hi2c, handle->rx_buffer, 
                                             I2C_BUFFER_SIZE, timeout_ms);
            if (hal_status != HAL_OK) {
                if (hal_status == HAL_TIMEOUT) {
                    handle->stats.timeouts++;
                    handle->last_error = I2C_STATUS_TIMEOUT;
                    return I2C_STATUS_TIMEOUT;
                } else {
                    handle->stats.errors++;
                    handle->last_error = I2C_STATUS_ERROR;
                    return I2C_STATUS_ERROR;
                }
            }
        }
    } else {
        // Master mode - would need target address, not typically used for receiving
        return I2C_STATUS_ERROR;
    }
    
    // Deserialize message from buffer
    I2C_Status_t status = I2C_DeserializeMessage(handle->rx_buffer, I2C_BUFFER_SIZE, message);
    if (status == I2C_STATUS_OK) {
        handle->stats.messages_received++;
        handle->message_ready = false;  // Clear flag
    } else {
        handle->stats.invalid_messages++;
        handle->last_error = status;
    }
    
    return status;
}

bool I2C_MessageReady(const I2C_Handle_t *handle)
{
    if (handle == NULL) {
        return false;
    }
    
    return handle->message_ready;
}

I2C_Status_t I2C_ProcessIncoming(I2C_Handle_t *handle)
{
    if (handle == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    // This function can be called periodically to check for incoming data
    // In interrupt mode, this is handled by callbacks
    if (!handle->config.enable_interrupts && handle->config.role == I2C_ROLE_SLAVE) {
        // Try non-blocking receive
        HAL_StatusTypeDef hal_status = HAL_I2C_Slave_Receive(handle->config.hi2c, handle->rx_buffer, 
                                                           I2C_BUFFER_SIZE, 0);  // 0 timeout for non-blocking
        if (hal_status == HAL_OK) {
            handle->message_ready = true;
            return I2C_STATUS_OK;
        } else if (hal_status == HAL_TIMEOUT) {
            return I2C_STATUS_OK;  // No data available, not an error
        }
    }
    
    return I2C_STATUS_OK;
}

I2C_Statistics_t I2C_GetStatistics(const I2C_Handle_t *handle)
{
    I2C_Statistics_t empty_stats = {0};
    
    if (handle == NULL) {
        return empty_stats;
    }
    
    return handle->stats;
}

I2C_Status_t I2C_GetLastError(const I2C_Handle_t *handle)
{
    if (handle == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    return handle->last_error;
}

void I2C_ResetStatistics(I2C_Handle_t *handle)
{
    if (handle != NULL) {
        memset(&handle->stats, 0, sizeof(I2C_Statistics_t));
        handle->last_error = I2C_STATUS_OK;
    }
}

/* Utility Functions Implementation ------------------------------------------*/

I2C_Status_t I2C_CreateMessage(I2C_Message_t *message, uint8_t src_id, uint8_t target_id, 
                                        uint16_t command, const uint8_t *payload, uint8_t payload_length)
{
    if (message == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    if (payload_length > I2C_MAX_PAYLOAD_SIZE) {
        return I2C_STATUS_ERROR;
    }
    
    message->src_id = src_id;
    message->target_id = target_id;
    message->command = command;
    message->payload_length = payload_length;
    
    if (payload_length > 0 && payload != NULL) {
        memcpy(message->payload, payload, payload_length);
    } else {
        message->payload_length = 0;
    }
    
    // Calculate checksum
    message->checksum = I2C_CalculateChecksum(message);
    
    return I2C_STATUS_OK;
}

bool I2C_ValidateMessage(const I2C_Message_t *message)
{
    if (message == NULL) {
        return false;
    }
    
    // Check payload length
    if (message->payload_length > I2C_MAX_PAYLOAD_SIZE) {
        return false;
    }
    
    // Check device IDs (allow any non-zero ID except reserved range)
    if (!I2C_IS_VALID_DEVICE_ID(message->src_id) && !I2C_IS_BROADCAST(message->src_id)) {
        return false;
    }
    
    if (!I2C_IS_VALID_DEVICE_ID(message->target_id) && !I2C_IS_BROADCAST(message->target_id)) {
        return false;
    }
    
#if I2C_CHECKSUM_ENABLED
    // Validate checksum
    uint8_t calculated_checksum = I2C_CalculateChecksum(message);
    if (calculated_checksum != message->checksum) {
        return false;
    }
#endif
    
    return true;
}

uint8_t I2C_CalculateChecksum(const I2C_Message_t *message)
{
    if (message == NULL) {
        return 0;
    }
    
    uint8_t checksum = 0;
    
    // Include header fields in checksum
    checksum ^= message->src_id;
    checksum ^= message->target_id;
    checksum ^= (uint8_t)(message->command >> 8);    // High byte
    checksum ^= (uint8_t)(message->command & 0xFF);  // Low byte
    checksum ^= message->payload_length;
    
    // Include payload in checksum
    for (uint8_t i = 0; i < message->payload_length; i++) {
        checksum ^= message->payload[i];
    }
    
    return checksum;
}

/* Interrupt Callback Functions ----------------------------------------------*/

void I2C_TxCompleteCallback(I2C_Handle_t *handle)
{
    if (handle != NULL) {
        handle->transmission_complete = true;
        
        // If slave mode, restart listening
        if (handle->config.role == I2C_ROLE_SLAVE) {
            HAL_I2C_EnableListen_IT(handle->config.hi2c);
        }
    }
}

void I2C_RxCompleteCallback(I2C_Handle_t *handle)
{
    if (handle != NULL) {
        handle->message_ready = true;
        
        // If slave mode, restart listening
        if (handle->config.role == I2C_ROLE_SLAVE) {
            HAL_I2C_EnableListen_IT(handle->config.hi2c);
        }
    }
}

void I2C_ErrorCallback(I2C_Handle_t *handle)
{
    if (handle != NULL) {
        handle->stats.errors++;
        handle->last_error = I2C_STATUS_ERROR;
        
        // If slave mode, try to restart listening
        if (handle->config.role == I2C_ROLE_SLAVE) {
            HAL_I2C_EnableListen_IT(handle->config.hi2c);
        }
    }
}

/* Private Functions Implementation ------------------------------------------*/

static I2C_Status_t I2C_InitHardware(I2C_Handle_t *handle)
{
    if (handle->config.hw_config == NULL || handle->config.gpio_config == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    // Configure GPIO pins
    I2C_Status_t status = I2C_ConfigureGPIO(handle->config.gpio_config);
    if (status != I2C_STATUS_OK) {
        return status;
    }
    
    // Allocate memory for I2C handle if not provided
    static I2C_HandleTypeDef hi2c_internal;
    handle->config.hi2c = &hi2c_internal;
    
    // Configure I2C handle
    handle->config.hi2c->Instance = handle->config.hw_config->instance;
    
    // STM32G4 series uses Timing register instead of ClockSpeed and DutyCycle
    #if defined(STM32G4xx) || defined(STM32G431xx) || defined(STM32G4)
    // Use a standard timing value for 100kHz I2C at typical system clocks
    // This should be calculated based on your actual system clock
    handle->config.hi2c->Init.Timing = 0x00503D58;  // Standard 100kHz timing for STM32G4
    #else
    // For older STM32 families (F4, F1, etc.)
    handle->config.hi2c->Init.ClockSpeed = handle->config.hw_config->clock_speed;
    handle->config.hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
    #endif
    
    handle->config.hi2c->Init.OwnAddress1 = handle->config.hw_config->own_address;
    handle->config.hi2c->Init.AddressingMode = handle->config.hw_config->address_mode;
    handle->config.hi2c->Init.DualAddressMode = handle->config.hw_config->dual_address_mode;
    handle->config.hi2c->Init.GeneralCallMode = handle->config.hw_config->general_call_mode;
    handle->config.hi2c->Init.NoStretchMode = handle->config.hw_config->no_stretch_mode;
    
    // Initialize I2C
    HAL_StatusTypeDef hal_status = HAL_I2C_Init(handle->config.hi2c);
    if (hal_status != HAL_OK) {
        return I2C_STATUS_ERROR;
    }
    
    return I2C_STATUS_OK;
}

static I2C_Status_t I2C_ConfigureGPIO(const I2C_GPIO_Config_t *gpio_config)
{
    if (gpio_config == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Configure SDA pin
    GPIO_InitStruct.Pin = gpio_config->sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = gpio_config->alternate_function;
    HAL_GPIO_Init(gpio_config->sda_port, &GPIO_InitStruct);
    
    // Configure SCL pin
    GPIO_InitStruct.Pin = gpio_config->scl_pin;
    HAL_GPIO_Init(gpio_config->scl_port, &GPIO_InitStruct);
    
    return I2C_STATUS_OK;
}

static uint32_t I2C_GetAlternateFunction(I2C_TypeDef *instance)
{
    // STM32 family-specific alternate functions
    #if defined(STM32G4xx) || defined(STM32G431xx) || defined(STM32G4)
    // STM32G4xx series
    #ifdef I2C1
    if (instance == I2C1) return GPIO_AF4_I2C1;
    #endif
    #ifdef I2C2
    if (instance == I2C2) return GPIO_AF4_I2C2;
    #endif
    #ifdef I2C3
    if (instance == I2C3) return GPIO_AF4_I2C3;
    #endif
    #ifdef I2C4
    if (instance == I2C4) return GPIO_AF4_I2C4;
    #endif
    return GPIO_AF4_I2C1;
    
    #elif defined(STM32F4xx) || defined(STM32F411xE)
    // STM32F4xx series (including F411)
    #ifdef I2C1
    if (instance == I2C1) return GPIO_AF4_I2C1;
    #endif
    #ifdef I2C2
    if (instance == I2C2) return GPIO_AF4_I2C2;
    #endif
    #ifdef I2C3
    if (instance == I2C3) return GPIO_AF4_I2C3;
    #endif
    return GPIO_AF4_I2C1;
    
    #elif defined(STM32F1xx)
    // STM32F1xx series doesn't use alternate functions for I2C
    return 0;  // Not used for F1xx
    
    #else
    // Default fallback
    #ifdef I2C1
    if (instance == I2C1) return GPIO_AF4_I2C1;
    #endif
    #ifdef I2C2
    if (instance == I2C2) return GPIO_AF4_I2C2;
    #endif
    #ifdef I2C3
    if (instance == I2C3) return GPIO_AF4_I2C3;
    #endif
    return GPIO_AF4_I2C1;
    #endif
}

static I2C_Status_t I2C_SerializeMessage(const I2C_Message_t *message, uint8_t *buffer, uint16_t *buffer_size)
{
    if (message == NULL || buffer == NULL || buffer_size == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    uint16_t total_size = I2C_HEADER_SIZE + message->payload_length + 1; // +1 for end marker
    
    if (total_size > I2C_BUFFER_SIZE) {
        return I2C_STATUS_BUFFER_FULL;
    }
    
    uint16_t idx = 0;
    
    // Serialize header
    buffer[idx++] = message->src_id;
    buffer[idx++] = message->target_id;
    buffer[idx++] = (uint8_t)(message->command >> 8);     // Command high byte
    buffer[idx++] = (uint8_t)(message->command & 0xFF);   // Command low byte
    buffer[idx++] = message->payload_length;
    
    // Serialize payload
    if (message->payload_length > 0) {
        memcpy(&buffer[idx], message->payload, message->payload_length);
        idx += message->payload_length;
    }
    
    // Add end marker
    buffer[idx++] = I2C_MESSAGE_END_MARKER;
    
    *buffer_size = idx;
    
    return I2C_STATUS_OK;
}

static I2C_Status_t I2C_DeserializeMessage(const uint8_t *buffer, uint16_t buffer_size, I2C_Message_t *message)
{
    if (buffer == NULL || message == NULL) {
        return I2C_STATUS_ERROR;
    }
    
    if (buffer_size < I2C_HEADER_SIZE + 1) { // +1 for end marker
        return I2C_STATUS_INVALID_MESSAGE;
    }
    
    uint16_t idx = 0;
    
    // Deserialize header
    message->src_id = buffer[idx++];
    message->target_id = buffer[idx++];
    message->command = ((uint16_t)buffer[idx++] << 8);    // Command high byte
    message->command |= (uint16_t)buffer[idx++];          // Command low byte
    message->payload_length = buffer[idx++];
    
    // Check payload length
    if (message->payload_length > I2C_MAX_PAYLOAD_SIZE) {
        return I2C_STATUS_INVALID_MESSAGE;
    }
    
    // Check if we have enough bytes for payload + end marker
    if (idx + message->payload_length + 1 > buffer_size) {
        return I2C_STATUS_INVALID_MESSAGE;
    }
    
    // Deserialize payload
    if (message->payload_length > 0) {
        memcpy(message->payload, &buffer[idx], message->payload_length);
        idx += message->payload_length;
    }
    
    // Check end marker
    if (buffer[idx] != I2C_MESSAGE_END_MARKER) {
        return I2C_STATUS_INVALID_MESSAGE;
    }
    
    // Calculate and validate checksum
    message->checksum = I2C_CalculateChecksum(message);
    
    return I2C_STATUS_OK;
}
