/**
 * @file i2c_protocol.h
 * @brief I2C Communication Protocol Library
 * @details Provides I2C communication protocol implementation for STM32 controllers.
 *          Handles message formatting, transmission, and reception.
 * 
 * Message Format: [src_id, target_id, command, payload, close]
 * - src_id: ID of the sender (1 byte)
 * - target_id: ID of the receiver (1 byte)
 * - command: Command or data type (2 bytes)
 * - payload: Data being sent (variable length)
 * - close: End of message indicator (1 byte, e.g., 0xFF)
 * 
 * @version 1.0
 * @date 2025
 */

#ifndef I2C_PROTOCOL_H
#define I2C_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Protocol Constants --------------------------------------------------------*/
#define I2C_MESSAGE_START_MARKER    0xAA    /**< Message start marker */
#define I2C_MESSAGE_END_MARKER      0xFF    /**< Message end marker */
#define I2C_MAX_PAYLOAD_SIZE        64      /**< Maximum payload size in bytes */
#define I2C_MAX_MESSAGE_SIZE        (I2C_MAX_PAYLOAD_SIZE + 6)  /**< Max total message size */
#define I2C_TIMEOUT_MS              1000    /**< Default I2C timeout in milliseconds */
#define I2C_BUFFER_SIZE             128     /**< Internal buffer size */

/* Reserved Device IDs -------------------------------------------------------*/
#define DEVICE_ID_BROADCAST     0xFF    /**< Broadcast to all devices */
#define DEVICE_ID_RESERVED_MIN  0xF0    /**< Start of reserved ID range */
#define DEVICE_ID_RESERVED_MAX  0xFE    /**< End of reserved ID range */

/* Reserved Command Types ----------------------------------------------------*/
#define CMD_RESERVED_MIN        0xFF00  /**< Start of reserved command range */
#define CMD_RESERVED_MAX        0xFFFF  /**< End of reserved command range */

/* Status Codes --------------------------------------------------------------*/
typedef enum {
    I2C_STATUS_OK              = 0x00,  /**< Operation successful */
    I2C_STATUS_ERROR           = 0x01,  /**< General error */
    I2C_STATUS_TIMEOUT         = 0x02,  /**< Communication timeout */
    I2C_STATUS_INVALID_MESSAGE = 0x03,  /**< Invalid message format */
    I2C_STATUS_BUFFER_FULL     = 0x04,  /**< Buffer overflow */
    I2C_STATUS_NOT_INITIALIZED = 0x05,  /**< Library not initialized */
    I2C_STATUS_BUSY            = 0x06   /**< I2C bus busy */
} I2C_Status_t;

/* I2C Role ------------------------------------------------------------------*/
typedef enum {
    I2C_ROLE_MASTER = 0,       /**< I2C Master mode */
    I2C_ROLE_SLAVE  = 1        /**< I2C Slave mode */
} I2C_Role_t;

/* GPIO Configuration Structure ----------------------------------------------*/
typedef struct {
    GPIO_TypeDef *sda_port;        /**< SDA GPIO port */
    uint16_t sda_pin;              /**< SDA GPIO pin */
    GPIO_TypeDef *scl_port;        /**< SCL GPIO port */
    uint16_t scl_pin;              /**< SCL GPIO pin */
    uint32_t alternate_function;   /**< GPIO alternate function for I2C */
} I2C_GPIO_Config_t;

/* I2C Hardware Configuration ------------------------------------------------*/
typedef struct {
    I2C_TypeDef *instance;         /**< I2C peripheral instance (I2C1, I2C2, etc.) */
    uint32_t clock_speed;          /**< I2C clock speed in Hz */
    uint32_t own_address;          /**< Own address when in slave mode (7-bit) */
    uint32_t address_mode;         /**< I2C_ADDRESSINGMODE_7BIT or I2C_ADDRESSINGMODE_10BIT */
    uint32_t dual_address_mode;    /**< I2C_DUALADDRESS_DISABLE or I2C_DUALADDRESS_ENABLE */
    uint32_t general_call_mode;    /**< I2C_GENERALCALL_DISABLE or I2C_GENERALCALL_ENABLE */
    uint32_t no_stretch_mode;      /**< I2C_NOSTRETCH_DISABLE or I2C_NOSTRETCH_ENABLE */
} I2C_HW_Config_t;

/* Configuration Structure ---------------------------------------------------*/
typedef struct {
    I2C_HandleTypeDef *hi2c;       /**< STM32 HAL I2C handle (if using existing handle) */
    I2C_HW_Config_t *hw_config; /**< Hardware configuration (if initializing from scratch) */
    I2C_GPIO_Config_t *gpio_config; /**< GPIO configuration (if initializing from scratch) */
    uint8_t device_id;             /**< This device's ID */
    I2C_Role_t role;           /**< Master or Slave role */
    uint32_t timeout_ms;           /**< Communication timeout in milliseconds */
    bool enable_interrupts;        /**< Enable interrupt-based communication */
    bool auto_init_hardware;       /**< Auto-initialize I2C hardware with provided configs */
} I2C_Config_t;

/* Message Structure ---------------------------------------------------------*/
typedef struct {
    uint8_t src_id;                /**< Source device ID */
    uint8_t target_id;             /**< Target device ID */
    uint16_t command;              /**< Command/data type */
    uint8_t payload_length;        /**< Length of payload data */
    uint8_t payload[I2C_MAX_PAYLOAD_SIZE];  /**< Payload data */
    uint8_t checksum;              /**< Message checksum (optional) */
} I2C_Message_t;

/* Statistics Structure ------------------------------------------------------*/
typedef struct {
    uint32_t messages_sent;        /**< Total messages sent */
    uint32_t messages_received;    /**< Total messages received */
    uint32_t errors;               /**< Total errors encountered */
    uint32_t timeouts;             /**< Total timeouts */
    uint32_t invalid_messages;     /**< Total invalid messages */
} I2C_Statistics_t;

/* Handle Structure ----------------------------------------------------------*/
typedef struct {
    I2C_Config_t config;       /**< Configuration */
    I2C_Statistics_t stats;    /**< Communication statistics */
    uint8_t rx_buffer[I2C_BUFFER_SIZE];  /**< Receive buffer */
    uint8_t tx_buffer[I2C_BUFFER_SIZE];  /**< Transmit buffer */
    volatile bool message_ready;    /**< Flag indicating new message received */
    volatile bool transmission_complete; /**< Flag indicating transmission complete */
    I2C_Status_t last_error;   /**< Last error status */
} I2C_Handle_t;

/* Quick Setup Functions -----------------------------------------------------*/

/**
 * @brief Quick setup with existing I2C handle
 * @param hi2c Pointer to already configured STM32 HAL I2C handle
 * @param handle Pointer to I2C handle to initialize
 * @param device_id This device's ID
 * @param role Master or Slave role
 * @return I2C_Status_t Status of initialization
 */
I2C_Status_t I2C_InitWithHandle(I2C_HandleTypeDef *hi2c, I2C_Handle_t *handle, 
                                        uint8_t device_id, I2C_Role_t role);

/**
 * @brief Quick setup with GPIO pins (auto-configures I2C)
 * @param handle Pointer to I2C handle to initialize
 * @param device_id This device's ID
 * @param role Master or Slave role
 * @param i2c_instance I2C peripheral instance (I2C1, I2C2, etc.)
 * @param sda_port SDA GPIO port
 * @param sda_pin SDA GPIO pin
 * @param scl_port SCL GPIO port
 * @param scl_pin SCL GPIO pin
 * @param own_address Own address when in slave mode (7-bit)
 * @return I2C_Status_t Status of initialization
 */
I2C_Status_t I2C_InitWithGPIO(I2C_Handle_t *handle, uint8_t device_id, I2C_Role_t role,
                                      I2C_TypeDef *i2c_instance, GPIO_TypeDef *sda_port, uint16_t sda_pin,
                                      GPIO_TypeDef *scl_port, uint16_t scl_pin, uint32_t own_address);

/* Core Functions ------------------------------------------------------------*/

/**
 * @brief Initialize I2C protocol with custom configuration
 * @param handle Pointer to I2C handle
 * @param config Pointer to configuration structure
 * @return I2C_Status_t Status of initialization
 */
I2C_Status_t I2C_Init(I2C_Handle_t *handle, const I2C_Config_t *config);

/**
 * @brief Deinitialize I2C protocol
 * @param handle Pointer to I2C handle
 * @return I2C_Status_t Status of deinitialization
 */
I2C_Status_t I2C_DeInit(I2C_Handle_t *handle);

/**
 * @brief Send a message via I2C
 * @param handle Pointer to I2C handle
 * @param message Pointer to message to send
 * @param target_address Target device I2C address (for master mode)
 * @return I2C_Status_t Status of transmission
 */
I2C_Status_t I2C_SendMessage(I2C_Handle_t *handle, const I2C_Message_t *message, uint16_t target_address);

/**
 * @brief Receive a message via I2C (blocking)
 * @param handle Pointer to I2C handle
 * @param message Pointer to message structure to fill
 * @param timeout_ms Timeout in milliseconds
 * @return I2C_Status_t Status of reception
 */
I2C_Status_t I2C_ReceiveMessage(I2C_Handle_t *handle, I2C_Message_t *message, uint32_t timeout_ms);

/**
 * @brief Check if a new message is available
 * @param handle Pointer to I2C handle
 * @return bool True if message is ready, false otherwise
 */
bool I2C_MessageReady(const I2C_Handle_t *handle);

/**
 * @brief Process incoming I2C data (non-blocking)
 * @param handle Pointer to I2C handle
 * @return I2C_Status_t Status of processing
 */
I2C_Status_t I2C_ProcessIncoming(I2C_Handle_t *handle);

/**
 * @brief Get communication statistics
 * @param handle Pointer to I2C handle
 * @return I2C_Statistics_t Current statistics
 */
I2C_Statistics_t I2C_GetStatistics(const I2C_Handle_t *handle);

/**
 * @brief Get last error status
 * @param handle Pointer to I2C handle
 * @return I2C_Status_t Last error status
 */
I2C_Status_t I2C_GetLastError(const I2C_Handle_t *handle);

/**
 * @brief Reset statistics and error counters
 * @param handle Pointer to I2C handle
 */
void I2C_ResetStatistics(I2C_Handle_t *handle);

/* Utility Functions ---------------------------------------------------------*/

/**
 * @brief Create a message with specified parameters
 * @param message Pointer to message structure to fill
 * @param src_id Source device ID
 * @param target_id Target device ID
 * @param command Command type
 * @param payload Pointer to payload data (can be NULL)
 * @param payload_length Length of payload data
 * @return I2C_Status_t Status of message creation
 */
I2C_Status_t I2C_CreateMessage(I2C_Message_t *message, uint8_t src_id, uint8_t target_id, 
                                        uint16_t command, const uint8_t *payload, uint8_t payload_length);

/**
 * @brief Validate message format and checksum
 * @param message Pointer to message to validate
 * @return bool True if message is valid, false otherwise
 */
bool I2C_ValidateMessage(const I2C_Message_t *message);

/**
 * @brief Calculate message checksum
 * @param message Pointer to message
 * @return uint8_t Calculated checksum
 */
uint8_t I2C_CalculateChecksum(const I2C_Message_t *message);

/* Interrupt Callbacks -------------------------------------------------------*/

/**
 * @brief I2C transmission complete callback (call from HAL callback)
 * @param handle Pointer to I2C handle
 */
void I2C_TxCompleteCallback(I2C_Handle_t *handle);

/**
 * @brief I2C reception complete callback (call from HAL callback)
 * @param handle Pointer to I2C handle
 */
void I2C_RxCompleteCallback(I2C_Handle_t *handle);

/**
 * @brief I2C error callback (call from HAL callback)
 * @param handle Pointer to I2C handle
 */
void I2C_ErrorCallback(I2C_Handle_t *handle);

/* Helper Macros -------------------------------------------------------------*/
#define I2C_IS_VALID_DEVICE_ID(id)     ((id) > 0 && (id) < DEVICE_ID_RESERVED_MIN)
#define I2C_IS_BROADCAST(id)            ((id) == DEVICE_ID_BROADCAST)
#define I2C_IS_RESERVED_DEVICE_ID(id)   ((id) >= DEVICE_ID_RESERVED_MIN && (id) <= DEVICE_ID_RESERVED_MAX)
#define I2C_IS_RESERVED_COMMAND(cmd)    ((cmd) >= CMD_RESERVED_MIN && (cmd) <= CMD_RESERVED_MAX)
#define I2C_MESSAGE_TOTAL_SIZE(msg)     (5 + (msg)->payload_length + 1)  // header + payload + close

#ifdef __cplusplus
}
#endif

#endif /* I2C_PROTOCOL_H */