/**
 * @file i2c_protocol_v2.h
 * @brief Simplified I2C Protocol Library - Message Handling Only
 * @details Handles only message formatting, parsing, and validation.
 *          Hardware setup (pins, I2C config) is handled by individual projects.
 * 
 * Message Format: [src_id][target_id][command_high][command_low][payload_length][payload...][0xFF]
 * - src_id: Source device ID (1 byte)
 * - target_id: Target device ID (1 byte)
 * - command: Command or data type (2 bytes, big-endian)
 * - payload_length: Length of payload data (1 byte)
 * - payload: Variable length data (0-64 bytes)
 * - 0xFF: End of message marker (1 byte)
 * 
 * @version 2.0
 * @date 2025
 */

#ifndef I2C_PROTOCOL_V2_H
#define I2C_PROTOCOL_V2_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Protocol Constants --------------------------------------------------------*/
#define I2C_MESSAGE_END_MARKER      0xFF    /**< Message end marker */
#define I2C_MAX_PAYLOAD_SIZE        64      /**< Maximum payload size in bytes */
#define I2C_MAX_MESSAGE_SIZE        (I2C_MAX_PAYLOAD_SIZE + 6)  /**< Max total message size */
#define I2C_HEADER_SIZE             5       /**< Size of message header */

/* Reserved Device IDs -------------------------------------------------------*/
#define DEVICE_ID_BROADCAST     0xFF    /**< Broadcast to all devices */
#define DEVICE_ID_RESERVED_MIN  0xF0    /**< Start of reserved ID range */
#define DEVICE_ID_RESERVED_MAX  0xFE    /**< End of reserved ID range */

/* Reserved Command Types ----------------------------------------------------*/
#define CMD_RESERVED_MIN        0xFF00  /**< Start of reserved command range */
#define CMD_RESERVED_MAX        0xFFFF  /**< End of reserved command range */

/* Status Codes --------------------------------------------------------------*/
typedef enum {
    I2C_PROTOCOL_OK              = 0x00,  /**< Operation successful */
    I2C_PROTOCOL_ERROR           = 0x01,  /**< General error */
    I2C_PROTOCOL_INVALID_MESSAGE = 0x02,  /**< Invalid message format */
    I2C_PROTOCOL_BUFFER_FULL     = 0x03,  /**< Buffer overflow */
    I2C_PROTOCOL_INVALID_PARAM   = 0x04   /**< Invalid parameter */
} I2C_Protocol_Status_t;

/* Message Structure ---------------------------------------------------------*/
typedef struct {
    uint8_t src_id;                /**< Source device ID */
    uint8_t target_id;             /**< Target device ID */
    uint16_t command;              /**< Command/data type */
    uint8_t payload_length;        /**< Length of payload data */
    uint8_t payload[I2C_MAX_PAYLOAD_SIZE];  /**< Payload data */
    uint8_t checksum;              /**< Message checksum */
} I2C_Protocol_Message_t;

/* Statistics Structure ------------------------------------------------------*/
typedef struct {
    uint32_t messages_processed;   /**< Total messages processed */
    uint32_t messages_created;     /**< Total messages created */
    uint32_t parse_errors;         /**< Total parse errors */
    uint32_t checksum_errors;      /**< Total checksum errors */
    uint32_t invalid_messages;     /**< Total invalid messages */
} I2C_Protocol_Stats_t;

/* Message Processing Functions ----------------------------------------------*/

/**
 * @brief Create a protocol message
 * @param message Pointer to message structure to fill
 * @param src_id Source device ID
 * @param target_id Target device ID
 * @param command Command type
 * @param payload Pointer to payload data (can be NULL if payload_length is 0)
 * @param payload_length Length of payload data (0-64 bytes)
 * @return I2C_Protocol_Status_t Status of message creation
 */
I2C_Protocol_Status_t I2C_Protocol_CreateMessage(I2C_Protocol_Message_t *message, 
                                                  uint8_t src_id, uint8_t target_id, 
                                                  uint16_t command, 
                                                  const uint8_t *payload, 
                                                  uint8_t payload_length);

/**
 * @brief Serialize message to buffer for transmission
 * @param message Pointer to message to serialize
 * @param buffer Buffer to write serialized data to
 * @param buffer_size Size of the buffer
 * @param bytes_written Pointer to store number of bytes written
 * @return I2C_Protocol_Status_t Status of serialization
 */
I2C_Protocol_Status_t I2C_Protocol_SerializeMessage(const I2C_Protocol_Message_t *message,
                                                     uint8_t *buffer,
                                                     uint16_t buffer_size,
                                                     uint16_t *bytes_written);

/**
 * @brief Parse received data buffer into message
 * @param buffer Buffer containing received data
 * @param buffer_length Length of received data
 * @param message Pointer to message structure to fill
 * @return I2C_Protocol_Status_t Status of parsing
 */
I2C_Protocol_Status_t I2C_Protocol_ParseMessage(const uint8_t *buffer,
                                                 uint16_t buffer_length,
                                                 I2C_Protocol_Message_t *message);

/**
 * @brief Validate message format and checksum
 * @param message Pointer to message to validate
 * @return bool True if message is valid, false otherwise
 */
bool I2C_Protocol_ValidateMessage(const I2C_Protocol_Message_t *message);

/**
 * @brief Calculate message checksum
 * @param message Pointer to message
 * @return uint8_t Calculated checksum
 */
uint8_t I2C_Protocol_CalculateChecksum(const I2C_Protocol_Message_t *message);

/**
 * @brief Check if message is intended for specific device
 * @param message Pointer to message to check
 * @param device_id Device ID to check against
 * @return bool True if message is for this device or broadcast
 */
bool I2C_Protocol_IsMessageForDevice(const I2C_Protocol_Message_t *message, uint8_t device_id);

/**
 * @brief Get minimum buffer size needed for a message
 * @param payload_length Length of payload
 * @return uint16_t Required buffer size
 */
uint16_t I2C_Protocol_GetRequiredBufferSize(uint8_t payload_length);

/**
 * @brief Find message end marker in buffer
 * @param buffer Buffer to search
 * @param buffer_length Length of buffer
 * @return int16_t Position of end marker, -1 if not found
 */
int16_t I2C_Protocol_FindEndMarker(const uint8_t *buffer, uint16_t buffer_length);

/* Statistics Functions ------------------------------------------------------*/

/**
 * @brief Initialize statistics structure
 * @param stats Pointer to statistics structure
 */
void I2C_Protocol_InitStats(I2C_Protocol_Stats_t *stats);

/**
 * @brief Update statistics for message processing
 * @param stats Pointer to statistics structure
 * @param status Result of message processing
 */
void I2C_Protocol_UpdateStats(I2C_Protocol_Stats_t *stats, I2C_Protocol_Status_t status);

/* Helper Macros -------------------------------------------------------------*/
#define I2C_PROTOCOL_IS_VALID_DEVICE_ID(id)     ((id) > 0 && (id) < DEVICE_ID_RESERVED_MIN)
#define I2C_PROTOCOL_IS_BROADCAST(id)            ((id) == DEVICE_ID_BROADCAST)
#define I2C_PROTOCOL_IS_RESERVED_DEVICE_ID(id)   ((id) >= DEVICE_ID_RESERVED_MIN && (id) <= DEVICE_ID_RESERVED_MAX)
#define I2C_PROTOCOL_IS_RESERVED_COMMAND(cmd)    ((cmd) >= CMD_RESERVED_MIN && (cmd) <= CMD_RESERVED_MAX)
#define I2C_PROTOCOL_MESSAGE_TOTAL_SIZE(msg)     (I2C_HEADER_SIZE + (msg)->payload_length + 1)

/* Example Usage Callback Type -----------------------------------------------*/
typedef void (*I2C_Protocol_MessageHandler_t)(const I2C_Protocol_Message_t *message, void *context);

#ifdef __cplusplus
}
#endif

#endif /* I2C_PROTOCOL_V2_H */