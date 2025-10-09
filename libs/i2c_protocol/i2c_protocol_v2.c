/**
 * @file i2c_protocol_v2.c
 * @brief Simplified I2C Protocol Library Implementation - Message Handling Only
 * @details Implementation of message formatting, parsing, and validation functions.
 *          Hardware setup is handled by individual projects.
 * 
 * @version 2.0
 * @date 2025
 */

#include "i2c_protocol_v2.h"

/* Private Constants ---------------------------------------------------------*/
#define I2C_CHECKSUM_ENABLED        1       /**< Enable checksum validation */

/* Message Processing Functions Implementation -------------------------------*/

I2C_Protocol_Status_t I2C_Protocol_CreateMessage(I2C_Protocol_Message_t *message, 
                                                  uint8_t src_id, uint8_t target_id, 
                                                  uint16_t command, 
                                                  const uint8_t *payload, 
                                                  uint8_t payload_length)
{
    if (message == NULL) {
        return I2C_PROTOCOL_INVALID_PARAM;
    }
    
    if (payload_length > I2C_MAX_PAYLOAD_SIZE) {
        return I2C_PROTOCOL_BUFFER_FULL;
    }
    
    // Fill message structure
    message->src_id = src_id;
    message->target_id = target_id;
    message->command = command;
    message->payload_length = payload_length;
    
    // Copy payload if provided
    if (payload_length > 0 && payload != NULL) {
        memcpy(message->payload, payload, payload_length);
    } else {
        message->payload_length = 0;
    }
    
    // Calculate and set checksum
    message->checksum = I2C_Protocol_CalculateChecksum(message);
    
    return I2C_PROTOCOL_OK;
}

I2C_Protocol_Status_t I2C_Protocol_SerializeMessage(const I2C_Protocol_Message_t *message,
                                                     uint8_t *buffer,
                                                     uint16_t buffer_size,
                                                     uint16_t *bytes_written)
{
    if (message == NULL || buffer == NULL || bytes_written == NULL) {
        return I2C_PROTOCOL_INVALID_PARAM;
    }
    
    uint16_t required_size = I2C_PROTOCOL_MESSAGE_TOTAL_SIZE(message);
    
    if (buffer_size < required_size) {
        return I2C_PROTOCOL_BUFFER_FULL;
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
    
    *bytes_written = idx;
    
    return I2C_PROTOCOL_OK;
}

I2C_Protocol_Status_t I2C_Protocol_ParseMessage(const uint8_t *buffer,
                                                 uint16_t buffer_length,
                                                 I2C_Protocol_Message_t *message)
{
    if (buffer == NULL || message == NULL) {
        return I2C_PROTOCOL_INVALID_PARAM;
    }
    
    if (buffer_length < I2C_HEADER_SIZE + 1) { // +1 for end marker
        return I2C_PROTOCOL_INVALID_MESSAGE;
    }
    
    uint16_t idx = 0;
    
    // Parse header
    message->src_id = buffer[idx++];
    message->target_id = buffer[idx++];
    message->command = ((uint16_t)buffer[idx++] << 8);    // Command high byte
    message->command |= (uint16_t)buffer[idx++];          // Command low byte
    message->payload_length = buffer[idx++];
    
    // Validate payload length
    if (message->payload_length > I2C_MAX_PAYLOAD_SIZE) {
        return I2C_PROTOCOL_INVALID_MESSAGE;
    }
    
    // Check if we have enough bytes for payload + end marker
    if (idx + message->payload_length + 1 > buffer_length) {
        return I2C_PROTOCOL_INVALID_MESSAGE;
    }
    
    // Parse payload
    if (message->payload_length > 0) {
        memcpy(message->payload, &buffer[idx], message->payload_length);
        idx += message->payload_length;
    }
    
    // Check end marker
    if (buffer[idx] != I2C_MESSAGE_END_MARKER) {
        return I2C_PROTOCOL_INVALID_MESSAGE;
    }
    
    // Calculate and set checksum for validation
    message->checksum = I2C_Protocol_CalculateChecksum(message);
    
    return I2C_PROTOCOL_OK;
}

bool I2C_Protocol_ValidateMessage(const I2C_Protocol_Message_t *message)
{
    if (message == NULL) {
        return false;
    }
    
    // Check payload length
    if (message->payload_length > I2C_MAX_PAYLOAD_SIZE) {
        return false;
    }
    
    // Check device IDs (allow any non-zero ID except reserved range for src)
    if (!I2C_PROTOCOL_IS_VALID_DEVICE_ID(message->src_id) && !I2C_PROTOCOL_IS_BROADCAST(message->src_id)) {
        return false;
    }
    
    // Target can be any valid ID or broadcast
    if (!I2C_PROTOCOL_IS_VALID_DEVICE_ID(message->target_id) && !I2C_PROTOCOL_IS_BROADCAST(message->target_id)) {
        return false;
    }
    
#if I2C_CHECKSUM_ENABLED
    // Validate checksum
    uint8_t calculated_checksum = I2C_Protocol_CalculateChecksum(message);
    if (calculated_checksum != message->checksum) {
        return false;
    }
#endif
    
    return true;
}

uint8_t I2C_Protocol_CalculateChecksum(const I2C_Protocol_Message_t *message)
{
    if (message == NULL) {
        return 0;
    }
    
    uint8_t checksum = 0;
    
    // Include header fields in checksum (excluding checksum itself)
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

bool I2C_Protocol_IsMessageForDevice(const I2C_Protocol_Message_t *message, uint8_t device_id)
{
    if (message == NULL) {
        return false;
    }
    
    return (message->target_id == device_id) || (message->target_id == DEVICE_ID_BROADCAST);
}

uint16_t I2C_Protocol_GetRequiredBufferSize(uint8_t payload_length)
{
    return I2C_HEADER_SIZE + payload_length + 1; // +1 for end marker
}

int16_t I2C_Protocol_FindEndMarker(const uint8_t *buffer, uint16_t buffer_length)
{
    if (buffer == NULL || buffer_length == 0) {
        return -1;
    }
    
    for (uint16_t i = 0; i < buffer_length; i++) {
        if (buffer[i] == I2C_MESSAGE_END_MARKER) {
            return (int16_t)i;
        }
    }
    
    return -1; // Not found
}

/* Statistics Functions Implementation ---------------------------------------*/

void I2C_Protocol_InitStats(I2C_Protocol_Stats_t *stats)
{
    if (stats != NULL) {
        memset(stats, 0, sizeof(I2C_Protocol_Stats_t));
    }
}

void I2C_Protocol_UpdateStats(I2C_Protocol_Stats_t *stats, I2C_Protocol_Status_t status)
{
    if (stats == NULL) {
        return;
    }
    
    switch (status) {
        case I2C_PROTOCOL_OK:
            stats->messages_processed++;
            break;
        case I2C_PROTOCOL_INVALID_MESSAGE:
            stats->invalid_messages++;
            break;
        case I2C_PROTOCOL_ERROR:
            stats->parse_errors++;
            break;
        case I2C_PROTOCOL_BUFFER_FULL:
        case I2C_PROTOCOL_INVALID_PARAM:
        default:
            stats->parse_errors++;
            break;
    }
}