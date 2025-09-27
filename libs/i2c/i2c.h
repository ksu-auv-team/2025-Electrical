/*
	Kennesaw State Univerisity AUV I2C Communication Protocol
	Created by: KSU AUV Team - 2025-2026


	Used for communication between the Orin and the embedded subordinates.

	Message Format: [src_id, target_id, command, payload, close]
	- src_id: ID of the sender (1 byte)
	- target_id: ID of the receiver (1 byte)
	- command: Command or data type (2 byte)
	- payload: Data being sent (variable length)
	- close: End of message indicator (1 byte, e.g., 0xFF)
*/
