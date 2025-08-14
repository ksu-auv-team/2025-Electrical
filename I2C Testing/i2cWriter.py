from machine import Pin, I2C

# Create I2C object
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # 400kHz

addr = 0x4B  # Replace with your device's address

# Write raw bytes directly to the device
i2c.writeto(addr, b'\x00\xA5')  # Example: sending two bytes 0x00, 0xA5