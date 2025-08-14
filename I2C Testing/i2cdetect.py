from machine import Pin, I2C

# Create I2C object
i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=400000)  # 400kHz

# Scan for devices
devices = i2c.scan()

if devices:
    print("I2C devices found:", [hex(dev) for dev in devices])
else:
    print("No I2C devices found")