# Test code for HW-611 (BME280) altimeter 

import smbus2
import bme280
from time import sleep

# Altitude above sea level
def barometric_altitude(P, P_0):
    return 44330 * (1 - (P / P_0) ** (1 / 5.255))


address = 0x76
bus = smbus2.SMBus(1)
calibration_params = bme280.load_calibration_params(bus, address)

try:
    while True:
        data = bme280.sample(bus, address, calibration_params)
        print(barometric_altitude(data.pressure, 1013.25))
        sleep(0.25)
except KeyboardInterrupt:
    print("\r", end="")
    print("Program halted.")
