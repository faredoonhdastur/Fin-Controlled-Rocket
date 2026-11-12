# Code for mpu6050 gyroscope/accelerometer

from mpu6050 import mpu6050
from time import sleep

sensor = mpu6050(0x68)

try:
    while True:
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()
        temp = sensor.get_temp()
        # Print accelerometer data.
        print("Accelerometer data")
        print("x: " + str(accel_data['x']))
        print("y: " + str(accel_data['y']))
        print("z: " + str(accel_data['z']))

        # Print gyroscope data.
        print("Gyroscope data")
        print("x: " + str(gyro_data['x']))
        print("y: " + str(gyro_data['y']))
        print("z: " + str(gyro_data['z']))

        # Print the temperature in Celsius.
        print("Temp: " + str(temp) + " C")

        # Pause for 0.5 seconds before the next read cycle.
        sleep(0.5)
except KeyboardInterrupt:
    print("\r", end="")
    print("Program halted.")
