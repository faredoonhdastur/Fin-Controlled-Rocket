# Simple flight program for the fin-controlled rocket.
# python flight.py          # flight: calibrates, waits for launch, flies, deploys
# python flight.py bench    # bench test: hold the rocket nose-up, tilt it, watch the fins


# before launch: ensure nothing has tilted inside avionics bay and MPU6050 is facing upwards.
# In flight: turn the "up" vector with the gyro, so we always know how far the nose has tilted away from the vertical axis and in which direction.
# Fin angle = (KP * tilt  -  KD * rotation rate) * (V_REF / speed)^2
# Apogee: once altitude has dropped APOGEE_DROP meters below max alt, then open the nosecone.

# Axes used everywhere below ("body axes"): x = toward the nose,  y = toward the BLUE fin,  z = toward the ORANGE fin

import csv
import math
import os
import sys
import time

from helpers import blend, body_axes, clamp, length, rotate, to_body

LOOP_HZ = 50

# how the MPU6050 is mounted: which sensor axis points where (check this on the bench)
NOSE = ("y", +1)
BLUE = ("x", +1)
AXES = body_axes(NOSE, BLUE)

FIN_PINS = {"blue": 12, "green": 16, "yellow": 20, "orange": 21}
FIN_SIGN = {"blue": +1, "green": +1, "yellow": +1, "orange": +1}        # set to -1 if a fin moves the wrong way
FIN_TRIM = {"blue": 0.0, "green": 0.0, "yellow": 0.0, "orange": 0.0}    # degrees, to centre each fin
DEG_PER_UNIT = 45.0         # fin degrees at servo value 1.0
MAX_FIN = 10.0              # degrees

NOSECONE_PIN = 26
NOSECONE_LOCKED = -1.0
NOSECONE_OPEN = +1.0        # releases the parachute

# gains are tuned at V_REF and get scaled by (V_REF / speed) ** SPEED_POWER,
# since fin force grows with speed squared
KP = 0.40                   # fin deg per deg of tilt
KD = 0.045                  # fin deg per deg/s of pitch or yaw rate
KROLL = 0.0025              # fin deg per deg/s of roll rate
KROLL_ANGLE = 0.007         # fin deg per deg rolled since launch, stops a slow spin
V_REF = 50.0                # m/s
V_MIN = 20.0                # m/s, gains stop growing below this
SPEED_POWER = 2

CALIBRATION_TIME = 3.0      # s the rocket has to sit still after start-up
LAUNCH_G = 3.0              # this many g for LAUNCH_SAMPLES loops in a row = lift-off
LAUNCH_SAMPLES = 5
CONTROL_DELAY = 0.3         # s before the fins start moving, to clear the rail
MIN_FLIGHT_TIME = 3.0       # s, never deploy before this
APOGEE_DROP = 3.0           # m
BACKUP_DEPLOY = 13.0        # s, deploy no matter what. Set to expected apogee time + 3
BARO_EVERY = 5              # barometer is slow, so only read it every 5th loop (10 Hz)
LOG_AFTER_DEPLOY = 120.0    # s, then the program ends

G = 9.81
FIN_ORDER = ("blue", "green", "yellow", "orange")


# Just the flight logic without hardware to test it on pc
class Flight:

    def __init__(self, bench=False):
        self.bench = bench
        self.phase = "pad"              # pad -> flight -> descent
        self.t_start = None
        self.t_prev = None
        self.t_launch = None
        self.t_deploy = None
        self.bias = None                # gyro bias, deg/s
        self.up = None                  # direction of "up" in body axes
        self.ground = None              # barometer altitude of the pad
        self.alt = 0.0                  # height above the pad
        self.max_alt = 0.0
        self.speed = 0.0
        self.launch_count = 0
        self.tilt = 0.0                 # degrees from vertical
        self.roll_angle = 0.0           # degrees rolled since launch
        self.fins = dict.fromkeys(FIN_ORDER, 0.0)

    # Call once per loop. gyro in deg/s and accel in m/s^2 (both body axes), altitude in meters.
    # Returns the fin angles in degrees and whether to deploy.
    def step(self, t, gyro, accel, altitude=None):
        if self.t_start is None:
            self.t_start = self.t_prev = t
        dt = min(t - self.t_prev, 0.1)
        self.t_prev = t
        self.fins = dict.fromkeys(FIN_ORDER, 0.0)

        if self.phase == "pad":
            self._pad(t, dt, gyro, accel, altitude)
        elif self.phase == "flight":
            self._fly(t, dt, gyro, accel, altitude)
        return self.fins, self.phase == "descent"

    def _pad(self, t, dt, gyro, accel, altitude):
        a = length(accel)
        # while sitting still keep averaging the gyro bias and "up" (at rest the accelerometer points up)
        if abs(a - G) < 0.15 * G:
            if self.bias is None:
                self.bias, self.up = gyro, accel
            self.bias = blend(self.bias, gyro, 0.02)
            self.up = blend(self.up, accel, 0.02)
        if altitude is not None:
            self.ground = altitude if self.ground is None else self.ground + 0.1 * (altitude - self.ground)

        ready = self.bias is not None and t - self.t_start > CALIBRATION_TIME
        if ready and (self.bench or a > LAUNCH_G * G):
            self.launch_count += 1
            self.speed += (accel[0] - G) * dt           # so we don't lose the speed gained while detecting launch
            if self.bench or self.launch_count >= LAUNCH_SAMPLES:
                self.phase = "flight"
                self.t_launch = t
                n = length(self.up)
                self.up = tuple(c / n for c in self.up)
        else:
            self.launch_count = 0
            self.speed = 0.0

    def _fly(self, t, dt, gyro, accel, altitude):
        t_flight = t - self.t_launch
        rate = tuple(g - b for g, b in zip(gyro, self.bias))        # deg/s: (roll, pitch, yaw)
        w = tuple(math.radians(r) for r in rate)

        # the rocket turned by |w|*dt about w, so from the rocket's point of view
        # "up" turned by the same angle the other way
        w_size = length(w)
        if w_size > 1e-9:
            self.up = rotate(self.up, tuple(c / w_size for c in w), -w_size * dt)
        self.roll_angle = clamp(self.roll_angle + rate[0] * dt, 360.0)
        self.tilt = math.degrees(math.acos(clamp(self.up[0], 1.0)))

        # speed is the acceleration along the nose added up, minus gravity
        self.speed = max(0.0, self.speed + (accel[0] - G * self.up[0]) * dt)

        if altitude is not None and self.ground is not None:
            self.alt += 0.3 * ((altitude - self.ground) - self.alt)
            self.max_alt = max(self.max_alt, self.alt)
        past_apogee = t_flight > MIN_FLIGHT_TIME and self.alt < self.max_alt - APOGEE_DROP
        if not self.bench and (past_apogee or t_flight > BACKUP_DEPLOY):
            self.phase = "descent"
            self.t_deploy = t
            return

        if t_flight < CONTROL_DELAY and not self.bench:
            return
        speed = V_REF if self.bench else max(self.speed, V_MIN)
        scale = (V_REF / speed) ** SPEED_POWER
        tilt_pitch = math.degrees(-self.up[2])          # how far to rotate about y to be upright
        tilt_yaw = math.degrees(self.up[1])             # same about z
        pitch = scale * (KP * tilt_pitch - KD * rate[1])
        yaw = scale * (KP * tilt_yaw - KD * rate[2])
        # roll works the same way: hold the roll angle the rocket had at launch
        roll = scale * (-KROLL * rate[0] - KROLL_ANGLE * self.roll_angle)
        self.fins = {
            "blue":   clamp(-pitch - roll, MAX_FIN),
            "green":  clamp(+pitch - roll, MAX_FIN),
            "orange": clamp(-yaw - roll, MAX_FIN),
            "yellow": clamp(+yaw - roll, MAX_FIN),
        }


def main():
    bench = "bench" in sys.argv

    import bme280
    import smbus2
    from gpiozero import Device, Servo
    from gpiozero.pins.lgpio import LGPIOFactory
    from mpu6050 import mpu6050

    Device.pin_factory = LGPIOFactory()
    fin_servos = {name: Servo(pin) for name, pin in FIN_PINS.items()}
    nosecone = Servo(NOSECONE_PIN)
    nosecone.value = NOSECONE_LOCKED

    def set_fins(angles):
        for name, servo in fin_servos.items():
            value = FIN_SIGN[name] * (angles[name] + FIN_TRIM[name]) / DEG_PER_UNIT
            servo.value = clamp(value, 1.0)

    set_fins(dict.fromkeys(FIN_ORDER, 0.0))

    imu = mpu6050(0x68)
    imu.set_accel_range(imu.ACCEL_RANGE_16G)      # the default +-2 g would max out at lift-off
    imu.set_gyro_range(imu.GYRO_RANGE_2000DEG)
    bus = smbus2.SMBus(1)
    baro_cal = bme280.load_calibration_params(bus, 0x76)

    def read_altitude():
        p = bme280.sample(bus, 0x76, baro_cal).pressure
        return 44330 * (1 - (p / 1013.25) ** (1 / 5.255))

    log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(log_dir, exist_ok=True)
    log_path = os.path.join(log_dir, time.strftime("flight_%Y%m%d_%H%M%S.csv"))
    log_file = open(log_path, "w", newline="")
    log = csv.writer(log_file)
    log.writerow(["t", "phase", "tilt", "roll_rate", "pitch_rate", "yaw_rate",
                  "speed", "alt", *FIN_ORDER])

    flight = Flight(bench)
    gyro = accel = (0.0, 0.0, 0.0)
    deployed = False
    last_phase = None
    n = 0
    print(f"Logging to {log_path}")
    print("Keep the rocket still for calibration...")

    try:
        while True:
            t = time.monotonic()
            altitude = None
            try:
                accel = to_body(imu.get_accel_data(), AXES)
                gyro = to_body(imu.get_gyro_data(), AXES)
                if n % BARO_EVERY == 0:
                    altitude = read_altitude()
            except OSError:
                pass                                   # read failed, just reuse the last values

            fins, deploy = flight.step(t, gyro, accel, altitude)
            set_fins(fins)
            if deploy and not deployed:
                nosecone.value = NOSECONE_OPEN
                deployed = True

            log.writerow([f"{t:.3f}", flight.phase, f"{flight.tilt:.2f}",
                          *(f"{g - b:.1f}" for g, b in zip(gyro, flight.bias or (0, 0, 0))),
                          f"{flight.speed:.1f}", f"{flight.alt:.1f}",
                          *(f"{fins[name]:.2f}" for name in FIN_ORDER)])
            if flight.phase != last_phase:
                last_phase = flight.phase
                log_file.flush()
                print("Phase:", flight.phase)
            if bench and n % 10 == 0:
                print(f"tilt {flight.tilt:5.1f} deg   " +
                      "  ".join(f"{name} {fins[name]:+5.1f}" for name in FIN_ORDER))
            if n % LOOP_HZ == 0:
                log_file.flush()
            if deployed and t - flight.t_deploy > LOG_AFTER_DEPLOY:
                break

            n += 1
            time.sleep(max(0.0, 1.0 / LOOP_HZ - (time.monotonic() - t)))
    except KeyboardInterrupt:
        print("\nProgram halted.")
    finally:
        set_fins(dict.fromkeys(FIN_ORDER, 0.0))
        time.sleep(0.5)
        log_file.close()
        for servo in [*fin_servos.values(), nosecone]:
            servo.detach()
            servo.close()


if __name__ == "__main__":
    main()
