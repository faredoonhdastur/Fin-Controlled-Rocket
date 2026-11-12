# Test code for the fin and nosecone servos.
# as per the wiring diagram:
# blue fin   -> GPIO 12 (pin 32)
# green fin  -> GPIO 16 (pin 36)
# yellow fin -> GPIO 20 (pin 38)
# orange fin -> GPIO 21 (pin 40)
# nosecone   -> GPIO 26 (pin 37)

from gpiozero import Device, Servo
from gpiozero.pins.lgpio import LGPIOFactory
from time import sleep

Device.pin_factory = LGPIOFactory()

SERVO_PINS = {
    "blue fin": 12,
    "green fin": 16,
    "yellow fin": 20,
    "orange fin": 21,
    "nosecone": 26,
}

STEP_DELAY = 0.5  # seconds between positions

servos = {name: Servo(pin) for name, pin in SERVO_PINS.items()}

try:
    # Start with everything centred.
    for servo in servos.values():
        servo.mid()
    sleep(1)

    while True:
        for name, servo in servos.items():
            print(f"Testing {name} (GPIO {SERVO_PINS[name]})")
            for move in (servo.min, servo.mid, servo.max, servo.mid):
                move()
                sleep(STEP_DELAY)
except KeyboardInterrupt:
    print("\r", end="")
    print("Program halted.")
finally:
    for servo in servos.values():
        servo.mid()
    sleep(0.5)
    for servo in servos.values():
        servo.detach()
        servo.close()
