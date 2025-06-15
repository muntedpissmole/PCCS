import RPi.GPIO as GPIO
import time
import sys

# Configuration
GPIO_PIN = 26  # BCM GPIO 26 (physical pin 37)
PWM_FREQ = 60  # PWM frequency in Hz
FADE_DURATION = 3.0  # Seconds for each fade (up or down)
STEPS = 100  # Number of steps for smooth fading

pwm = None  # Initialize to avoid NameError

def fade_cycle(pwm):
    # Fade up from 0% to 100%
    for duty in range(STEPS + 1):
        pwm.ChangeDutyCycle(duty)
        time.sleep(FADE_DURATION / STEPS)
    print(f"Faded up to 100% at {time.strftime('%H:%M:%S')}")

    # Fade down from 100% to 0%
    for duty in range(STEPS, -1, -1):
        pwm.ChangeDutyCycle(duty)
        time.sleep(FADE_DURATION / STEPS)
    print(f"Faded down to 0% at {time.strftime('%H:%M:%S')}")

try:
    GPIO.setmode(GPIO.BCM)
    print("GPIO mode set to BCM")

    GPIO.setup(GPIO_PIN, GPIO.OUT)
    print(f"GPIO {GPIO_PIN} set as output")

    pwm = GPIO.PWM(GPIO_PIN, PWM_FREQ)
    print(f"Starting PWM on GPIO {GPIO_PIN} at {PWM_FREQ} Hz")

    pwm.start(0)  # Start at 0% duty cycle
    print("LED strip fading up and down every 6 seconds. Press Ctrl+C to stop.")

    while True:
        fade_cycle(pwm)

except KeyboardInterrupt:
    print("\nStopped by user")
except Exception as e:
    print(f"Error: {e}", file=sys.stderr)
    sys.exit(1)

finally:
    print("Cleaning up GPIO")
    if pwm is not None:
        try:
            pwm.stop()
            print("PWM stopped")
        except Exception as e:
            print(f"Error stopping PWM: {e}", file=sys.stderr)
    try:
        GPIO.cleanup()
        print("GPIO cleaned up")
    except Exception as e:
        print(f"Error cleaning GPIO: {e}", file=sys.stderr)