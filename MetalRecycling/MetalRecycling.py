import RPi.GPIO as GPIO
import time

# GPIO pin configuration for stepper motors
STEP_PIN_1         = 21  # Pin connected to the STEP input of the TB6600
DIR_PIN_1          = 20  # Pin connected to the DIR input of the TB6600
LIMIT_SWITCH_PIN_1 = 16  # Pin connected to the limit switch

STEP_PIN_2         = 19  # Pin connected to the STEP input of the TB6600
DIR_PIN_2          = 26  # Pin connected to the DIR input of the TB6600
LIMIT_SWITCH_PIN_2 = 12  # Pin connected to the limit switch

# GPIO pin configuration for servo motors
SERVO_PIN_1 = 13  # Pin connected to the signal input of servo 1
SERVO_PIN_2 = 6   # Pin connected to the signal input of servo 2

# Conveyor distances
MIN_ST_POS = 1000 # Below this value, servo 1 is activated
MAX_ST_POS = 3000 # Above this value, servo 2 is activated
Z_Axis     = 750  # Distance the Z-axis must travel to reach the conveyor

# Speed settings
NORMAL_SPEED_DELAY      = 0.001  # Pulse duration for normal speed
CALIBRATION_SPEED_DELAY = 0.005  # Pulse duration for calibration speed
conveyor_speed          = 3      # Adjust this value to match the speed of your conveyor belt

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Stepper motor GPIO setup
GPIO.setup(STEP_PIN_1, GPIO.OUT)
GPIO.setup(DIR_PIN_1, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(STEP_PIN_2, GPIO.OUT)
GPIO.setup(DIR_PIN_2, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN_2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Servo motor GPIO setup
GPIO.setup(SERVO_PIN_1, GPIO.OUT)
GPIO.setup(SERVO_PIN_2, GPIO.OUT)

# Setup PWM on the servo pins
pwm1 = GPIO.PWM(SERVO_PIN_1, 50)  # 50Hz PWM frequency for servo 1
pwm2 = GPIO.PWM(SERVO_PIN_2, 50)  # 50Hz PWM frequency for servo 2
pwm1.start(0)
pwm2.start(0)

# Variables
current_position_1   = 0
current_position_2   = 0
steps_per_revolution = 200  # Number of steps for a full revolution (e.g., 1.8 degrees/step)

def calculate_delay(speed):
    """Calculate the wait time depending on the speed of the conveyor belt."""
    base_delay = 10  # Base wait time in seconds at a standard speed
    delay = base_delay / speed  # Adjust the wait time based on the speed
    return delay

def step_motor(step_pin, dir_pin, steps, direction, speed_delay):
    """Control the stepper motor."""
    GPIO.output(dir_pin, direction)
    for _ in range(steps):
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(speed_delay)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(speed_delay)

def calibrate_motor(step_pin, dir_pin, limit_switch_pin):
    """Calibrate the motor by moving to the 0 position."""
    print("Calibration started...")
    GPIO.output(dir_pin, GPIO.LOW)  # Set direction 
    while GPIO.input(limit_switch_pin) == GPIO.LOW:
        GPIO.output(step_pin, GPIO.HIGH)
        time.sleep(CALIBRATION_SPEED_DELAY)
        GPIO.output(step_pin, GPIO.LOW)
        time.sleep(CALIBRATION_SPEED_DELAY)
    print("Limit switch reached. 0 position set.")

def move_to_position(step_pin, dir_pin, target_position, current_position):
    """Move the motor to the desired position."""
    steps_to_move = target_position - current_position
    direction = GPIO.HIGH if steps_to_move > 0 else GPIO.LOW
    steps_to_move = abs(steps_to_move)

    print(f"Moving from position {current_position} to {target_position}...")
    step_motor(step_pin, dir_pin, steps_to_move, direction, NORMAL_SPEED_DELAY)
    print(f"Position reached: {target_position}")
    return target_position

def set_servo_angle(pwm, angle):
    """Set the angle of a servo."""
    duty_cycle = 2 + (angle / 18)  # Calculate the duty cycle for the desired angle
    GPIO.output(SERVO_PIN_1, True)
    pwm.ChangeDutyCycle(duty_cycle)
    time.sleep(0.5)
    GPIO.output(SERVO_PIN_1, False)
    pwm.ChangeDutyCycle(0)

try:
    calibrate_motor(STEP_PIN_1, DIR_PIN_1, LIMIT_SWITCH_PIN_1)
    calibrate_motor(STEP_PIN_2, DIR_PIN_2, LIMIT_SWITCH_PIN_2)
    current_position_1 = 0
    current_position_2 = 0

    while True:
        user_input = input("Enter a position: ")
        try:
            x_position = float(user_input)

            if x_position < MIN_ST_POS:
                delay = calculate_delay(conveyor_speed)
                print(f"Waiting for {delay:.2f} seconds depending on the speed of the conveyor belt...")
                time.sleep(delay)
                print("Servo 1 movement started...")
                set_servo_angle(pwm1, 170)
                time.sleep(3.5)
                set_servo_angle(pwm1, 10)

            elif MIN_ST_POS <= x_position <= MAX_ST_POS:
                current_position_1 = move_to_position(STEP_PIN_1, DIR_PIN_1, int(x_position), current_position_1)
                delay = calculate_delay(conveyor_speed)
                print(f"Waiting for {delay:.2f} seconds depending on the speed of the conveyor belt...")
                time.sleep(delay)
                current_position_2 = move_to_position(STEP_PIN_2, DIR_PIN_2, current_position_2 + Z_Axis, current_position_2)
                # Code to activate solenoid
                time.sleep(1.5)
                current_position_2 = move_to_position(STEP_PIN_2, DIR_PIN_2, current_position_2 - Z_Axis, current_position_2)
                # Code to move to the correct bin

            elif x_position > MAX_ST_POS:
                delay = calculate_delay(conveyor_speed)
                print(f"Waiting for {delay:.2f} seconds depending on the speed of the conveyor belt...")
                time.sleep(delay)
                print("Servo 2 movement started...")
                set_servo_angle(pwm2, 10)
                time.sleep(3.5)
                set_servo_angle(pwm2, 170)

            else:
                print("The entered number is not within the defined ranges.")

        except ValueError:
            print("Enter a valid number.")

except KeyboardInterrupt:
    print("\nProgram stopped.")

finally:
    pwm1.stop()
    pwm2.stop()
    GPIO.cleanup()
