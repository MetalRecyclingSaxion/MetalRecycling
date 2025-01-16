import RPi.GPIO as GPIO
import time

# GPIO pin configuration for stepper motors
MOTOR1_STEP         = 21  # Pin connected to the STEP input of the TB6600
MOTOR1_DIR          = 20  # Pin connected to the DIR input of the TB6600
LIMIT_SWITCH_PIN_1  = 16  # Pin connected to the limit switch

MOTOR2_STEP         = 19  # Pin connected to the STEP input of the TB6600
MOTOR2_DIR          = 26  # Pin connected to the DIR input of the TB6600
LIMIT_SWITCH_PIN_2  = 12  # Pin connected to the limit switch

# GPIO pin configuration for servo motors
SERVO_PIN_1 = 13  # Pin connected to the signal input of servo 1
SERVO_PIN_2 = 6   # Pin connected to the signal input of servo 2

# Conveyor distances
MIN_ST_POS  = 1000 # Below this value, servo 1 is activated
MAX_ST_POS  = 3000 # Above this value, servo 2 is activated
FLIPPER_DIS = 1    # Distance Flipper from camera 
ROBOT_DIS   = 1.5  # Dstance Robot from camera
Z_Axis1     = 800  # Distance the Z-axis must travel to reach the conveyor
Z_Axis2     = 400  # Distance the Z-axis must travel to pick up object
last_x_pos  = 0    # Last x position used to calculate nu position x-axis

# Speed settings
NORMAL_SPEED_DELAY      = 0.001  # Pulse duration for normal speed
CALIBRATION_SPEED_DELAY = 0.005  # Pulse duration for calibration speed
conveyor_speed          = 3      # Adjust this value to match the speed of your conveyor belt

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Stepper motor GPIO setup
GPIO.setup(MOTOR1_STEP, GPIO.OUT)
GPIO.setup(MOTOR1_DIR, GPIO.OUT)
GPIO.setup(LIMIT_SWITCH_PIN_1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(MOTOR2_STEP, GPIO.OUT)
GPIO.setup(MOTOR2_DIR, GPIO.OUT)
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

def calculate_delay(speed, dis):
    """Calculate the wait time depending on the speed of the conveyor belt."""
    base_delay = 10 / speed  # Adjust the wait time based on the speed
    delay = base_delay * dis
    
    return delay

def move_motors(steps_motor1, steps_motor2, speed=0.001):
    # Richting instellen op basis van het aantal stappen
    GPIO.output(MOTOR1_DIR, GPIO.HIGH if steps_motor1 > 0 else GPIO.LOW)
    GPIO.output(MOTOR2_DIR, GPIO.HIGH if steps_motor2 > 0 else GPIO.LOW)

    # Absolute waarde van stappen nemen
    steps_motor1 = abs(steps_motor1)
    steps_motor2 = abs(steps_motor2)

    # Bereken de totale stappen en de verhoudingen
    max_steps = max(steps_motor1, steps_motor2)
    ratio_motor1 = steps_motor1 / max_steps
    ratio_motor2 = steps_motor2 / max_steps

    step1_count = 0
    step2_count = 0

    for step in range(max_steps):
        # Motor 1 stappen
        if step1_count < steps_motor1 and (step / max_steps) >= step1_count / steps_motor1:
            GPIO.output(MOTOR1_STEP, GPIO.HIGH)
            time.sleep(speed / 2)  # Korte puls
            GPIO.output(MOTOR1_STEP, GPIO.LOW)
            step1_count += 1

        # Motor 2 stappen
        if step2_count < steps_motor2 and (step / max_steps) >= step2_count / steps_motor2:
            GPIO.output(MOTOR2_STEP, GPIO.HIGH)
            time.sleep(speed / 2)  # Korte puls
            GPIO.output(MOTOR2_STEP, GPIO.LOW)
            step2_count += 1

        # Wacht tussen de stappen om snelheid te regelen
        time.sleep(speed / 2)

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

    print(f"Moving to position")
    step_motor(step_pin, dir_pin, steps_to_move, direction, NORMAL_SPEED_DELAY)
    print(f"Position reached")
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
    calibrate_motor(MOTOR1_STEP, MOTOR1_DIR, LIMIT_SWITCH_PIN_1)
    calibrate_motor(MOTOR2_STEP, MOTOR2_DIR, LIMIT_SWITCH_PIN_2)
    current_position_1 = 0
    current_position_2 = 0

    while True:
        user_input = input("Enter a position: ")
        try:
            x_position = float(user_input)

            if x_position < MIN_ST_POS:
                delay = calculate_delay(conveyor_speed, FLIPPER_DIS)
                print(f"Waiting for {delay:.2f} seconds depending on the speed of the conveyor belt...")
                time.sleep(delay)
                print("Servo 1 movement started...")
                set_servo_angle(pwm1, 170)
                time.sleep(3.5)
                set_servo_angle(pwm1, 10)

            elif MIN_ST_POS <= x_position <= MAX_ST_POS:
                #Calculate new x position based on last x position
                new_x_pos = x_position - last_x_pos
                #Move both motor simultaneously to position on top op object
                move_motors(int(new_x_pos),Z_Axis1,speed=0.001)
                #Wait for object to reach robot
                delay = calculate_delay(conveyor_speed, ROBOT_DIS)
                print(f"Waiting for {delay:.2f} seconds depending on the speed of the conveyor belt...")
                time.sleep(delay)
                #Move robot down
                current_position_2 = move_to_position(MOTOR2_STEP, MOTOR2_DIR, current_position_2 + Z_Axis2, current_position_2)
                #Code to activate solenoid
                time.sleep(1.5)
                current_position_2 = move_to_position(MOTOR2_STEP, MOTOR2_DIR, current_position_2 - (Z_Axis1 + Z_Axis2), current_position_2)
                #Set last x position
                last_x_pos = x_position 
                #Code to move to the correct bin

            elif x_position > MAX_ST_POS:
                delay = calculate_delay(conveyor_speed, FLIPPER_DIS)
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

