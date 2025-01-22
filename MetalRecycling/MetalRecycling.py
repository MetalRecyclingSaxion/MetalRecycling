from tkinter import MOVETO
import RPi.GPIO as GPIO
import time

# GPIO pin configuration for stepper motors
Motor1Step         = 21  # Pin connected to the STEP input of the TB6600
Motor1Dir          = 20  # Pin connected to the DIR input of the TB6600
LimitSwitchPin1    = 16  # Pin connected to the limit switch

Motor2Step         = 19  # Pin connected to the STEP input of the TB6600
Motor2Dir          = 26  # Pin connected to the DIR input of the TB6600
LimitSwitchPin2    = 12  # Pin connected to the limit switch

# GPIO pin configuration for servo motors
ServoPin1 = 13  # Pin connected to the signal input of servo 1
ServoPin2 = 6   # Pin connected to the signal input of servo 2

# Emergency stop button pin
EmergencyStopPin = 27  # Pin connected to the emergency stop button

# Conveyor distances
MinStPos   = 500  # Below this value, servo 1 is activated
MaxStPos   = 1500 # Above this value, servo 2 is activated
FlipperDis = 1    # Distance Flipper from camera 
RobotDis   = 1.5  # Distance Robot from camera
ZAxis1     = 800  # Distance the Z-axis must travel to reach the conveyor
ZAxis2     = 400  # Distance the Z-axis must travel to pick up object
LastXPos   = 0    # Last x position used to calculate nu position x-axis

#Coordinates metal bins
CopperBinXPos = 200
SteelBinXPos  = 1700

# Speed settings
NormalSpeedDelay      = 0.001  # Pulse duration for normal speed
CalibrationSpeedDelay = 0.005  # Pulse duration for calibration speed
ConveyorSpeed         = 3      # Adjust this value to match the speed of your conveyor belt

# Setup GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Stepper motor GPIO setup
GPIO.setup(Motor1Step, GPIO.OUT)
GPIO.setup(Motor1Dir, GPIO.OUT)
GPIO.setup(LimitSwitchPin1, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(Motor2Step, GPIO.OUT)
GPIO.setup(Motor2Dir, GPIO.OUT)
GPIO.setup(LimitSwitchPin2, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Servo motor GPIO setup
GPIO.setup(ServoPin1, GPIO.OUT)
GPIO.setup(ServoPin2, GPIO.OUT)

# Emergency stop GPIO setup
GPIO.setup(EmergencyStopPin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Setup PWM on the servo pins
Pwm1 = GPIO.PWM(ServoPin1, 50)  # 50Hz PWM frequency for servo 1
Pwm2 = GPIO.PWM(ServoPin2, 50)  # 50Hz PWM frequency for servo 2
Pwm1.start(0)
Pwm2.start(0)

# Variables
CurrentPosition1   = 0
CurrentPosition2   = 0
StepsPerRevolution = 200  # Number of steps for a full revolution (e.g., 1.8 degrees/step)

def CheckEmergencyStop():
    """Check if the emergency stop button is pressed."""
    if GPIO.input(EmergencyStopPin) == GPIO.LOW:
        print("Emergency stop activated!")
        raise KeyboardInterrupt

def CalculateDelay(speed, dis):
    """Calculate the wait time depending on the speed of the conveyor belt."""
    BaseDelay = 10 / speed  # Adjust the wait time based on the speed
    Delay = BaseDelay * dis
    
    return Delay

def MoveMotors(StepsMotor1, StepsMotor2, Speed=0.001):
    GPIO.output(Motor1Dir, GPIO.HIGH if StepsMotor1 > 0 else GPIO.LOW)
    GPIO.output(Motor2Dir, GPIO.HIGH if StepsMotor2 > 0 else GPIO.LOW)

    StepsMotor1 = abs(StepsMotor1)
    StepsMotor2 = abs(StepsMotor2)

    MaxSteps = max(StepsMotor1, StepsMotor2)
    Step1Count = 0
    Step2Count = 0

    for Step in range(MaxSteps):
        CheckEmergencyStop()

        if Step1Count < StepsMotor1:
            GPIO.output(Motor1Step, GPIO.HIGH)
            time.sleep(Speed / 2)
            GPIO.output(Motor1Step, GPIO.LOW)
            Step1Count += 1

        if Step2Count < StepsMotor2:
            GPIO.output(Motor2Step, GPIO.HIGH)
            time.sleep(Speed / 2)
            GPIO.output(Motor2Step, GPIO.LOW)
            Step2Count += 1

        time.sleep(Speed / 2)

def StepMotor(StepPin, DirPin, Steps, Direction, SpeedDelay):
    """Control the stepper motor."""
    GPIO.output(DirPin, Direction)
    for _ in range(Steps):
        CheckEmergencyStop()
        GPIO.output(StepPin, GPIO.HIGH)
        time.sleep(SpeedDelay)
        GPIO.output(StepPin, GPIO.LOW)
        time.sleep(SpeedDelay)

def CalibrateMotor(StepPin, DirPin, LimitSwitchPin):
    """Calibrate the motor by moving to the 0 position."""
    print("Calibration started...")
    GPIO.output(DirPin, GPIO.LOW)
    while GPIO.input(LimitSwitchPin) == GPIO.LOW:
        CheckEmergencyStop()
        GPIO.output(StepPin, GPIO.HIGH)
        time.sleep(CalibrationSpeedDelay)
        GPIO.output(StepPin, GPIO.LOW)
        time.sleep(CalibrationSpeedDelay)
    print("Limit switch reached. 0 position set.")

def MoveToPosition(StepPin, DirPin, TargetPosition, CurrentPosition):
    """Move the motor to the desired position."""
    StepsToMove = TargetPosition - CurrentPosition
    Direction = GPIO.HIGH if StepsToMove > 0 else GPIO.LOW
    StepsToMove = abs(StepsToMove)

    print(f"Moving to position")
    StepMotor(StepPin, DirPin, StepsToMove, Direction, NormalSpeedDelay)
    print(f"Position reached")
    return TargetPosition

def SetServoAngle(Pwm, Angle):
    """Set the angle of a servo."""
    DutyCycle = 2 + (Angle / 18)
    GPIO.output(ServoPin1, True)
    Pwm.ChangeDutyCycle(DutyCycle)
    time.sleep(0.5)
    GPIO.output(ServoPin1, False)
    Pwm.ChangeDutyCycle(0)

try:
    CalibrateMotor(Motor1Step, Motor1Dir, LimitSwitchPin1)
    CalibrateMotor(Motor2Step, Motor2Dir, LimitSwitchPin2)
    CurrentPosition1 = 0
    CurrentPosition2 = 0
    
    #Code to move to middle of the X-axis after calibrating
    MoveToPosition(Motor1Step, Motor1Dir, ((MaxStPos + MinStPos)//2), CurrentPosition1)

    while True:
        CheckEmergencyStop()
        UserInput = input("Enter a position: ")
        try:
            XPosition = float(UserInput)

            if XPosition < MinStPos:
                Delay = CalculateDelay(ConveyorSpeed, FlipperDis)
                print(f"Waiting for {Delay:.2f} seconds depending on the speed of the conveyor belt...")
                time.sleep(Delay)
                CheckEmergencyStop()
                print("Servo 1 movement started...")
                SetServoAngle(Pwm1, 170)
                time.sleep(3.5)
                SetServoAngle(Pwm1, 10)

            elif MinStPos <= XPosition <= MaxStPos:
                # Calculate new x position based on last x position
                NewXPos = XPosition - LastXPos
                # Move both motors simultaneously to position on top of object
                MoveMotors(int(NewXPos), ZAxis1, Speed=0.001)
                # Wait for object to reach robot
                Delay = CalculateDelay(ConveyorSpeed, RobotDis)
                print(f"Waiting for {Delay:.2f} seconds depending on the speed of the conveyor belt...")
                time.sleep(Delay)
                # Move robot down
                CurrentPosition2 = MoveToPosition(Motor2Step, Motor2Dir, CurrentPosition2 + ZAxis2, CurrentPosition2)
                # Code to activate solenoid
                time.sleep(1.5)
                CurrentPosition2 = MoveToPosition(Motor2Step, Motor2Dir, CurrentPosition2 - (ZAxis1 + ZAxis2), CurrentPosition2) 
                # Code to move to the correct bin
                MetalType = input("Enter a metal type: ")
                
                if MetalType == 'Copper' :
                    XPosBin = CopperBinXPos
                    
                elif MetalType == 'Steel' :
                    XPosBin = SteelBinXPos
                    
                #Move X-axis to correct bin
                LastXPos = MoveToPosition(Motor1Step, Motor1Dir, XPosBin, XPosition)
                    
                  

            elif XPosition > MaxStPos:
                Delay = CalculateDelay(ConveyorSpeed, FlipperDis)
                print(f"Waiting for {Delay:.2f} seconds depending on the speed of the conveyor belt...")
                time.sleep(Delay)
                CheckEmergencyStop()
                print("Servo 2 movement started...")
                SetServoAngle(Pwm2, 10)
                time.sleep(3.5)
                SetServoAngle(Pwm2, 170)

            else:
                print("The entered number is not within the defined ranges.")

        except ValueError:
            print("Enter a valid number.")

except KeyboardInterrupt:
    print("\nProgram stopped.")

finally:
    Pwm1.stop()
    Pwm2.stop()
    GPIO.cleanup()
