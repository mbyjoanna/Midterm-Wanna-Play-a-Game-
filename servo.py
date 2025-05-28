import machine
from machine import I2C, Pin, Timer
import servo
import time
import math

# Initialize servos
servo0 = servo.Servo(0)  # Servo on PWM channel 0
servo1 = servo.Servo(1)  # Servo on PWM channel 1
servo2 = servo.Servo(2)  # Servo on PWM channel 2

# Variables to store current positions
current_angle0 = 0
current_angle1 = 0
current_angle2 = 0

# Variables for target positions
target_angle0 = 0
target_angle1 = 0
target_angle2 = 0

# MOVEMENT CONTROL - SIGNIFICANTLY CHANGED
is_moving = False
movement_timer = None
total_movement_time = 10000  # Increased total movement time to 10 seconds for slower movement
update_interval = 20  # Reduced update interval to 20ms for smoother motion
movement_steps = 0  # Will be calculated based on largest angle change
current_step = 0    # Track which step we're on

def move_servos(angle0, angle1, angle2):
    """
    Immediately move servos to specified angles.
    """
    servo0.write(angle0)
    servo1.write(angle1)
    servo2.write(angle2)

def timer_callback(timer):
    """
    This function is called by the timer at regular intervals.
    It moves the servos gradually along a predetermined path.
    """
    global current_angle0, current_angle1, current_angle2, is_moving, current_step
    
    # Check if we've completed all steps
    if current_step >= movement_steps:
        # We've reached the target, stop moving
        is_moving = False
        timer.deinit()  # Stop the timer
        
        # Set exact target angles
        move_servos(target_angle0, target_angle1, target_angle2)
        current_angle0 = target_angle0
        current_angle1 = target_angle1
        current_angle2 = target_angle2
        print("Movement complete.")
        return
    
    # Calculate new intermediate positions based on linear interpolation
    fraction = (current_step + 1) / movement_steps
    
    # Compute new angles based on the step
    new_angle0 = start_angle0 + (target_angle0 - start_angle0) * fraction
    new_angle1 = start_angle1 + (target_angle1 - start_angle1) * fraction
    new_angle2 = start_angle2 + (target_angle2 - start_angle2) * fraction
    
    # Move servos to new positions
    move_servos(new_angle0, new_angle1, new_angle2)
    
    # Update current angles
    current_angle0 = new_angle0
    current_angle1 = new_angle1
    current_angle2 = new_angle2
    
    # Increment step
    current_step += 1

def start_slow_movement(new_angle0, new_angle1, new_angle2):
    """
    Start slow movement to new target angles.
    Uses linear interpolation over fixed time period.
    """
    global target_angle0, target_angle1, target_angle2, is_moving, movement_timer
    global start_angle0, start_angle1, start_angle2, movement_steps, current_step
    
    # Store starting positions
    start_angle0 = current_angle0
    start_angle1 = current_angle1
    start_angle2 = current_angle2
    
    # Set new targets
    target_angle0 = new_angle0
    target_angle1 = new_angle1
    target_angle2 = new_angle2
    
    # Calculate number of steps based on desired time and update interval
    movement_steps = int(total_movement_time / update_interval)
    current_step = 0
    
    print(f"Starting {total_movement_time/1000:.1f}-second movement to: {new_angle0:.1f}, {new_angle1:.1f}, {new_angle2:.1f}")
    print(f"Movement will take exactly {total_movement_time/1000:.1f} seconds")
    print(f"Using {movement_steps} steps with {update_interval}ms interval")
    
    # If already moving, stop the current movement
    if is_moving and movement_timer is not None:
        movement_timer.deinit()
    
    # Start new movement
    is_moving = True
    movement_timer = Timer(-1)  # Use any available timer
    movement_timer.init(period=update_interval, mode=Timer.PERIODIC, callback=timer_callback)

def move_to_position(x, y, link1_length, link2_length):
    """
    Move the robot arm to a position (x,y) using inverse kinematics.
    Uses timer for slow, controlled movement over exactly 10 seconds.
    
    Parameters:
    - x, y: Target position coordinates
    - link1_length, link2_length: Lengths of the two arm links
    
    Returns:
    - True if the position is reachable, False otherwise
    """
    import math
    
    # Check if position is within reach
    distance = math.sqrt(x**2 + y**2)
    if distance > (link1_length + link2_length) or distance < abs(link1_length - link2_length):
        print("Position out of reach")
        return False
    
    # Calculate angles using inverse kinematics
    cos_angle2 = (x**2 + y**2 - link1_length**2 - link2_length**2) / (2 * link1_length * link2_length)
    if cos_angle2 > 1 or cos_angle2 < -1:
        print("Position out of reach (trigonometric error)")
        return False
        
    angle2 = math.acos(cos_angle2)  # Elbow angle
    
    # Convert to degrees for servo control
    angle2_deg = math.degrees(angle2)
    
    # Calculate first joint angle
    k1 = link1_length + link2_length * cos_angle2
    k2 = link2_length * math.sin(angle2)
    angle1 = math.atan2(y, x) - math.atan2(k2, k1)
    angle1_deg = math.degrees(angle1)
    
    # Calculate third angle to keep chair upright
    angle3_deg = -(angle1_deg + angle2_deg)
    
    # Start exactly 10-second movement to target
    start_slow_movement(angle1_deg, angle2_deg, angle3_deg)
    
    return True

# Initialize by setting servos to starting position
move_servos(0, 0, 0)
current_angle0 = 0
current_angle1 = 0
current_angle2 = 0

# Example usage:
# To move to a position (x=10, y=5) with arm links of lengths 10 and 8
# move_to_position(10, 5, 10, 8)
