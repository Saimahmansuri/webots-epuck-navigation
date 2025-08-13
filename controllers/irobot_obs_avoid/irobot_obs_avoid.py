from controller import Robot, Motor, DistanceSensor, TouchSensor, PositionSensor, LED, Receiver
import random
import math

# Constants
TIME_STEP = 16
MAX_SPEED = 10
HALF_SPEED = 8
WHEEL_RADIUS = 0.5
AXLE_LENGTH = 0.271756
target_position = (-1.31, 0.71)
OBSTACLE_DISTANCE = 0.3  # meters to consider obstacle avoidance
ANGLE_TOLERANCE = 0.1  # radians (~5.7 degrees)

# Device names
BUMPER_NAMES = ["bumper_left", "bumper_right"]
CLIFF_NAMES = ["cliff_left", "cliff_front_left", "cliff_front_right", "cliff_right"]
LED_NAMES = ["led_on", "led_play", "led_step"]

# Initialize robot
robot = Robot()

# GPS
gps = robot.getDevice('gps')
gps.enable(TIME_STEP)

# Motors
left_motor = robot.getDevice("left wheel motor")
right_motor = robot.getDevice("right wheel motor")
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0)
right_motor.setVelocity(0)

# Position sensors
left_encoder = robot.getDevice("left wheel sensor")
right_encoder = robot.getDevice("right wheel sensor")
left_encoder.enable(TIME_STEP)
right_encoder.enable(TIME_STEP)

# LEDs
leds = [robot.getDevice(name) for name in LED_NAMES]

# Bumpers
bumpers = [robot.getDevice(name) for name in BUMPER_NAMES]
for b in bumpers:
    b.enable(TIME_STEP)

# Cliff sensors
cliffs = [robot.getDevice(name) for name in CLIFF_NAMES]
for s in cliffs:
    s.enable(TIME_STEP)

# Receiver (virtual wall detector)
receiver = robot.getDevice("receiver")
receiver.enable(TIME_STEP)

def step():
    return robot.step(TIME_STEP) != -1

def passive_wait(duration):
    start_time = robot.getTime()
    while robot.getTime() < start_time + duration:
        if not step():
            break

def stop():
    left_motor.setVelocity(0)
    right_motor.setVelocity(0)

def go_forward(speed=MAX_SPEED):
    left_motor.setVelocity(speed)
    right_motor.setVelocity(speed)

def go_backward():
    left_motor.setVelocity(-HALF_SPEED)
    right_motor.setVelocity(-HALF_SPEED)

def turn(angle_rad):
    stop()
    l0 = left_encoder.getValue()
    r0 = right_encoder.getValue()
    step()
    sign = 1 if angle_rad >= 0 else -1
    left_motor.setVelocity(sign * HALF_SPEED)
    right_motor.setVelocity(-sign * HALF_SPEED)

    orientation = 0.0
    while step():
        l1 = left_encoder.getValue()
        r1 = right_encoder.getValue()
        dl = (l1 - l0) * WHEEL_RADIUS
        dr = (r1 - r0) * WHEEL_RADIUS
        orientation = sign * (dl - dr) / AXLE_LENGTH
        if orientation >= abs(angle_rad):
            break

    stop()
    step()

def is_bumper_pressed(index):
    return bumpers[index].getValue() != 0

def is_cliff(index):
    return cliffs[index].getValue() < 100.0

def flush_receiver():
    while receiver.getQueueLength() > 0:
        receiver.nextPacket()
        
def get_current_position():
    gps_values = gps.getValues()  # (x, y, z)
    return gps_values[0], gps_values[2]  # (x, z)

def distance_to_target(current, target):
    dx = target[0] - current[0]
    dz = target[1] - current[1]
    return math.hypot(dx, dz)

def angle_to_target(current, target):
    dx = target[0] - current[0]
    dz = target[1] - current[1]
    return math.atan2(dz, dx)

def get_relative_angle(current_angle, target_angle):
    """Returns the smallest difference between two angles"""
    return math.atan2(math.sin(target_angle - current_angle), 
                     math.cos(target_angle - current_angle))

def avoid_obstacle(side):
    """Execute obstacle avoidance maneuver"""
    print(f"{side} obstacle detected")
    go_backward()
    passive_wait(0.5)
    if side == "Left":
        turn(random.uniform(0.2, 1.0) * math.pi)
    else:
        turn(-random.uniform(0.2, 1.0) * math.pi)

# Main loop
print("Python controller of the iRobot Create robot started")
leds[0].set(1)
passive_wait(0.5)

while step():
    flush_receiver()
    
    # Get current position and distance to target
    current_pos = get_current_position()
    dist = distance_to_target(current_pos, target_position)
    
    if dist < 0.1:
        print('Destination reached.')
        stop()
        break
    
    # Obstacle detection (highest priority)
    if receiver.getQueueLength() > 0:
        print("Virtual wall detected")
        turn(math.pi)
    elif is_bumper_pressed(0) or is_cliff(0) or is_cliff(1):
        avoid_obstacle("Left")
    elif is_bumper_pressed(1) or is_cliff(2) or is_cliff(3):
        avoid_obstacle("Right")
    else:
        # Navigation to target
        angle = angle_to_target(current_pos, target_position)
        rel_angle = get_relative_angle(0, angle)  # Relative to current forward
        
        # Adjust steering based on angle to target
        if abs(rel_angle) > ANGLE_TOLERANCE:
            # Proportional control for smoother turns
            turn_speed = min(MAX_SPEED, HALF_SPEED + abs(rel_angle) * 5)
            
            if rel_angle > 0:
                # Turn left
                left_motor.setVelocity(turn_speed * 0.3)
                right_motor.setVelocity(turn_speed)
            else:
                # Turn right
                left_motor.setVelocity(turn_speed)
                right_motor.setVelocity(turn_speed * 0.3)
        else:
            # Go straight with speed proportional to distance
            speed = min(MAX_SPEED, HALF_SPEED + dist * 2)
            go_forward(speed)