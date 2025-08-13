from controller import Robot, DistanceSensor, LED, Motor

TIME_STEP = 32
MAX_SPEED = 6.28

# Sensor configuration
DISTANCE_SENSORS_NUMBER = 8
GROUND_SENSORS_NUMBER = 3
LEDS_NUMBER = 10

DISTANCE_SENSOR_NAMES = [f'ps{i}' for i in range(DISTANCE_SENSORS_NUMBER)]
GROUND_SENSOR_NAMES = [f'gs{i}' for i in range(GROUND_SENSORS_NUMBER)]
LED_NAMES = [f'led{i}' for i in range(LEDS_NUMBER)]

# Braitenberg weights & offsets (left, right)
weights = [
    [-1.3, -1.0], [-1.3, -1.0], [-0.5, 0.5], [0.0, 0.0],
    [0.0, 0.0], [0.05, -0.5], [-0.75, 0], [-0.75, 0]
]
offsets = [0.5 * MAX_SPEED, 0.5 * MAX_SPEED]

# Init robot
robot = Robot()

# Motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')
left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))
left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

# Distance sensors
distance_sensors = []
for name in DISTANCE_SENSOR_NAMES:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    distance_sensors.append(sensor)

# Ground sensors
ground_sensors = []
for name in GROUND_SENSOR_NAMES:
    try:
        sensor = robot.getDevice(name)
        sensor.enable(TIME_STEP)
        ground_sensors.append(sensor)
    except:
        ground_sensors.append(None)

# LEDs
leds = []
for name in LED_NAMES:
    leds.append(robot.getDevice(name))

# Actuator state
speeds = [0.0, 0.0]
led_states = [False] * LEDS_NUMBER

# Utilities
def read_distance_sensors():
    values = [s.getValue() / 4096 for s in distance_sensors]  # scale to [0,1]
    return values

def read_ground_sensors():
    values = []
    for s in ground_sensors:
        if s:
            values.append(s.getValue())
        else:
            values.append(1000.0)  # no cliff
    return values

def cliff_detected(gs_values):
    for v in gs_values:
        if v < 500.0:
            return True
    return False

def set_leds():
    for i in range(LEDS_NUMBER):
        leds[i].set(1 if led_states[i] else 0)

def step():
    return robot.step(TIME_STEP) != -1

def go_backwards():
    left_motor.setVelocity(-MAX_SPEED)
    right_motor.setVelocity(-MAX_SPEED)
    wait(0.2)

def turn_left():
    left_motor.setVelocity(-MAX_SPEED)
    right_motor.setVelocity(MAX_SPEED)
    wait(0.2)

def wait(seconds):
    start_time = robot.getTime()
    while robot.getTime() < start_time + seconds:
        robot.step(TIME_STEP)

# Main loop
print("Braitenberg Python controller started.")

counter = 0
while step():
    # Blink LEDs in sequence
    counter += 1
    led_states = [False] * LEDS_NUMBER
    led_states[(counter // 10) % LEDS_NUMBER] = True
    set_leds()

    ds_values = read_distance_sensors()
    gs_values = read_ground_sensors()

    if cliff_detected(gs_values):
        go_backwards()
        turn_left()
        continue

    # Braitenberg behavior
    speeds = [0.0, 0.0]
    for i in range(DISTANCE_SENSORS_NUMBER):
        for j in range(2):  # left, right
            speeds[j] += ds_values[i] * weights[i][j]

    for j in range(2):
        speeds[j] = min(MAX_SPEED, max(-MAX_SPEED, offsets[j] + speeds[j] * MAX_SPEED))

    # Apply speeds
    left_motor.setVelocity(speeds[0])
    right_motor.setVelocity(speeds[1])
