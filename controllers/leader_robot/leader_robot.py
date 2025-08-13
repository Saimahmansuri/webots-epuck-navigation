"""
E-puck Robot Controller for Apartment Navigation
Navigates to kitchen (fridge) or living room (sofa) based on user input
Uses GPS for positioning, compass for orientation, and distance sensors for obstacle avoidance
"""

from controller import Robot
import math
import json
import time
import os

# Create robot instance
print("Creating robot instance...")
robot = Robot()
print("✓ Robot instance created successfully")

# Get timestep for simulation
timestep = int(robot.getBasicTimeStep())
print(f"✓ Timestep set to: {timestep} ms")

# Initialize devices
print("Initializing robot devices...")

# Motors
print("Setting up motors...")
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

if left_motor is None:
    print("❌ ERROR: Could not find 'left wheel motor' device")
else:
    print("✓ Left wheel motor found")
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

if right_motor is None:
    print("❌ ERROR: Could not find 'right wheel motor' device")
else:
    print("✓ Right wheel motor found")
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)

# Distance sensors (e-puck has 8 distance sensors)
print("Setting up distance sensors...")
distance_sensors = []
sensor_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for name in sensor_names:
    sensor = robot.getDevice(name)
    if sensor is None:
        print(f"❌ WARNING: Could not find distance sensor '{name}'")
    else:
        sensor.enable(timestep)
        distance_sensors.append(sensor)
        print(f"✓ Distance sensor '{name}' enabled")

print(f"✓ {len(distance_sensors)} distance sensors initialized")

# GPS for position tracking
print("Setting up GPS...")
gps = robot.getDevice('gps')
if gps is None:
    print("❌ ERROR: Could not find 'gps' device")
else:
    gps.enable(timestep)
    print("✓ GPS enabled")

# Compass for orientation
print("Setting up compass...")
compass = robot.getDevice('compass')
if compass is None:
    print("❌ ERROR: Could not find 'compass' device")
else:
    compass.enable(timestep)
    print("✓ Compass enabled")

print("Device initialization complete!")

# Target coordinates
KITCHEN_WAYPOINT = (-2.6, -5.9, 0)  # Intermediate point before kitchen
KITCHEN_TARGET = (-0.52, -0.5, 0)  # Fridge position (final destination)
LIVING_ROOM_WAYPOINT = (-4.517, -6.0805, 0)  # Intermediate point before living room
LIVING_ROOM_TARGET = (-7.0533, -0.8042, 0)  # Sofa position (final destination)

# *** CHANGE DESTINATION HERE ***
# Set DESTINATION to 1 for Kitchen or 2 for Living Room
DESTINATION = 1  # <-- Change this value: 1=Kitchen, 2=Living Room

# Robot parameters
MAX_SPEED = 4.5   # Moderate speed for good navigation
DISTANCE_THRESHOLD = 0.7  # Distance to consider target reached (increased further)
SLOW_DOWN_THRESHOLD = 1.5  # Distance to start slowing down
OBSTACLE_THRESHOLD = 80.0  # Distance sensor threshold for obstacles

# Communication with blind robot
# Use the main project directory (go up from controllers/leader_robot/ to project root)
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
COMMUNICATION_FILE = os.path.join(PROJECT_DIR, "robot_communication.json")
print(f"📁 Communication file path: {COMMUNICATION_FILE}")

class Navigation:
    def __init__(self):
        self.target = None
        self.final_destination = None
        self.waypoint = None
        self.state = "WAITING_INPUT"  # WAITING_INPUT, NAVIGATING_TO_WAYPOINT, NAVIGATING_TO_DESTINATION, REACHED
        self.last_position = [0, 0, 0]
        self.stuck_counter = 0
        self.stuck_threshold = 50  # Steps before considering robot stuck
        self.escape_mode = False
        self.escape_counter = 0
        self.escape_attempts = 0
        self.max_escape_attempts = 3
        self.target_reached_counter = 0
        self.target_reached_threshold = 2  # Steps to confirm target reached (reduced)
        self.waypoint_reached = False
        self.last_command_sent = None
        self.command_send_interval = 0.1  # Send commands every 100ms
        self.path_history = []  # Record path for blind robot to follow
        self.path_record_interval = 2  # Record position every 2 steps (very frequent breadcrumbs)
        self.path_step_counter = 0  # Count steps for path recording
        self.waiting_for_follower = False
        self.follower_wait_threshold = 1.8  # Wait if blind robot is more than 1.8m away
        self.follower_resume_threshold = 1.0  # Resume when blind robot is within 1.0m
        self.ideal_follower_distance = 0.7  # Ideal distance for blind robot to maintain (60-80cm is good)
        
    def set_target(self, target):
        self.target = target
        self.state = "NAVIGATING"
        print(f"Target set to: ({target[0]:.3f}, {target[1]:.3f}, {target[2]:.3f})")
        
    def set_kitchen_destination(self, waypoint, final_destination):
        """Set kitchen navigation with waypoint system"""
        self.waypoint = waypoint
        self.final_destination = final_destination
        self.target = waypoint  # Start with waypoint
        self.waypoint_reached = False
        self.state = "NAVIGATING_TO_WAYPOINT"
        print(f"🗺️ Kitchen navigation started!")
        print(f"  Waypoint: ({waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f})")
        print(f"  Final destination: ({final_destination[0]:.3f}, {final_destination[1]:.3f}, {final_destination[2]:.3f})")
        print(f"🎯 First target: Waypoint ({waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f})")
        
    def set_living_room_destination(self, waypoint, final_destination):
        """Set living room navigation with waypoint system"""
        self.waypoint = waypoint
        self.final_destination = final_destination
        self.target = waypoint  # Start with waypoint
        self.waypoint_reached = False
        self.state = "NAVIGATING_TO_WAYPOINT"
        print(f"🗺️ Living room navigation started!")
        print(f"  Waypoint: ({waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f})")
        print(f"  Final destination: ({final_destination[0]:.3f}, {final_destination[1]:.3f}, {final_destination[2]:.3f})")
        print(f"🎯 First target: Waypoint ({waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f})")
        
    def switch_to_final_destination(self):
        """Switch from waypoint to final destination"""
        if self.final_destination is not None:
            self.target = self.final_destination
            self.waypoint_reached = True
            self.state = "NAVIGATING_TO_DESTINATION"
            self.target_reached_counter = 0  # Reset counter for final destination
            print(f"✅ Waypoint reached! Switching to final destination...")
            print(f"🎯 New target: Final destination ({self.final_destination[0]:.3f}, {self.final_destination[1]:.3f}, {self.final_destination[2]:.3f})")
            
    def get_blind_robot_position(self):
        """Read blind robot position from communication file"""
        try:
            if os.path.exists(COMMUNICATION_FILE):
                # Try multiple times with small delay if file is locked
                for attempt in range(3):
                    try:
                        with open(COMMUNICATION_FILE, 'r') as f:
                            content = f.read().strip()
                            if content:
                                data = json.loads(content)
                                blind_pos_data = data.get('blind_robot_position', None)
                                if blind_pos_data:
                                    return [blind_pos_data['x'], blind_pos_data['y'], blind_pos_data['z']]
                        break
                    except (json.JSONDecodeError, IOError) as e:
                        if attempt < 2:  # Try again on first two attempts
                            time.sleep(0.001)  # 1ms delay
                            continue
                        else:
                            if robot.getTime() % 5.0 < timestep / 1000.0:  # Print error occasionally
                                print(f"📁 File access issue: {type(e).__name__}")
            return None
        except Exception as e:
            return None
            
    def calculate_distance_to_blind_robot(self, leader_pos, blind_pos):
        """Calculate distance between leader and blind robot"""
        if leader_pos is None or blind_pos is None:
            return float('inf')
        
        dx = leader_pos[0] - blind_pos[0]
        dy = leader_pos[1] - blind_pos[1]
        return math.sqrt(dx*dx + dy*dy)
        
    def should_wait_for_follower(self, leader_position):
        """Check if leader should wait for blind robot to catch up"""
        blind_position = self.get_blind_robot_position()
        if blind_position is None:
            return False  # No blind robot position available
            
        distance_to_blind = self.calculate_distance_to_blind_robot(leader_position, blind_position)
        
        # State machine for waiting
        if not self.waiting_for_follower:
            # Check if we should start waiting
            if distance_to_blind > self.follower_wait_threshold:
                self.waiting_for_follower = True
                print(f"⏸️ Leader waiting for blind robot (distance: {distance_to_blind:.1f}m > {self.follower_wait_threshold:.1f}m)")
                return True
        else:
            # Already waiting, check if we can resume
            if distance_to_blind <= self.follower_resume_threshold:
                self.waiting_for_follower = False
                print(f"▶️ Leader resuming navigation (blind robot caught up: {distance_to_blind:.1f}m <= {self.follower_resume_threshold:.1f}m)")
                return False
            else:
                # Still waiting
                return True
                
        return False
            
    def send_command_to_blind_robot(self, left_speed, right_speed, current_state="navigating"):
        """Send movement command to blind robot via shared file"""
        try:
            # Determine command based on motor speeds
            command = "stop"
            speed_modifier = 1.0
            
            # Calculate average speed for speed modifier
            avg_speed = (abs(left_speed) + abs(right_speed)) / 2.0
            if avg_speed > 0:
                speed_modifier = avg_speed / MAX_SPEED
            
            # Determine movement command
            if current_state == "WAITING_FOR_FOLLOWER":
                command = "wait"  # Special command for waiting
            elif abs(left_speed) < 0.1 and abs(right_speed) < 0.1:
                command = "stop"
            elif current_state == "REACHED":
                command = "reached"
            elif left_speed < 0 and right_speed < 0:
                command = "reverse"
            elif abs(left_speed - right_speed) > 1.0:  # Turning
                if left_speed > right_speed:
                    command = "turn_right"
                else:
                    command = "turn_left"
            else:  # Moving forward
                command = "forward"
            
            # Get leader position for blind robot to follow
            leader_position = self.get_position()
            
            # Record path history for blind robot to follow
            self.record_path_point(leader_position)
            
            # Create command data with path history
            command_data = {
                "command": command,
                "speed_modifier": round(speed_modifier, 2),
                "leader_state": current_state,
                "timestamp": time.time(),
                "leader_position": {
                    "x": round(leader_position[0], 3),
                    "y": round(leader_position[1], 3),
                    "z": round(leader_position[2], 3)
                },
                "path_history": self.get_recent_path_history(),
                "leader_speeds": {
                    "left": round(left_speed, 2),
                    "right": round(right_speed, 2)
                }
            }
            
            # Write to communication file with better error handling
            try:
                # Write to temporary file first, then rename (atomic operation)
                temp_file = COMMUNICATION_FILE + ".tmp"
                with open(temp_file, 'w') as f:
                    json.dump(command_data, f)
                    f.flush()
                    os.fsync(f.fileno())  # Force write to disk
                
                # Atomic rename
                if os.path.exists(temp_file):
                    try:
                        os.replace(temp_file, COMMUNICATION_FILE)
                    except:
                        # Fallback if replace fails
                        if os.path.exists(COMMUNICATION_FILE):
                            os.remove(COMMUNICATION_FILE)
                        os.rename(temp_file, COMMUNICATION_FILE)
                        
                    # Log command changes occasionally
                    if self.last_command_sent != command:
                        print(f"📡 Sending to blind robot: {command} (speed: {speed_modifier:.2f}) ✓")
                        self.last_command_sent = command
                        
            except Exception as e:
                if robot.getTime() % 3.0 < timestep / 1000.0:  # Print error occasionally
                    print(f"⚠️ Communication error: {type(e).__name__}")
                # Clean up temp file if it exists
                temp_file = COMMUNICATION_FILE + ".tmp"
                if os.path.exists(temp_file):
                    try:
                        os.remove(temp_file)
                    except:
                        pass
                
        except Exception as e:
            print(f"⚠️ Error in command generation: {e}")
            
    def record_path_point(self, position):
        """Record current position in path history for blind robot"""
        self.path_step_counter += 1
        
        # Only record every N steps to avoid too many points
        if self.path_step_counter % self.path_record_interval == 0:
            path_point = {
                "x": round(position[0], 3),
                "y": round(position[1], 3),
                "z": round(position[2], 3),
                "timestamp": time.time()
            }
            self.path_history.append(path_point)
            
            # Keep only recent path history (last 100 points = ~200 steps with interval 2)
            if len(self.path_history) > 100:
                self.path_history = self.path_history[-100:]
                
    def get_recent_path_history(self):
        """Get recent path history for blind robot (last 30 points)"""
        return self.path_history[-30:] if len(self.path_history) > 0 else []
        
    def get_position(self):
        """Get current robot position from GPS"""
        try:
            position = gps.getValues()
            if position is None or len(position) < 3:
                print("❌ WARNING: GPS reading failed or incomplete")
                return [0, 0, 0]
            return position
        except Exception as e:
            print(f"❌ ERROR reading GPS: {e}")
            return [0, 0, 0]
        
    def get_orientation(self):
        """Get current robot orientation from compass"""
        try:
            compass_values = compass.getValues()
            if compass_values is None or len(compass_values) < 3:
                print("❌ WARNING: Compass reading failed or incomplete")
                return 0
            # Calculate bearing from compass values
            bearing = math.atan2(compass_values[0], compass_values[1])
            return bearing
        except Exception as e:
            print(f"❌ ERROR reading compass: {e}")
            return 0
        
    def calculate_distance_to_target(self, current_pos):
        """Calculate Euclidean distance to target"""
        if self.target is None:
            return float('inf')
        
        dx = self.target[0] - current_pos[0]
        dy = self.target[1] - current_pos[1]
        return math.sqrt(dx*dx + dy*dy)
        
    def calculate_angle_to_target(self, current_pos, current_orientation):
        """Calculate angle to target relative to current orientation"""
        if self.target is None:
            return 0
            
        dx = self.target[0] - current_pos[0]
        dy = self.target[1] - current_pos[1]
        target_angle = math.atan2(dy, dx)
        
        # Normalize angle difference
        angle_diff = target_angle - current_orientation
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff
        
    def detect_obstacles(self):
        """Detect obstacles using distance sensors"""
        sensor_values = [sensor.getValue() for sensor in distance_sensors]
        
        # Lower threshold for more sensitive detection when trapped
        close_threshold = OBSTACLE_THRESHOLD * 0.6 if self.escape_mode else OBSTACLE_THRESHOLD
        very_close_threshold = OBSTACLE_THRESHOLD * 1.5  # For very close obstacles
        
        # Check front sensors (ps0, ps1, ps2, ps6, ps7)
        front_obstacle = (sensor_values[0] > close_threshold or 
                         sensor_values[1] > close_threshold or 
                         sensor_values[2] > close_threshold or 
                         sensor_values[6] > close_threshold or 
                         sensor_values[7] > close_threshold)
        
        # Check for very close front obstacles
        front_very_close = (sensor_values[0] > very_close_threshold or 
                           sensor_values[1] > very_close_threshold or 
                           sensor_values[2] > very_close_threshold or 
                           sensor_values[6] > very_close_threshold or 
                           sensor_values[7] > very_close_threshold)
        
        # Check left side (ps5, ps6, ps7)
        left_obstacle = (sensor_values[5] > close_threshold or 
                        sensor_values[6] > close_threshold or 
                        sensor_values[7] > close_threshold)
        
        # Check right side (ps0, ps1, ps2)
        right_obstacle = (sensor_values[0] > close_threshold or 
                         sensor_values[1] > close_threshold or 
                         sensor_values[2] > close_threshold)
        
        # Check back sensors (ps3, ps4) for backing up
        back_obstacle = (sensor_values[3] > close_threshold or 
                        sensor_values[4] > close_threshold)
        
        return front_obstacle, left_obstacle, right_obstacle, front_very_close, back_obstacle, sensor_values
        
    def navigate(self):
        """Main navigation logic"""
        if self.state not in ["NAVIGATING", "NAVIGATING_TO_WAYPOINT", "NAVIGATING_TO_DESTINATION"] or self.target is None:
            return 0.0, 0.0
        
        # Debug: First time navigation is called
        if not hasattr(self, '_navigation_started'):
            print("✓ Navigation function started - robot is navigating!")
            self._navigation_started = True
            
        # Get current position and orientation
        current_pos = self.get_position()
        current_orientation = self.get_orientation()
        
        # Debug: Check if we're getting valid sensor data
        if not hasattr(self, '_sensor_check_done'):
            print(f"✓ GPS reading: {current_pos}")
            print(f"✓ Compass reading: {current_orientation:.3f} radians")
            self._sensor_check_done = True
        
        # Check if robot is stuck (position hasn't changed much)
        position_change = math.sqrt((current_pos[0] - self.last_position[0])**2 + 
                                   (current_pos[1] - self.last_position[1])**2)
        
        if position_change < 0.01:  # Very small movement
            self.stuck_counter += 1
        else:
            self.stuck_counter = 0
            # Reset escape attempts if robot has been moving normally for a while
            if position_change > 0.05 and not self.escape_mode and self.escape_attempts > 0:
                print("✅ Good movement detected - resetting escape attempt counter")
                self.escape_attempts = 0  # Reset escape attempts on good movement
            self.last_position = current_pos[:]
        
        # Enter escape mode if stuck
        if self.stuck_counter > self.stuck_threshold and not self.escape_mode:
            self.escape_attempts += 1
            if self.escape_attempts <= self.max_escape_attempts:
                print(f"🚨 ROBOT IS STUCK! Entering escape mode (Attempt {self.escape_attempts}/{self.max_escape_attempts})...")
                self.escape_mode = True
                self.escape_counter = 0
            else:
                print("❌ Maximum escape attempts reached. Robot may be permanently stuck.")
                print("Trying aggressive random movement...")
                self.escape_mode = True
                self.escape_counter = 0
        
        # Check follower distance and adapt speed accordingly
        blind_position = self.get_blind_robot_position()
        follower_speed_modifier = 1.0
        is_waiting_for_follower = False
        
        if blind_position is not None:
            distance_to_blind = self.calculate_distance_to_blind_robot(current_pos, blind_position)
            
            if distance_to_blind > self.follower_wait_threshold:
                # Blind robot is too far - COMPLETELY STOP and wait
                print(f"⏸️ LEADER STOPPED - Blind robot too far: {distance_to_blind:.1f}m > {self.follower_wait_threshold:.1f}m")
                is_waiting_for_follower = True
                return 0.0, 0.0
            elif distance_to_blind > self.ideal_follower_distance * 2.0:
                # Blind robot is falling behind significantly - slow down
                follower_speed_modifier = 0.5  # Slow down 
                print(f"🐌 Leader slowing down - Blind robot far: {distance_to_blind:.1f}m")
            elif distance_to_blind > self.ideal_follower_distance * 1.3:
                # Blind robot is getting distant - slow down slightly
                follower_speed_modifier = 0.8  # Slight slowdown
                if robot.getTime() % 2.0 < timestep / 1000.0:  # Print occasionally
                    print(f"🚶 Leader slightly slower - Blind robot: {distance_to_blind:.1f}m")
            elif distance_to_blind < self.ideal_follower_distance * 0.4:
                # Blind robot is very close - speed up to give space
                follower_speed_modifier = 1.2
        
        # Check if target is reached
        distance_to_target = self.calculate_distance_to_target(current_pos)
        if distance_to_target < DISTANCE_THRESHOLD:
            self.target_reached_counter += 1
            
            if self.state == "NAVIGATING_TO_WAYPOINT":
                print(f"🗺️ Near waypoint! Distance: {distance_to_target:.3f} (confirmation {self.target_reached_counter}/{self.target_reached_threshold})")
            else:
                print(f"🎯 Near target! Distance: {distance_to_target:.3f} (confirmation {self.target_reached_counter}/{self.target_reached_threshold})")
            
            if self.target_reached_counter >= self.target_reached_threshold:
                if self.state == "NAVIGATING_TO_WAYPOINT":
                    # Waypoint reached, switch to final destination
                    print(f"✅ Waypoint confirmed reached! Distance: {distance_to_target:.3f}")
                    self.switch_to_final_destination()
                    # Continue navigation to final destination
                else:
                    # Final destination reached
                    self.state = "REACHED"
                    print(f"✅ Final destination confirmed reached! Distance: {distance_to_target:.3f}")
                    return 0.0, 0.0
            else:
                # Move very slowly to fine-tune position
                # Continue with navigation but at very slow speed
                pass
        else:
            # Reset counter if we move away from target
            if self.target_reached_counter > 0:
                print(f"⚠️ Moved away from target, resetting counter")
                self.target_reached_counter = 0
        
        # Detect obstacles
        front_obstacle, left_obstacle, right_obstacle, front_very_close, back_obstacle, sensor_values = self.detect_obstacles()
        
        # Calculate desired direction to target
        angle_to_target = self.calculate_angle_to_target(current_pos, current_orientation)
        
        # Default speeds
        left_speed = MAX_SPEED
        right_speed = MAX_SPEED
        
        # ESCAPE MODE - Smart behavior to get unstuck
        if self.escape_mode:
            self.escape_counter += 1
            print(f"🔄 ESCAPE MODE: Step {self.escape_counter} (Attempt {self.escape_attempts}/{self.max_escape_attempts})")
            
            # Check if we found a clear path during escape
            path_clear = not front_obstacle and not front_very_close
            if path_clear and self.escape_counter > 10:  # Found clear path after some movement
                print("✅ Clear path found! Exiting escape mode...")
                self.escape_mode = False
                self.escape_counter = 0
                self.stuck_counter = 0
                # Continue with normal navigation
                left_speed = MAX_SPEED * 0.5
                right_speed = MAX_SPEED * 0.5
            
            # Different escape strategies based on attempt number
            elif self.escape_attempts == 1:  # First attempt: Back and turn
                if self.escape_counter < 25:  # Back up
                    if not back_obstacle:
                        print("⬅️ Strategy 1: Backing up...")
                        left_speed = -MAX_SPEED * 0.8
                        right_speed = -MAX_SPEED * 0.8
                    else:
                        print("🔄 Can't back up, turning right...")
                        left_speed = MAX_SPEED * 0.8
                        right_speed = -MAX_SPEED * 0.8
                elif self.escape_counter < 60:  # Turn right
                    print("➡️ Strategy 1: Turning right...")
                    left_speed = MAX_SPEED * 0.9
                    right_speed = -MAX_SPEED * 0.9
                else:  # Force exit
                    print("⏰ Strategy 1 timeout, trying next strategy...")
                    self.escape_mode = False
                    self.escape_counter = 0
                    self.stuck_counter = 0
            
            elif self.escape_attempts == 2:  # Second attempt: Back and turn left
                if self.escape_counter < 25:  # Back up
                    if not back_obstacle:
                        print("⬅️ Strategy 2: Backing up...")
                        left_speed = -MAX_SPEED * 0.9
                        right_speed = -MAX_SPEED * 0.9
                    else:
                        print("🔄 Can't back up, turning left...")
                        left_speed = -MAX_SPEED * 0.8
                        right_speed = MAX_SPEED * 0.8
                elif self.escape_counter < 60:  # Turn left
                    print("⬅️ Strategy 2: Turning left...")
                    left_speed = -MAX_SPEED * 0.9
                    right_speed = MAX_SPEED * 0.9
                else:  # Force exit
                    print("⏰ Strategy 2 timeout, trying next strategy...")
                    self.escape_mode = False
                    self.escape_counter = 0
                    self.stuck_counter = 0
            
            elif self.escape_attempts == 3:  # Third attempt: Aggressive zigzag
                if self.escape_counter < 40:
                    if self.escape_counter % 8 < 4:  # Zigzag pattern
                        print("↗️ Strategy 3: Zigzag right...")
                        left_speed = MAX_SPEED * 1.0
                        right_speed = -MAX_SPEED * 0.5
                    else:
                        print("↖️ Strategy 3: Zigzag left...")
                        left_speed = -MAX_SPEED * 0.5
                        right_speed = MAX_SPEED * 1.0
                else:  # Force exit
                    print("⏰ Strategy 3 timeout...")
                    self.escape_mode = False
                    self.escape_counter = 0
                    self.stuck_counter = 0
            
            else:  # Last resort: Random movement
                print("🎲 Last resort: Random movement...")
                import time
                random_factor = (int(time.time() * 1000) % 4)
                if random_factor == 0:
                    left_speed = MAX_SPEED * 1.0
                    right_speed = -MAX_SPEED * 1.0
                elif random_factor == 1:
                    left_speed = -MAX_SPEED * 1.0
                    right_speed = MAX_SPEED * 1.0
                elif random_factor == 2:
                    left_speed = -MAX_SPEED * 0.8
                    right_speed = -MAX_SPEED * 0.8
                else:
                    left_speed = MAX_SPEED * 0.8
                    right_speed = MAX_SPEED * 0.8
                
                if self.escape_counter > 80:  # Force exit after long random movement
                    print("⏰ Random movement timeout, continuing navigation...")
                    self.escape_mode = False
                    self.escape_counter = 0
                    self.stuck_counter = 0
        
        # NORMAL NAVIGATION MODE
        elif front_very_close:
            print("🛑 VERY CLOSE OBSTACLE - Emergency avoidance!")
            # Very aggressive avoidance
            if not back_obstacle:
                # Back up immediately
                left_speed = -MAX_SPEED * 1.0
                right_speed = -MAX_SPEED * 1.0
            else:
                # Spin in place
                left_speed = MAX_SPEED * 1.0
                right_speed = -MAX_SPEED * 1.0
        
        elif front_obstacle:
            print("⚠️ Front obstacle detected - avoiding")
            # Turn away from obstacle
            if not right_obstacle and not left_obstacle:
                # Both sides clear, choose based on target direction
                if angle_to_target > 0:
                    # Turn left
                    left_speed = -MAX_SPEED * 0.4
                    right_speed = MAX_SPEED * 0.4
                else:
                    # Turn right
                    left_speed = MAX_SPEED * 0.4
                    right_speed = -MAX_SPEED * 0.4
            elif not right_obstacle:
                # Turn right
                left_speed = MAX_SPEED * 0.4
                right_speed = -MAX_SPEED * 0.4
            elif not left_obstacle:
                # Turn left
                left_speed = -MAX_SPEED * 0.4
                right_speed = MAX_SPEED * 0.4
            else:
                # Both sides blocked, back up and turn
                if not back_obstacle:
                    left_speed = -MAX_SPEED * 0.3
                    right_speed = -MAX_SPEED * 0.2  # Slight turn while backing
                else:
                    # Completely surrounded, spin
                    left_speed = MAX_SPEED * 0.5
                    right_speed = -MAX_SPEED * 0.5
        else:
            # No front obstacle, navigate toward target
            # Calculate speed modifier based on distance to target
            speed_modifier = 1.0
            if distance_to_target < DISTANCE_THRESHOLD * 0.8:  # Within 80% of threshold
                # Very close to target - move very slowly for precision
                speed_modifier = 0.1
                print(f"🎯 Very close to target - precision mode (modifier: {speed_modifier:.2f})")
            elif distance_to_target < SLOW_DOWN_THRESHOLD:
                # Slow down as we approach the target
                speed_modifier = max(0.2, distance_to_target / SLOW_DOWN_THRESHOLD)
                print(f"🎯 Approaching target - slowing down (distance: {distance_to_target:.3f}, modifier: {speed_modifier:.2f})")
            
            if abs(angle_to_target) > 0.15:  # Need to turn
                if angle_to_target > 0:
                    # Turn left
                    left_speed = MAX_SPEED * 0.4 * speed_modifier
                    right_speed = MAX_SPEED * 0.8 * speed_modifier
                else:
                    # Turn right
                    left_speed = MAX_SPEED * 0.8 * speed_modifier
                    right_speed = MAX_SPEED * 0.4 * speed_modifier
            else:
                # Move straight toward target
                left_speed = MAX_SPEED * 0.8 * speed_modifier  # Good forward speed
                right_speed = MAX_SPEED * 0.8 * speed_modifier  # Good forward speed
        
        # Apply follower distance modifier to maintain formation
        left_speed *= follower_speed_modifier
        right_speed *= follower_speed_modifier
        
        # No global speed reduction - let blind robot adapt to leader's actual speed
        
        # Send command to blind robot with proper state
        current_state_for_blind = self.state
        if self.state == "REACHED":
            current_state_for_blind = "REACHED"
        elif is_waiting_for_follower or (blind_position is not None and self.calculate_distance_to_blind_robot(current_pos, blind_position) > self.follower_wait_threshold):
            current_state_for_blind = "WAITING_FOR_FOLLOWER"
        elif self.state == "NAVIGATING_TO_WAYPOINT":
            current_state_for_blind = "TO_WAYPOINT" 
        elif self.state == "NAVIGATING_TO_DESTINATION":
            current_state_for_blind = "TO_DESTINATION"
        
        self.send_command_to_blind_robot(left_speed, right_speed, current_state_for_blind)
        
        # Print status
        if robot.getTime() % 1.0 < timestep / 1000.0:  # Print every second
            # Add navigation stage info
            if self.waiting_for_follower:
                stage_info = "[⏸️ WAITING FOR FOLLOWER]"
            elif self.state == "NAVIGATING_TO_WAYPOINT":
                stage_info = "[🗺️ Stage 1: To Waypoint]"
            elif self.state == "NAVIGATING_TO_DESTINATION":
                stage_info = "[🎯 Stage 2: To Final Destination]"
            else:
                stage_info = "[🧭 Navigating]"
            
            status = f"{stage_info} Position: ({current_pos[0]:.3f}, {current_pos[1]:.3f}), "
            status += f"Distance to target: {distance_to_target:.3f}, "
            status += f"Angle to target: {math.degrees(angle_to_target):.1f}°"
            
            # Add follower distance info
            blind_position = self.get_blind_robot_position()
            if blind_position is not None:
                distance_to_blind = self.calculate_distance_to_blind_robot(current_pos, blind_position)
                status += f", Follower distance: {distance_to_blind:.1f}m"
            
            if self.escape_mode:
                status += f" [🚨 ESCAPE MODE - Step {self.escape_counter}]"
            elif self.stuck_counter > 10:
                status += f" [⚠️ Movement slow - {self.stuck_counter}/{self.stuck_threshold}]"
            
            print(status)
            
            # Print sensor debug when stuck or in escape mode
            if self.escape_mode or self.stuck_counter > 20:
                sensor_debug = f"Sensors: "
                for i, val in enumerate(sensor_values):
                    sensor_debug += f"ps{i}:{val:.0f} "
                print(f"  🔍 {sensor_debug}")
                print(f"  🧭 Front:{front_obstacle}, Left:{left_obstacle}, Right:{right_obstacle}, Back:{back_obstacle}")
        
        return left_speed, right_speed

# Initialize navigation system
nav = Navigation()

# Get initial user input
print("\n" + "="*50)
print("ROBOT INITIALIZATION PHASE")
print("="*50)
print("Performing initial robot step to enable all devices...")
step_result = robot.step(timestep)  # Initial step to enable devices

if step_result == -1:
    print("❌ ERROR: Robot step failed during initialization")
    exit(1)
else:
    print("✓ Initial robot step completed successfully")

print("Waiting for GPS and sensors to stabilize...")
# Take a few more steps to let sensors stabilize
for i in range(5):
    print(f"  Stabilization step {i+1}/5...")
    step_result = robot.step(timestep)
    if step_result == -1:
        print(f"❌ ERROR: Robot step failed during stabilization step {i+1}")
        break
    print(f"  ✓ Stabilization step {i+1}/5 completed")

print("✓ Robot fully initialized and ready")
print("🤖 Leader robot will send commands to blind robot via:", COMMUNICATION_FILE)

# Initialize/clear communication file for fresh start
print(f"🗂️ Working directory: {os.getcwd()}")
print("🔄 Clearing communication file for fresh start...")

# Remove old communication file if it exists
try:
    if os.path.exists(COMMUNICATION_FILE):
        os.remove(COMMUNICATION_FILE)
        print("✓ Old communication file removed")
except Exception as e:
    print(f"⚠️ Warning: Could not remove old communication file: {e}")

# Create fresh communication file
try:
    initial_command = {
        "command": "stop",
        "speed_modifier": 0.0,
        "leader_state": "INITIALIZING",
        "timestamp": time.time(),
        "leader_position": {"x": 0.0, "y": 0.0, "z": 0.0},
        "path_history": [],
        "leader_speeds": {"left": 0.0, "right": 0.0}
    }
    with open(COMMUNICATION_FILE, 'w') as f:
        json.dump(initial_command, f)
        f.flush()
    
    # Verify initialization
    if os.path.exists(COMMUNICATION_FILE):
        file_size = os.path.getsize(COMMUNICATION_FILE)
        print(f"✓ Fresh communication file created ({file_size} bytes)")
    else:
        print(f"❌ Communication file creation failed")
        
except PermissionError as e:
    print(f"❌ Permission denied creating communication file: {e}")
    print(f"   Check if directory is writable: {os.getcwd()}")
except Exception as e:
    print(f"⚠️ Warning: Could not initialize communication file: {e}")
    print(f"   File path: {COMMUNICATION_FILE}")

print("\n" + "="*50)
print("USER INPUT PHASE")
print("="*50)

# Use the DESTINATION variable set at the top of the file
print("Reading destination from DESTINATION variable...")
print("Available destinations:")
print("1. Kitchen (Fridge)")
print("2. Living Room (Sofa)")
print(f"Selected destination: {DESTINATION}")

if DESTINATION == 1:
    nav.set_kitchen_destination(KITCHEN_WAYPOINT, KITCHEN_TARGET)
    print("✓ Kitchen navigation with waypoint system activated")
elif DESTINATION == 2:
    nav.set_living_room_destination(LIVING_ROOM_WAYPOINT, LIVING_ROOM_TARGET)
    print("✓ Living room navigation with waypoint system activated")
else:
    print("❌ Invalid DESTINATION value! Defaulting to Kitchen...")
    nav.set_kitchen_destination(KITCHEN_WAYPOINT, KITCHEN_TARGET)
    print("✓ Kitchen navigation with waypoint system activated [Default]")

# Main control loop
print("\n" + "="*50)
print("NAVIGATION PHASE")
print("="*50)
print("⚠️  ROBOT POSITIONING NOTE:")
print("   • Ensure blind robot is placed close to leader robot (1-2m)")
print("   • Both robots should face the same initial direction")
print("   • Blind robot will copy leader's exact movements")
print()
print("Starting main control loop...")
step_count = 0
while robot.step(timestep) != -1:
    step_count += 1
    
    if step_count == 1:
        print("✓ Main control loop is running")
        print("✓ Robot is active and responding")
    
    if step_count % 100 == 0:  # Every 100 steps (about every 3.2 seconds)
        print(f"Control loop running... (step {step_count})")
    
    # Get motor speeds from navigation system
    left_speed, right_speed = nav.navigate()
    
    # Set motor speeds
    if step_count == 1:
        print(f"✓ Setting motor speeds: Left={left_speed:.2f}, Right={right_speed:.2f}")
    
    try:
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    except Exception as e:
        print(f"❌ ERROR setting motor speeds: {e}")
    
    # Check if target reached
    if nav.state == "REACHED":
        if nav.waypoint_reached:
            print("🎯 Final destination reached successfully!")
            if DESTINATION == 1:
                print("✅ Kitchen navigation complete! Robot reached the fridge via waypoint.")
            elif DESTINATION == 2:
                print("✅ Living room navigation complete! Robot reached the sofa via waypoint.")
            else:
                print("✅ Navigation complete! Robot reached the destination via waypoint.")
        else:
            print("🎯 Target reached successfully!")
            print("✅ Navigation complete! Robot has reached the destination.")
        
        print("🛑 Stopping robot...")
        left_motor.setVelocity(0.0)
        right_motor.setVelocity(0.0)
        
        print("To navigate to a different destination, change the DESTINATION variable and restart.")
