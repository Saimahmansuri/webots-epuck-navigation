"""
Blind Robot Controller for Apartment Navigation
Follows commands from the leader robot without using its own sensors
Reads movement instructions from a shared communication file
"""

from controller import Robot
import os
import json
import time
import math

# Create robot instance
print("🤖 Creating blind robot instance...")
robot = Robot()
print("✓ Blind robot instance created successfully")

# Get timestep for simulation
timestep = int(robot.getBasicTimeStep())
print(f"✓ Timestep set to: {timestep} ms")

# Initialize devices
print("Initializing blind robot devices...")

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

# GPS for position tracking (needed to follow leader)
print("Setting up GPS for following...")
gps = robot.getDevice('gps')
if gps is None:
    print("❌ WARNING: Could not find 'gps' device - blind robot will use basic following")
else:
    gps.enable(timestep)
    print("✓ GPS enabled for leader following")

# Compass for orientation (needed for seeking leader)
print("Setting up compass for orientation...")
compass = robot.getDevice('compass')
if compass is None:
    print("❌ WARNING: Could not find 'compass' device - blind robot will use basic orientation")
else:
    compass.enable(timestep)
    print("✓ Compass enabled for orientation tracking")

print("✓ Blind robot initialization complete!")
print("👁️‍🗨️ This robot is 'blind' - it only follows leader commands")

# Robot parameters
MAX_SPEED = 4.5   # Moderate speed to match leader
FOLLOW_DISTANCE = 1.5  # Desired distance behind leader (meters)
MAX_FOLLOW_DISTANCE = 3.0  # Maximum distance before robot hurries to catch up
MIN_FOLLOW_DISTANCE = 0.8  # Minimum distance before robot slows down

# Communication file path
# Use the main project directory (go up from controllers/blind_robot/ to project root)
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
COMMUNICATION_FILE = os.path.join(PROJECT_DIR, "robot_communication.json")
print(f"📁 Blind robot looking for communication file at: {COMMUNICATION_FILE}")

class BlindRobotController:
    def __init__(self):
        self.state = "WAITING_FOR_LEADER"  # WAITING_FOR_LEADER, FOLLOWING, STOPPED
        self.last_command_time = 0
        self.command_timeout = 2.0  # Stop if no command received for 2 seconds
        self.current_command = None
        self.step_count = 0
        self.leader_position = None
        self.last_distance_to_leader = 0.0
        self.prev_distance_to_leader = 0.0
        self.path_history = []
        self.distance_history = []  # Track distance changes to detect if we're actually following
        self.stuck_counter = 0
        
    def read_leader_command(self):
        """Read the latest command from the leader robot"""
        try:
            if os.path.exists(COMMUNICATION_FILE):
                file_size = os.path.getsize(COMMUNICATION_FILE)
                if file_size > 0:
                    # Try multiple times with small delay if file is locked
                    for attempt in range(3):
                        try:
                            with open(COMMUNICATION_FILE, 'r') as f:
                                content = f.read().strip()
                                if content:
                                    data = json.loads(content)
                                    return data
                            break
                        except (json.JSONDecodeError, IOError) as e:
                            if attempt < 2:  # Try again on first two attempts
                                time.sleep(0.001)  # 1ms delay
                                continue
                            else:
                                # Only print error occasionally to avoid spam
                                if self.step_count % 500 == 0:
                                    print(f"📁 File read issue: {type(e).__name__}")
                                return None
                else:
                    # File exists but is empty
                    if self.step_count % 200 == 0:  # Print occasionally
                        print(f"📁 Communication file exists but is empty")
                    return None
            else:
                # File doesn't exist
                if self.step_count % 500 == 0:  # Print occasionally
                    print(f"📁 Communication file doesn't exist at: {COMMUNICATION_FILE}")
                return None
        except Exception as e:
            if self.step_count % 300 == 0:  # Print occasionally
                print(f"⚠️ Error reading command file: {type(e).__name__}")
            return None
    
    def get_position(self):
        """Get current blind robot position from GPS"""
        try:
            if gps is not None:
                position = gps.getValues()
                if position is not None and len(position) >= 3:
                    return position
            return [0, 0, 0]  # Default position if GPS unavailable
        except Exception as e:
            print(f"⚠️ Error reading blind robot GPS: {e}")
            return [0, 0, 0]
    
    def calculate_distance_to_leader(self, leader_pos, blind_pos):
        """Calculate distance between leader and blind robot"""
        if leader_pos is None or blind_pos is None:
            return float('inf')
        
        dx = leader_pos[0] - blind_pos[0]
        dy = leader_pos[1] - blind_pos[1]
        return math.sqrt(dx*dx + dy*dy)
    
    def find_closest_path_point(self, blind_position):
        """Find the closest point on the leader's path for following"""
        if not self.path_history or len(self.path_history) == 0:
            return None
            
        # Look at recent path points only
        recent_points = self.path_history[-10:] if len(self.path_history) > 10 else self.path_history
        
        min_distance = float('inf')
        closest_point = None
        
        for point in recent_points:
            distance = self.calculate_distance_to_leader(
                [point['x'], point['y'], point['z']], 
                blind_position
            )
            if distance < min_distance:
                min_distance = distance
                closest_point = point
                
        return closest_point
    
    def find_next_breadcrumb_to_follow(self, blind_position, path_history, leader_position):
        """Find the optimal breadcrumb to follow from the leader's path"""
        if not path_history or len(path_history) == 0:
            return None
            
        # Find the breadcrumb that is:
        # 1. Behind the leader (so we follow the path, not chase the leader)
        # 2. Reasonably close to us (within reach)
        # 3. Forms a good following distance
        
        # Target following distance (we want to be this far behind the leader's path)
        target_following_distance = 0.7  # 70cm behind leader's path (matches leader's ideal distance)
        
        best_breadcrumb = None
        best_score = float('inf')
        
        for breadcrumb in path_history:
            breadcrumb_pos = [breadcrumb['x'], breadcrumb['y'], breadcrumb['z']]
            
            # Distance from blind robot to this breadcrumb
            distance_to_breadcrumb = self.calculate_distance_to_leader(breadcrumb_pos, blind_position)
            
            # Distance from this breadcrumb to leader's current position
            breadcrumb_to_leader_distance = self.calculate_distance_to_leader(breadcrumb_pos, leader_position)
            
            # Skip breadcrumbs that are too far from us (unreachable)
            if distance_to_breadcrumb > 2.0:
                continue
                
            # Skip breadcrumbs that are ahead of or at the leader's current position
            if breadcrumb_to_leader_distance < 0.3:
                continue
                
            # Score this breadcrumb based on how close it is to our ideal following position
            # Ideal: breadcrumb is about target_following_distance behind the leader
            distance_deviation = abs(breadcrumb_to_leader_distance - target_following_distance)
            
            # Also consider how far the breadcrumb is from us (prefer closer ones)
            accessibility_score = distance_to_breadcrumb * 0.3
            
            total_score = distance_deviation + accessibility_score
            
            if total_score < best_score:
                best_score = total_score
                best_breadcrumb = breadcrumb
                
        # If no good breadcrumb found, take the closest one that's behind the leader
        if best_breadcrumb is None and len(path_history) > 0:
            for breadcrumb in path_history:
                breadcrumb_pos = [breadcrumb['x'], breadcrumb['y'], breadcrumb['z']]
                breadcrumb_to_leader_distance = self.calculate_distance_to_leader(breadcrumb_pos, leader_position)
                
                # Only consider breadcrumbs that are behind the leader
                if breadcrumb_to_leader_distance > 0.4:
                    distance_to_breadcrumb = self.calculate_distance_to_leader(breadcrumb_pos, blind_position)
                    if distance_to_breadcrumb < best_score:
                        best_score = distance_to_breadcrumb
                        best_breadcrumb = breadcrumb
        
        return best_breadcrumb
    
    def get_orientation(self):
        """Get current robot orientation from compass"""
        try:
            if compass is not None:
                compass_values = compass.getValues()
                if compass_values is not None and len(compass_values) >= 3:
                    # Calculate bearing from compass values
                    bearing = math.atan2(compass_values[0], compass_values[1])
                    return bearing
            return 0
        except Exception as e:
            print(f"⚠️ Error reading compass: {e}")
            return 0
            
    def calculate_angle_to_leader(self, blind_pos, leader_pos, current_orientation):
        """Calculate angle to turn towards leader"""
        if leader_pos is None:
            return 0
            
        dx = leader_pos[0] - blind_pos[0]
        dy = leader_pos[1] - blind_pos[1]
        target_angle = math.atan2(dy, dx)
        
        # Normalize angle difference
        angle_diff = target_angle - current_orientation
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
            
        return angle_diff
    
    def is_actually_following(self, distance_to_leader):
        """Detect if robot is actually following or just thinks it is"""
        # Track distance history
        self.distance_history.append(distance_to_leader)
        if len(self.distance_history) > 20:  # Keep last 20 readings
            self.distance_history = self.distance_history[-20:]
            
        # If we have enough history, check if distance is decreasing when it should
        if len(self.distance_history) >= 10:
            recent_avg = sum(self.distance_history[-5:]) / 5
            older_avg = sum(self.distance_history[-10:-5]) / 5
            
            # If distance isn't decreasing when we're close, we might be lost
            # But be more lenient - only flag as stuck if significantly not improving
            if distance_to_leader < 3.0 and recent_avg > older_avg + 0.2:  # More lenient criteria
                self.stuck_counter += 1
            else:
                self.stuck_counter = max(0, self.stuck_counter - 2)  # Decrease faster
                
            # If stuck for too long, we're probably not actually following
            return self.stuck_counter < 30  # Reduced from 50 to 30
        
        return True  # Assume following if not enough data

    def execute_command(self, command_data):
        """Execute the command received from leader"""
        if command_data is None:
            return 0.0, 0.0
        
        command = command_data.get('command', 'stop')
        speed_modifier = command_data.get('speed_modifier', 1.0)
        leader_state = command_data.get('leader_state', 'unknown')
        timestamp = command_data.get('timestamp', 0)
        leader_pos_data = command_data.get('leader_position', None)
        leader_speeds = command_data.get('leader_speeds', {})
        
        # Check if command is recent enough
        current_time = time.time()
        if current_time - timestamp > self.command_timeout:
            if self.step_count % 100 == 0:  # Print occasionally
                print(f"⏰ Command too old ({current_time - timestamp:.1f}s ago), stopping")
            return 0.0, 0.0
        
        # Update last command time
        self.last_command_time = current_time
        
        # Get blind robot position and update path history
        blind_position = self.get_position()
        
        # Update path history from leader
        path_history_data = command_data.get('path_history', [])
        if path_history_data:
            self.path_history = path_history_data
        
        # Path-following behavior: Follow exact breadcrumbs left by leader
        target_position = None
        following_modifier = 1.0
        distance_info = ""
        use_breadcrumb_navigation = False
        
        if leader_pos_data is not None and path_history_data:
            leader_position = [leader_pos_data['x'], leader_pos_data['y'], leader_pos_data['z']]
            distance_to_leader = self.calculate_distance_to_leader(leader_position, blind_position)
            
            # Find the optimal breadcrumb to follow
            target_breadcrumb = self.find_next_breadcrumb_to_follow(blind_position, path_history_data, leader_position)
            
            if target_breadcrumb:
                target_position = [target_breadcrumb['x'], target_breadcrumb['y'], target_breadcrumb['z']]
                distance_to_breadcrumb = self.calculate_distance_to_leader(target_position, blind_position)
                use_breadcrumb_navigation = True
                
                # Speed adjustment based on how far we are from our target breadcrumb
                if distance_to_breadcrumb <= 0.3:
                    # Very close to breadcrumb - match leader speed exactly
                    following_modifier = 1.0
                    distance_info = f" [🍞 Following breadcrumb: {distance_to_breadcrumb:.2f}m - Perfect]"
                elif distance_to_breadcrumb <= 0.8:
                    # Moderate distance to breadcrumb - speed up slightly
                    following_modifier = 1.3
                    distance_info = f" [🍞 Chasing breadcrumb: {distance_to_breadcrumb:.2f}m - Faster]"
                else:
                    # Far from breadcrumb - speed up significantly
                    following_modifier = 1.8
                    distance_info = f" [🍞 Far from breadcrumb: {distance_to_breadcrumb:.2f}m - Much faster]"
            else:
                # No suitable breadcrumb - fall back to following leader directly
                target_position = leader_position
                if distance_to_leader <= 0.5:
                    following_modifier = 1.0
                    distance_info = f" [👥 Following leader: {distance_to_leader:.2f}m - Close]"
                elif distance_to_leader <= 1.5:
                    following_modifier = 1.5
                    distance_info = f" [👥 Catching up to leader: {distance_to_leader:.1f}m - Fast]"
                else:
                    following_modifier = 2.0
                    distance_info = f" [👥 Emergency: {distance_to_leader:.1f}m - Very fast]"
            
            self.leader_position = leader_position
            self.prev_distance_to_leader = self.last_distance_to_leader
            self.last_distance_to_leader = distance_to_leader
        
        # Apply speed modification for close formation following
        final_speed_modifier = speed_modifier * following_modifier
        
        # Execute the command
        left_speed = 0.0
        right_speed = 0.0
        
        # Helper function to cap speed
        def cap_speed(speed):
            return max(-MAX_SPEED, min(MAX_SPEED, speed))
        
        # Breadcrumb navigation: Navigate to target position and adapt to leader speeds  
        if target_position and leader_speeds and 'left' in leader_speeds and 'right' in leader_speeds:
            # Calculate navigation direction to target breadcrumb/position
            current_orientation = self.get_orientation()
            angle_to_target = self.calculate_angle_to_leader(blind_position, target_position, current_orientation)
            
            # Base speed from leader's actual motor speeds
            base_speed = (abs(leader_speeds['left']) + abs(leader_speeds['right'])) / 2.0
            if base_speed < 0.1:  # Leader is stopped or moving very slowly
                base_speed = MAX_SPEED * 0.4  # Use moderate default speed
            
            # Navigate toward target with proportional control
            if abs(angle_to_target) > 0.2:  # Need to turn toward target
                # Turn toward target while maintaining forward motion
                turn_strength = min(abs(angle_to_target), 1.0)  # Cap turn strength
                
                if angle_to_target > 0:  # Turn left
                    left_speed = cap_speed(base_speed * (1.0 - turn_strength * 0.6) * following_modifier)
                    right_speed = cap_speed(base_speed * (1.0 + turn_strength * 0.2) * following_modifier)
                else:  # Turn right
                    left_speed = cap_speed(base_speed * (1.0 + turn_strength * 0.2) * following_modifier)
                    right_speed = cap_speed(base_speed * (1.0 - turn_strength * 0.6) * following_modifier)
                    
                status = f"🧭 Navigating to target (turn: {math.degrees(angle_to_target):.0f}°, speed: {base_speed:.1f})"
            else:
                # Moving straight toward target
                left_speed = cap_speed(base_speed * following_modifier)
                right_speed = cap_speed(base_speed * following_modifier)
                status = f"🧭 Moving straight to target (speed: {base_speed:.1f})"
            
            # Override for special commands
            if command == "wait":
                # Leader is waiting - continue navigating to current target at higher speed
                left_speed *= 1.5  # Speed up when leader is waiting
                right_speed *= 1.5
                status += " [⏸️ Leader waiting - Speed up]"
            elif command == "stop":
                left_speed = 0.0
                right_speed = 0.0
                status = "🛑 Stopped"
            elif command == "reached":
                left_speed = 0.0
                right_speed = 0.0
                status = "🎯 Destination reached!"
                self.state = "STOPPED"
        elif target_position:
            # Fallback navigation when no leader speeds available - navigate to target position
            current_orientation = self.get_orientation()
            angle_to_target = self.calculate_angle_to_leader(blind_position, target_position, current_orientation)
            
            # Use moderate speed for navigation
            nav_speed = MAX_SPEED * 0.6 * following_modifier
            
            if abs(angle_to_target) > 0.3:  # Need to turn towards target
                if angle_to_target > 0:
                    left_speed = cap_speed(-nav_speed * 0.7)
                    right_speed = cap_speed(nav_speed)
                    status = f"🧭 Fallback: Turning left toward target"
                else:
                    left_speed = cap_speed(nav_speed)
                    right_speed = cap_speed(-nav_speed * 0.7)
                    status = f"🧭 Fallback: Turning right toward target"
            else:
                # Move towards target
                left_speed = cap_speed(nav_speed)
                right_speed = cap_speed(nav_speed)
                status = f"🧭 Fallback: Moving toward target"
        else:
            # Fallback to command-based control if no leader speeds available
            if command == "forward":
                left_speed = cap_speed(MAX_SPEED * final_speed_modifier)
                right_speed = cap_speed(MAX_SPEED * final_speed_modifier)
                status = f"➡️ Moving forward (speed: {final_speed_modifier:.2f})"
                
            elif command == "turn_left":
                left_speed = cap_speed(-MAX_SPEED * final_speed_modifier * 0.6)
                right_speed = cap_speed(MAX_SPEED * final_speed_modifier * 0.6)
                status = f"⬅️ Turning left (speed: {final_speed_modifier:.2f})"
                
            elif command == "turn_right":
                left_speed = cap_speed(MAX_SPEED * final_speed_modifier * 0.6)
                right_speed = cap_speed(-MAX_SPEED * final_speed_modifier * 0.6)
                status = f"➡️ Turning right (speed: {final_speed_modifier:.2f})"
                
            elif command == "reverse":
                left_speed = cap_speed(-MAX_SPEED * final_speed_modifier * 0.8)
                right_speed = cap_speed(-MAX_SPEED * final_speed_modifier * 0.8)
                status = f"⬅️ Moving backward (speed: {final_speed_modifier:.2f})"
                
            elif command == "stop":
                left_speed = 0.0
                right_speed = 0.0
                status = "🛑 Stopped"
                
            elif command == "reached":
                left_speed = 0.0
                right_speed = 0.0
                status = "🎯 Destination reached!"
                self.state = "STOPPED"
                
            elif command == "wait":
                # Leader is waiting but we don't have leader position - just stop
                left_speed = 0.0
                right_speed = 0.0
                status = "⏸️ Leader waiting - No position data"
                
            else:
                left_speed = 0.0
                right_speed = 0.0
                status = f"❓ Unknown command: {command}"
        
        # Send blind robot position back to leader
        try:
            # Read current communication file and update with position
            if os.path.exists(COMMUNICATION_FILE):
                comm_data = None
                # Try to read existing data
                for attempt in range(2):  # Fewer attempts for position updates
                    try:
                        with open(COMMUNICATION_FILE, 'r') as f:
                            content = f.read().strip()
                            if content:
                                comm_data = json.loads(content)
                        break
                    except (json.JSONDecodeError, IOError):
                        if attempt < 1:
                            time.sleep(0.001)
                            continue
                        else:
                            comm_data = {}  # Use empty dict if read fails
                            break
                
                if comm_data is None:
                    comm_data = {}
                
                # Add blind robot position
                comm_data['blind_robot_position'] = {
                    'x': round(blind_position[0], 3),
                    'y': round(blind_position[1], 3),
                    'z': round(blind_position[2], 3),
                    'timestamp': time.time()
                }
                
                # Write back to file atomically  
                temp_file = COMMUNICATION_FILE + ".blind_tmp"
                with open(temp_file, 'w') as f:
                    json.dump(comm_data, f)
                    f.flush()
                    os.fsync(f.fileno())
                
                # Atomic rename
                try:
                    os.replace(temp_file, COMMUNICATION_FILE)
                except:
                    # Fallback
                    if os.path.exists(COMMUNICATION_FILE):
                        os.remove(COMMUNICATION_FILE)
                    os.rename(temp_file, COMMUNICATION_FILE)
                    
        except Exception as e:
            # Silently handle communication errors
            if self.step_count % 300 == 0:  # Print occasionally
                print(f"⚠️ Position update error: {type(e).__name__}")
            # Clean up temp file
            temp_file = COMMUNICATION_FILE + ".blind_tmp"
            if os.path.exists(temp_file):
                try:
                    os.remove(temp_file)
                except:
                    pass
        
        # Print status every second or on state change
        if (self.step_count % 100 == 0 or 
            self.current_command != command or 
            command in ["reached", "stop"] or
            abs(self.last_distance_to_leader - self.prev_distance_to_leader) > 0.5):
            print(f"👁️‍🗨️ Blind robot: {status}{distance_info} | Leader: {leader_state}")
            
        self.current_command = command
        return left_speed, right_speed
    
    def follow_leader(self):
        """Main following logic"""
        # Read command from leader
        command_data = self.read_leader_command()
        
        if command_data is None:
            # No command available
            if self.state == "WAITING_FOR_LEADER":
                if self.step_count % 300 == 0:  # Print every 10 seconds
                    print("⏳ Waiting for leader robot commands...")
            return 0.0, 0.0
        
        # First command received
        if self.state == "WAITING_FOR_LEADER":
            self.state = "FOLLOWING"
            print("✅ Leader robot detected! Starting to follow...")
        
        # Execute the command
        return self.execute_command(command_data)

# Initialize blind robot controller
blind_controller = BlindRobotController()

print("\n" + "="*50)
print("BLIND ROBOT INITIALIZATION PHASE")
print("="*50)
print("Performing initial robot step to enable all devices...")
step_result = robot.step(timestep)  # Initial step to enable devices

if step_result == -1:
    print("❌ ERROR: Robot step failed during initialization")
    exit(1)
else:
    print("✓ Initial robot step completed successfully")

print("✓ Blind robot fully initialized and ready")

print("\n" + "="*50)
print("FOLLOWING PHASE")
print("="*50)
print("👁️‍🗨️ Blind robot waiting for leader commands...")
print(f"🗂️ Blind robot working directory: {os.getcwd()}")
print("📁 Communication file:", COMMUNICATION_FILE)
print()
print("⚠️  IMPORTANT: For proper following behavior:")
print("   • Place blind robot close to leader robot (within 1-2 meters)")
print("   • Ensure both robots face the same direction initially")
print("   • Blind robot will copy leader's exact motor commands")
print()

# Main control loop
step_count = 0
while robot.step(timestep) != -1:
    step_count += 1
    blind_controller.step_count = step_count
    
    if step_count == 1:
        print("✓ Blind robot control loop is running")
        print("✓ Robot is active and waiting for leader")
    
    # Get motor speeds from blind controller
    left_speed, right_speed = blind_controller.follow_leader()
    
    # Set motor speeds
    try:
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    except Exception as e:
        print(f"❌ ERROR setting motor speeds: {e}")
    
    # Check if stopped due to reaching destination
    if blind_controller.state == "STOPPED":
        if step_count % 500 == 0:  # Print occasionally
            print("🎯 Blind robot has stopped - destination reached!") 