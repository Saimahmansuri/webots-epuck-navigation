# Webots E-Puck Leader-Follower Robot Navigation system

# A cooperative navigation system using two Webots E-Puck robots: a GPS-guided leader and a blind follower using command-based path following and inter-robot communication.

This system implements a **leader-follower** navigation architecture where:
- **Leader Robot**: Uses GPS, compass, and distance sensors for intelligent navigation with waypoint system
- **Blind Robot**: Follows leader's commands without using its own navigation sensors

## 🤖 System Architecture

```
Leader Robot                    Blind Robot
├── GPS Navigation             ├── Command Execution
├── Obstacle Avoidance         ├── Motor Control  
├── Waypoint System           ├── Status Monitoring
└── Command Generation ────────→ └── Leader Following
           (JSON file)
```

## 📁 File Structure

```
controllers/
├── leader_robot/
│   └── leader_robot.py      # Smart navigation + command generation
├── blind_robot/
│   └── blind_robot.py       # Command following (no sensors)
└── robot_communication.json # Shared communication file
```

## 🚀 Quick Start

### 1. Setup in Webots

1. **Add Leader Robot** to your scene:
   - Set controller to: `leader_robot`
   - Ensure it has: GPS, Compass, Distance sensors

2. **Add Blind Robot** to your scene:
   - Set controller to: `blind_robot`
   - Only needs: Motors (no navigation sensors required)

### 2. Configure Destination

Edit `controllers/leader_robot/leader_robot.py`:

```python
# Choose Destination in leader_robot.py (line ~84)
DESTINATION = 1  # 1 = Kitchen, 2 = Living Room
```

### 3. Run the System

1. **Start Leader Robot**: Reset and run simulation
2. **Start Blind Robot**: Add to simulation or reset if already present
3. **Watch**: Leader navigates and blind robot follows

## 📡 Communication Protocol

The robots communicate via `robot_communication.json`:

```json
{
    "command": "forward",           # Movement command
    "speed_modifier": 0.75,         # Speed (0.0 - 1.0)
    "leader_state": "TO_WAYPOINT",  # Leader's current state
    "timestamp": 1640995200.0,      # Command timestamp
    "leader_speeds": {              # Raw motor speeds
        "left": 4.71,
        "right": 4.71
    }
}
```

### Command Types

| Command | Description | Blind Robot Action |
|---------|-------------|-------------------|
| `forward` | Move straight | Both motors forward |
| `turn_left` | Turn left | Left motor reverse, right forward |
| `turn_right` | Turn right | Left motor forward, right reverse |
| `reverse` | Move backward | Both motors reverse |
| `stop` | Stop movement | Both motors stop |
| `reached` | Destination reached | Stop and enter reached state |

## 🗺️ Navigation Flow

### Kitchen Navigation (DESTINATION = 1)
```
Start → Waypoint (-2.6, -5.9, 0) → Fridge (-0.52, -0.5, 0)
```

### Living Room Navigation (DESTINATION = 2)
```
Start → Waypoint (-4.517, -6.0805, 0) → Sofa (-7.0533, -0.8042, 0)
```

## 📊 Console Output Examples

### Leader Robot Output:
```
🗺️ Kitchen navigation started!
  Waypoint: (-2.600, -5.900, 0.000)
  Final destination: (-0.520, -0.500, 0.000)
📡 Sending to blind robot: forward (speed: 0.90)
[🗺️ Stage 1: To Waypoint] Position: (-1.234, -2.567), Distance: 2.135
📡 Sending to blind robot: turn_left (speed: 0.60)
✅ Waypoint confirmed reached! Distance: 0.620
📡 Sending to blind robot: reached (speed: 0.00)
```

### Blind Robot Output:
```
👁️‍🗨️ Blind robot: ➡️ Moving forward (speed: 0.90) | Leader: TO_WAYPOINT
👁️‍🗨️ Blind robot: ⬅️ Turning left (speed: 0.60) | Leader: TO_WAYPOINT
👁️‍🗨️ Blind robot: 🎯 Destination reached! | Leader: REACHED
```

## ⚙️ Configuration Options

### Leader Robot Settings (leader_robot.py):
```python
DISTANCE_THRESHOLD = 0.7        # Target reached distance
SLOW_DOWN_THRESHOLD = 1.5       # Start slowing down distance
OBSTACLE_THRESHOLD = 80.0       # Obstacle detection sensitivity
```

### Blind Robot Settings (blind_robot.py):
```python
MAX_SPEED = 6.28               # Maximum motor speed
command_timeout = 2.0          # Stop if no command for X seconds
```

## 🔧 Troubleshooting

### Common Issues:

1. **Blind robot not moving**:
   - Check if `robot_communication.json` exists
   - Verify leader robot is running first
   - Check console for timeout messages

2. **Communication file errors**:
   - Ensure both robots have write access
   - Check for JSON formatting errors
   - Restart both robots if file is corrupted

3. **Robots out of sync**:
   - Leader sends commands every 100ms
   - Blind robot times out after 2 seconds
   - Check timestamps in communication file

### Debug Commands:

```bash
# Check communication file content
cat robot_communication.json

# Monitor file changes (Linux/Mac)
watch -n 0.1 cat robot_communication.json
```

## 🎯 Advanced Features

### Custom Command Extensions:
Add new commands by modifying both controllers:

1. **Leader** (`send_command_to_blind_robot`): Add command generation logic
2. **Blind** (`execute_command`): Add command execution logic

### Example - Add "slow_forward" command:
```python
# In leader_robot.py
elif avg_speed < 0.3 and command == "forward":
    command = "slow_forward"

# In blind_robot.py  
elif command == "slow_forward":
    left_speed = MAX_SPEED * speed_modifier * 0.3
    right_speed = MAX_SPEED * speed_modifier * 0.3
    status = f"🐌 Moving slowly forward (speed: {speed_modifier:.2f})"
```

## 📈 Performance Tips

1. **Reduce communication frequency** for better performance
2. **Increase command timeout** for slower systems
3. **Use relative positions** for better synchronization
4. **Add error recovery** for robust operation

---


## 🧠 Author

**Saimah Mansuri**  
MSc Artificial Intelligence, Sheffield Hallam University  
🌐 [LinkedIn] https://www.linkedin.com/in/saimah-mansuri-2a7bab200
📧 Saimahuk141@gmail.com

**Ready to test?** Set `DESTINATION = 2` in the leader robot and watch both robots navigate to the living room via the waypoint system! 🚀 
