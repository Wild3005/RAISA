# RAISA Humanoid Robot System

ROS2-based humanoid robot with **master.cpp** controlling all Python nodes (dual_leg, pose_detection, etc).

---

## System Overview

The **Master Node** (`master.cpp`) is the central controller that:
- Subscribes to all sensor data (vision, UWB, lidar, dual leg)
- Makes navigation decisions via Finite State Machine (FSM)
- Publishes robot commands and navigation goals
- Handles UI button/keyboard input

All Python scripts (`dual_leg_pub.py`, `pose_detection.py`, etc.) publish data → Master consumes and decides → Master commands robot.

---

## Two Operating Modes

### Mode 1: UWB Tracking Mode
Uses UWB anchors (Pozyx) to track human position

### Mode 2: Camera Tracking Mode  
Uses MediaPipe camera + Lidar to detect and track human

---

## Accessing the Robot

SSH into the robot:
```bash
ssh raisa@<robot_ip>
cd raisa_humanoid
```

---

## Setup & Run

### **MODE 1: UWB Tracking**

#### 1. Configure Master Node
Edit `src/master/src/master.cpp` 
```cpp
fsm_mode.value = MODE_TRACK_UWB;  // Set to UWB mode
```

#### 2. Configure Launch File
Edit `src/ros2_utils/launch/all.launch.py` - Enable these nodes in `LaunchDescription([...])`:
```python
pozyx_node,    # UWB positioning
dual_leg,      # ESP32 leg sensors
master,        # Master controller
io_reeman_node,
rosbridge_server,
ui_server,
rosapi_node,
web_video_server,
```

**Comment out** camera nodes:
```python
# capture,
# pose_detection,
```

#### 3. Build & Run
```bash
./make.sh      # Build the workspace
./run.sh       # Launch the system
```

---

### **MODE 2: Camera Tracking**

#### 1. Configure Master Node
Edit `src/master/src/master.cpp` 
```cpp
fsm_mode.value = MODE_TRACK_CAMERA;  // Set to camera mode
```

#### 2. Configure Launch File
Edit `src/ros2_utils/launch/all.launch.py` - Enable these nodes in `LaunchDescription([...])`:
```python
capture,          # Camera capture
pose_detection,   # MediaPipe pose detection
master,
io_reeman_node,
rosbridge_server,
ui_server,
rosapi_node,
web_video_server,
```

**Comment out** UWB nodes:
```python
# pozyx_node,
# dual_leg,
```

#### 3. Build & Run
```bash
./make.sh      # Build the workspace
./run.sh       # Launch the system
```

---

### Main Control Buttons

| Button | Function | How It Works |
|--------|----------|--------------|
| **5** | **Orientation Following** | **UWB Mode**: Robot rotates to match the UWB anchor's orientation. If the UWB anchor is facing north, Raisa will face north.<br>**Camera Mode**: Robot rotates to match the person's orientation detected by the camera. If the person is facing south, Raisa will also face south. |
| **6** | **Human Following (Standing)** | Robot navigates to the person's right side.<br>• **Person facing away**: Raisa navigates and stops beside (on the right) of the person.<br>• **Person facing toward Raisa**: Raisa stops on the right side and performs a 180° rotation to face the same direction as the person.<br>Works identically in both UWB and Camera modes. |
| **7** | **Human Following (Sitting)** | Same behavior as Button 6, but Raisa's final orientation (theta) leans toward the person instead of facing the same direction. Used when the person is sitting down. |
| **8** | **Continuous Following** | **UWB Mode**: Continuously follows the UWB tag worn by the person.<br>**Camera Mode**: Continuously follows as long as the person remains visible in the camera frame. Robot maintains tracking while the person moves. |
| **12** | **UWB Calibration** | **Setup**: Place both UWB modules (robot's and person's) on Raisa's shelf to align them (minimize orientation difference).<br>**Action**: Press Button 12 to calibrate the orientation offset between the robot's odometry and the UWB coordinate system.<br>**After calibration**: Test other UWB-based buttons (5, 6, 7, 8) for accurate orientation tracking. |

---



## Quick Debug

```bash
# Check if master is running
ros2 node list | grep master

# Monitor FSM state
ros2 topic echo /ui/robot/fsm_mode

# Check vision detection
ros2 topic echo /vision/pose_detected

# Check UWB data
ros2 topic echo /uwb_pose2d

# View all topics
ros2 topic list
```

---


## Setting Up Dual Leg System

The dual leg system uses ESP32 microcontrollers with AS5600 magnetic encoders. Each leg has its own WiFi access point for configuration.

### Hardware Overview
- **Left Leg**: ESP32 + AS5600 encoder
- **Right Leg**: ESP32 + AS5600 encoder  
- **Communication**: UDP over WiFi
- **Ports**: Left (12345), Right (12346)

### Initial Configuration (One-time Setup)

Each ESP32 must be configured to connect to the robot's WiFi network:

#### Step 1: Connect to Left Leg
```bash
1. Connect your laptop/phone to WiFi AP: "AS5600-LeftLeg"
2. Open browser and navigate to: http://192.168.4.1
3. In the web interface:
   - Enter robot's WiFi SSID (Raisa's Conenected IP)
   - Enter WiFi password
   - Click "Connect" or "Save"
4. ESP32 will restart and connect to robot's network
```

#### Step 2: Connect to Right Leg
```bash
1. Connect your laptop/phone to WiFi AP: "AS5600-RightLeg"
2. Open browser and navigate to: http://192.168.4.1
3. In the web interface:
   - Enter robot's WiFi SSID (Raisa's Conenected IP)
   - Enter WiFi password
   - Click "Connect" or "Save"
4. ESP32 will restart and connect to robot's network
```

### Verification

Once both legs are connected to the robot's network:

```bash
ros2 topic echo /dual_leg

```

### Troubleshooting

**Legs not publishing data:**
```bash
# 1. Check if ESP32s are on robot's network
ping <left_esp_ip>
ping <right_esp_ip>

# 2. Check if dual_leg_pub.py is running
ros2 node list | grep dual_leg

# 3. Restart the node
ros2 run dual_leg dual_leg_pub.py
```

**Web interface not loading (192.168.4.1):**
- Ensure you're connected to the ESP32's AP (AS5600-LeftLeg or AS5600-RightLeg)
- Try resetting the ESP32 (power cycle)
- Check if your device's WiFi didn't auto-switch to another network

**Angles seem incorrect:**
- Straighten both of the legs (one at a time) then reset the degrees on the leg's web interface
- Check UDP port configuration in ESP32 firmware (should be 12345 for left, 12346 for right)
- Calibrate by moving leg through full range and observing output

