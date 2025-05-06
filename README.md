## NLP, Voice Command, and Anomaly Detection Modules  
**Implemented and Tested by: Sweta Mishra**  

These modules enable natural user interaction and situational awareness within the museum robot system. The components cover voice input, natural language understanding, user preference extraction, and real-time anomaly detection using visual input. All implementations align with the goals outlined in the original project proposal.

---

### 1. `voice_input_test.py`  
Tests the voice recognition pipeline using user speech commands. Transcribes input and validates voice interaction flow.

**Voice Queries Tested:**
- Take me to the sculpture gallery  
- I want to see modern art and skip crowded sections  
- Show me something peaceful and quiet  
- Are there any interactive exhibits for kids?

---

### 2. `nlp_test.py`  
Verifies GPT-4’s ability to handle and respond to typed natural language queries related to the museum setting.

**Typed Queries Tested:**
- Can you take me to a sculpture gallery that’s not too crowded?  
- Give me a short tour focused on modern art  
- I want to avoid noisy areas and see digital installations

---

### 3. `preference_test.py`  
Implements rule-based extraction of user preferences from spoken or typed input. This module detects themes like interest in modern art, sculpture, crowd avoidance, and more.

**Example Preferences Extracted:**
- Avoid Crowds  
- Visit Modern Art Section  
- Visit Sculpture Gallery  
- Visit Painting Section

---

### 4. `voice_dialog_interface.py`  
Integrates voice input, GPT-4 response generation, and real-time preference extraction in a complete user interface. The system captures user voice, generates personalized responses, and extracts intent in a structured form.

---
### 5. `anomaly_module`  
Implements real-time detection of crowds and obstacles using YOLOv8, enhancing the robot’s awareness of its environment. The module processes visual input from the camera and flags anomalies to support safe interaction in the museum space. 

---


These components provide robust support for natural language understanding and personalization based on user commands, as described in the original proposal.


# Navigation Module (ROS 2 + Tiago + Dynamic Path Planning)  
**Implemented and Tested by: Aishwarya Das**

This module enables real-time autonomous navigation in a simulated museum environment using ROS 2 Humble and the Tiago robot. It forms the foundation of adaptive movement, supporting user-driven commands and personalized routing powered by dialog input.

---

### 1. System Setup & Simulation Integration

- **Environment Setup**: Installed and configured Ubuntu 22.04 with ROS 2 Humble for simulation-based development.
- **Workspace Build**: Created and built a custom ROS 2 workspace (`ros2_ws`) for integrating all Tiago-related packages and project modules.
- **Package Installation**: Cloned and compiled essential Tiago packages:
  - `tiago_robot` : https://github.com/pal-robotics/tiago_robot.git
  - `tiago_simulation` : https://github.com/pal-robotics/tiago_simulation.git
  - `pal_gazebo_worlds` : https://github.com/pal-robotics/pal_gazebo_worlds.git
- **Launch Configuration**: Launched Tiago robot in a custom world in Gazebo with flags disabling MoveIt (arm) and enabling navigation.
  - World includes dynamic elements for navigation and testing.

---

### 2. Navigation Features & Functionalities

**🔹 ROS 2 Navigation Stack Integration**
- Set up ROS 2 `nav2` stack for autonomous path planning.
- Robot accepts goal poses and computes obstacle-aware paths in real time.

**🔹 Custom World Integration**
- Integrated custom `box_world.world` into `tiago_gazebo` simulation.
- Ensured proper coordinate frames and map origin alignment for navigation accuracy.

**🔹 Dynamic Path Execution**
- Robot receives target destinations from the GPT-4-powered NLP system.
- Executes movement commands like turning, forward motion, and halting using `nav2` actions.
- Supports rerouting logic based on preferences such as:
  - “Skip crowded areas”
  - “Take a peaceful path”

**🔹 Crowd-Aware Rerouting (Framework Ready)**
- Navigation stack is built to dynamically adjust costmaps using simulated obstacle or anomaly feeds.
- Prepares groundwork for future integration with YOLOv8-based detection module.

**🔹 Multi-User Command Handling (Context-Linked)**
- Preserves user-specific navigation history using dialog context.
- Navigation behavior is dynamically tailored per user session through backend logic.

---

### 🛠️ Sample Workflow

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash

# Launch Tiago in a custom Gazebo world
ros2 launch tiago_gazebo tiago_gazebo.launch.py \
  world_name:=install/my_custom_world/share/my_custom_world/worlds/box_world.world \
  is_public_sim:=True use_moveit:=False navigation:=True gui:=True

# Launch Nav2 (ROS 2 Navigation Stack)
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=/absolute/path/to/museum_map.yaml

# Test camera with rqt_image_view
ros2 run rqt_image_view rqt_image_view

# Launch SLAM Toolbox
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True

# Open RViz and Visualize the Robot
ros2 run rviz2 rviz2

# Move Tiago using teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Save the map after exploring
ros2 run nav2_map_server map_saver_cli -f ~/ros2_ws/maps/museum_map

# Launch Nav2 with the saved map
ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=True \
  map:=/home/aish/ros2_ws/maps/museum_map.yaml
```

Ensure:
- You have a map YAML and PGM (or PNG) file.

```yaml
plugins:
  - obstacle_layer
  - inflation_layer
  - voxel_layer
  - crowd_layer

crowd_layer:
  plugin: "nav2_costmap_2d::CrowdLayer"
  topic: "/crowd_zones"
  enabled: True
```

- AMCL (localization) node is running.
- You see TF links in rviz2.

---

### Key Contributions

- Full setup and validation of ROS 2 Humble on Ubuntu 22.04.
- End-to-end Tiago simulation integration within Gazebo.
- Configured dynamic navigation pipeline via ROS 2 `nav2` stack.
- Bridged GPT-4/NLP command input with ROS goal execution.
- Enabled scalable support for context-aware navigation, obstacle rerouting, and user preference parsing.

---

### 3. Path Adjustment from Anomaly Detection to Nav2

#### Goal:
Use output from anomaly detection (YOLOv8, OpenCV) to dynamically update navigation costmaps.

#### Integration

- Publish obstacle zones to `/crowd_zones` using `OccupancyGrid` using `anomaly_obstacle_publisher.py`.
- Configure `nav2_params.yaml` to include `crowd_layer`.


---

### Summary

| Task                          | Tool              | Format          |
|-------------------------------|-------------------|------------------|
| Obstacle detection output     | Anomaly module    | `OccupancyGrid` |
| Dynamic costmap update        | `crowd_layer`     | ROS 2 plugin     |
| Goal setting                  | RViz              | `2D Nav Goal`    |

---

## 4. Multi-User Dialog Management with Speaker Context

#### Goal:
Support simultaneous multi-user interaction with speaker-specific navigation preferences.

#### Tool:
- `pyannote-audio` for speaker diarization
- OpenAI Whisper for speech to text parsing

####  Example: Maintain user-specific history

```python
conversation_history = {
  "Speaker 0": ["Where is the sculpture gallery?", "Avoid crowds"],
  "Speaker 1": ["Take me to modern art", "What’s next?"]
}
```

Use this context to guide personalized navigation sessions.

---

### 5. GPT Prompt Personalization

```python
prompt = f"""
Speaker 0 said:
- {conversation_history['Speaker 0'][-2]}
- {conversation_history['Speaker 0'][-1]}

Speaker 1 replied:
- {conversation_history['Speaker 1'][-1]}

Now continue the conversation...
"""
```

---

### Contribution Summary

- Implemented speaker-aware dialog tracking
- Enabled preference preservation per user session
- Integrated NLP with path planning for interactive museum tours

