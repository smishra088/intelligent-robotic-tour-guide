## NLP & Voice Command Modules  
**Implemented and Tested by: Sweta Mishra**  

These modules enable natural interaction with the museum robot system through both voice and text input. The components work together to process user queries using speech recognition, GPT-4-based natural language understanding, and user preference extraction. All features align with the capabilities outlined in the project proposal.

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

These components provide robust support for natural language understanding and personalization based on user commands, as described in the original proposal.


```

### Navigation Module (ROS + Dynamic Path Planning) – Intelligent Robotic Tour Guide  
**Implemented by: Aishwarya Das**

#### Overview  
This module is a core component of the Intelligent Robotic Conversational and Interaction System for a simulated museum tour guide. It is responsible for real-time robotic control, environment-aware path planning, and adaptive movement through virtual museum spaces using the ROS 2 framework and Tiago robot in the Gazebo simulation environment.

The navigation system integrates dynamic path planning capabilities, allowing the robot to adjust its trajectory based on user preferences and simulated crowd density. Built on ROS 2 Humble, the module enables seamless command execution from the NLP and dialog systems, ensuring smooth, context-aware navigation throughout the museum.

---

#### Key Functionalities  

**ROS 2 and Tiago Simulation Setup**  
- Configured ROS 2 Humble on Ubuntu 22.04 within VirtualBox.  
- Integrated Tiago robot packages (`tiago_robot`, `tiago_simulation`, `pal_gazebo_worlds`) into a custom ROS 2 workspace (`ros2_ws`).  
- Successfully launched Tiago in Gazebo using public simulation settings without MoveIt for arm control.

**Dynamic Path Planning**  
- Employed the ROS 2 Navigation Stack for real-time path computation.  
- Enabled dynamic rerouting based on runtime parameters (e.g., "avoid crowded areas").  
- Future-ready integration with obstacle and anomaly detection modules (e.g., crowd detection via ML).

**Command Execution and Movement Control**  
- The robot interprets user navigation commands (e.g., “Go to the modern art gallery”) from the NLP module and executes motion via ROS navigation goals.  
- Supports basic movement primitives such as `forward`, `turn`, and `stop`.

**Crowd-Aware Navigation (Pluggable)**  
- The navigation stack is designed to accept simulated obstacle/crowd inputs from the anomaly detection module.  
- Path costmaps dynamically adjust to reflect user preferences and detected congestion zones.

**Personalization-Ready Routing**  
- Accepts high-level user instructions for route customization:  
  - “Skip crowded areas”  
  - “Take the scenic path”  
- Integrates these instructions into the global planner for route generation.

---

#### Sample Launch and Control Workflow  
```bash
# Launch Tiago in Gazebo (without MoveIt)
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True use_moveit:=False

# Example of issuing a navigation command (pseudo-code)
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{...}"
```

