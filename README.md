# NLP Module (GPT-4 Mock) – Intelligent Robotic Tour Guide

**Implemented by: Sweta Mishra**  

## Overview

This module is part of the Intelligent Robotic Conversational and Interaction System for a simulated museum tour guide. It is responsible for handling complex user queries and generating intelligent, context-aware responses using natural language processing techniques.

Due to quota restrictions with OpenAI’s GPT-4 API, a mock GPT-4 interface was implemented to simulate intelligent responses. This enables seamless system integration and testing while maintaining compatibility with the OpenAI SDK for future deployment.

## Key Functionalities

- **Simulated GPT-4 Interface**  
  A Python module (`gpt_interface.py`) was developed to simulate GPT-4 responses based on user prompts using predefined logic.

- **Multi-turn Capability (Pluggable)**  
  The module is structured to support multi-turn conversations, enabling future integration with live GPT-4 or similar LLMs.

- **Command Understanding**  
  Handles tour-related commands such as:
  - “Take me to the sculpture gallery”
  - “Show me modern art”

- **Personalization-Ready**  
  The module is designed to be extended with user-specific data to provide personalized responses in future versions.

## Sample Code Logic

```python
def get_gpt_response(prompt):
    if "sculpture" in prompt.lower():
        return "Sure! The sculpture gallery is down the main hall to your right."
    elif "modern art" in prompt.lower():
        return "Absolutely! Let's head over to the modern art exhibit."
    else:
        return "Great! I’ll guide you through the museum based on your interests."




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

