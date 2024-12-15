# Resilient Control and Coordination of Connected and Autonomous Vehicles (CAVs)

## Overview
This project focuses on developing a hardware-in-the-loop (HIL) simulation environment to test and validate a resilient control system for Connected and Autonomous Vehicles (CAVs). By integrating a trust-aware control framework, the system addresses safety and security concerns in urban environments, making it robust against adversarial attacks and control errors. The simulation leverages CARLA, ROS, and the AgileX LIMO robot to evaluate control coordination and attack resilience.

## Features
- **Trust-Aware Control**: Implements Optimal Control Barrier Functions (OCBF) for collision mitigation and intersection management.
- **HIL Simulation**: Combines CARLA's virtual environment with physical AgileX LIMO hardware.
- **Adversarial Attack Testing**: Simulates Replay and Distributed Denial of Service (DDoS) attacks.
- **ROS-Based Communication**: Establishes a robust publisher-subscriber network for control and status updates.
- **Dynamic Traffic Coordination**: Includes a FIFO-based intersection management system.

## Installation

### Prerequisites
- **CARLA Simulator**: [CARLA Installation Guide](https://carla.org/)
- **ROS**: Compatible with the version running on your system.

### Steps
1. Clone the repository:
   ```bash
   git clone https://github.com/calsfu/Trust-Aware-Control-using-HIL-Carla-Simulation.git
   cd Trust-Aware-Control-using-HIL-Carla-Simulation
   ```
2. Set up CARLA simulation:
3. Install ROS dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
4. Configure the AgileX LIMO robot:
   - Follow the AgileX LIMO setup guide to connect the robot and establish Wi-Fi teleoperation.
5. Run the ROS workspace:
   ```bash
   catkin_make
   source devel/setup.bash
   ```

## Usage

### Running the Simulation
1. Launch the CARLA simulator:
   ```bash
   ./CarlaUE4.sh
   ```
2. Start the ROS nodes:
   ```bash
   rosrun cav_project main_coordinator.py
   rosrun cav_project carla_env.py
   rosrun cav_project replay_attack
   rosrun cav_project ddos_attack
   ```
3. Control the LIMO robot using the HIL integration and observe its behavior in CARLA.

### Attack Injection
- Configure and enable Replay/DDoS attack nodes in the ROS network to test system resilience.

## System Architecture
### Major Components
- **Road-Side Unit (RSU)**: Manages CAV intersections and coordinates collision avoidance.
- **OCBF Controller**: Ensures safe and optimal path planning.
- **Main Coordinator**: Integrates the RSU, OCBF, and attack injection mechanisms.
- **CARLA Simulation**: Handles the virtual environment with multiple CAV agents.
- **AgileX LIMO Robot**: Acts as the physical agent in the HIL system.

### ROS Communication Network
- Publisher/Subscriber topics link the CARLA environment and the LIMO robot, enabling real-time updates of position, velocity, and control data.
- Attack resilience is achieved by validating timestamps and frequency variables in messages.

## Hardware and Software Requirements
### Hardware
- AgileX LIMO Robot
- Motion Capture System (optional, for enhanced synchronization)

### Software
- CARLA Simulator
- ROS

## Challenges and Known Limitations
- **Synchronization Accuracy**: Limited by the absence of a motion capture system.
- **ROS Network Issues**: Minor bugs in the ROS environment caused delays.
- **Legacy Code Integration**: Debugging and adapting the provided code took significant time.

## Contributors
- Cole Resurreccion
- Loren Moreira
- Mike Davidshofer
- Xushuai Zhang

## System Design Graph
![image](https://github.com/user-attachments/assets/9b421d20-5c81-4e75-930f-633b71d54e99)


## Acknowledgments
This project is part of EC 545 at Boston University. Special thanks to the course staff and referenced works for their guidance and contributions.
