# First-Order Boustrophedon Navigator


In this assignment, you will understand the provided code in ROS2 with Turtlesim, and refactor and/or tune the navigator to implement a precise lawnmower survey (a boustrophedon pattern). The current code will do a pattern shown above, which is not a uniform lawnmower survey. 
Explore literature on how lawnmower surveys typically look, and modify the code to meet the requirements for a uniform survey. 

### Background
Boustrophedon patterns (from Greek: "ox-turning", like an ox drawing a plow) are fundamental coverage survey trajectories useful in space exploration and Earth observation. These patterns are useful for:

- **Space Exploration**: Rovers could use boustrophedon patterns to systematically survey areas of interest, ensuring complete coverage when searching for geological samples or mapping terrain. However, due to energy constraints, informative paths are usually optimized, and this results in paths that are sparser than complete coverage sampling, and may still produce high-accuracy reconstructions. 
  
- **Earth Observation**: Aerial vehicles employ these patterns for:
  - Agricultural monitoring and precision farming
  - Search and rescue operations
  - Environmental mapping and monitoring
  - Geological or archaeological surveys
  
- **Ocean Exploration**: Autonomous underwater vehicles (AUVs) use boustrophedon patterns to:
  - Map the ocean floor
  - Search for shipwrecks or aircraft debris
  - Monitor marine ecosystems
  
The efficiency and accuracy of these surveys depend heavily on the robot's ability to follow the prescribed path with minimal deviation (cross-track error). This assignment simulates these real-world challenges in a 2D environment using a first-order dynamical system (the turtlesim robot).

### Objective
Tune a PD controller to make a first-order system execute the most precise boustrophedon pattern possible. The goal is to minimize the cross-track error while maintaining smooth motion.

### Learning Outcomes
- Understanding PD control parameters and their effects on first-order systems
- Practical experience with controller tuning
- Analysis of trajectory tracking performance
- ROS2 visualization and debugging

## Prerequisites

### System Requirements
Choose one of the following combinations:
- Ubuntu 22.04 + ROS2 Humble
- Ubuntu 23.04 + ROS2 Iron
- Ubuntu 23.10 + ROS2 Iron
- Ubuntu 24.04 + ROS2 Jazzy

### Required Packages
```bash
sudo apt install ros-$ROS_DISTRO-turtlesim
sudo apt install ros-$ROS_DISTRO-rqt*
```

### Python Dependencies
```bash
pip3 install numpy matplotlib
```

## Getting Started

### Repository Setup
1. Fork the course repository:
   - Visit: https://github.com/DREAMS-lab/RAS-SES-598-Space-Robotics-and-AI
   - Click "Fork" in the top-right corner
   - Select your GitHub account as the destination

2. Clone your fork (outside of ros2_ws):
```bash
cd ~/
git clone https://github.com/YOUR_USERNAME/RAS-SES-598-Space-Robotics-and-AI.git
```

3. Create a symlink to the assignment in your ROS2 workspace:
```bash
cd ~/ros2_ws/src
ln -s ~/RAS-SES-598-Space-Robotics-and-AI/assignments/first_order_boustrophedon_navigator .
```

### Building and Running
1. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select first_order_boustrophedon_navigator
source install/setup.bash
```

2. Launch the demo:
```bash
ros2 launch first_order_boustrophedon_navigator boustrophedon.launch.py
```

3. Monitor performance:
```bash
# View cross-track error as a number
ros2 topic echo /cross_track_error

# Or view detailed statistics in the launch terminal
```

4. Visualize trajectory and performance:
```bash
ros2 run rqt_plot rqt_plot
```
Add these topics:
- /turtle1/pose/x
- /turtle1/pose/y
- /turtle1/cmd_vel/linear/x
- /turtle1/cmd_vel/angular/z
- /cross_track_error

### Usage

1. Launch the turtlesim node:
   ```bash
   rosrun turtlesim turtlesim_node
   ```
2. Run the coverage path planning script:
   ```bash
   rosrun your_package_name coverage_path_planner.py
   ```
3. Adjust control parameters (`Kp_linear`, `Kd_linear`, `Kp_angular`, `Kd_angular`) in the `config.yaml` file to optimize performance.

---

## Parameter Tuning

Fine-tune the following parameters to achieve desired performance:

- `Kp_linear`: Proportional gain for linear velocity.
- `Kd_linear`: Derivative gain for linear velocity.
- `Kp_angular`: Proportional gain for angular velocity.
- `Kd_angular`: Derivative gain for angular velocity.
- `spacing`: Distance between adjacent paths.

---
## Tuning Process and Performance Analysis

### Performance Metrics

- **Average Cross-track Error**: This measures the average deviation from the desired trajectory across the entire path. A lower value signifies more precise control and a smaller error margin during navigation.
  
- **Maximum Cross-track Error**: This captures the largest deviation from the desired path at any given point. It is critical for evaluating extreme performance and ensuring the vehicle does not stray far from the target trajectory.

- **Smoothness of Motion**: This refers to the continuity and fluidity of the movement. A smooth motion ensures there are minimal sudden jerks or abrupt changes in direction, contributing to stability and efficiency.

- **Cornering Performance**: This evaluates the robot's ability to navigate turns or curves without excessive overshoot, oscillations, or difficulty in maintaining the desired path. Good cornering performance ensures precise navigation around sharp turns.

### Pattern Parameters

- **Coverage Efficiency**: This defines how well the robot can cover the target area within a given time, often expressed as the percentage of the area covered relative to the total area. It measures the operational effectiveness of the path.

- **Pattern Completeness**: This metric assesses how thoroughly the robot follows the expected path, ensuring no part of the desired area is left uncovered. A high completion rate reflects a well-executed trajectory following process.

### Detailed Analysis of Tuning Process

### 1. Methodology Used for Tuning
The tuning process involves adjusting the PID controller parameters — specifically **Kp_linear**, **Kd_linear**, **Kp_angular**, and **Kd_angular** — to minimize cross-track error while optimizing smoothness and cornering performance. Initially, the parameters were set with a broad range of values to understand their impact on performance. Fine-tuning was then done by systematically adjusting each parameter and observing the changes in both linear and angular behaviors.

### 2. Performance Plots and Metrics
The tuning iterations were analyzed using performance plots, such as the cross-track error over time and the smoothness of motion (which can be visualized as a time-series of velocity and angular velocity profiles). These plots helped identify which parameter adjustments led to smoother, more efficient motion and minimized deviation. Each case's performance metrics were calculated and compared across both spacing scenarios to identify the optimal settings for low error and clean cornering.

### 3. Challenges Encountered and Solutions

- **Challenge 1**: A major challenge was managing the trade-off between speed and accuracy. In some cases, increasing **Kp_linear** to improve straight-line motion led to higher cross-track error in tight turns.  
  **Solution**: This was addressed by fine-tuning **Kd_angular** to stabilize cornering behavior while maintaining a reasonable linear speed.

- **Challenge 2**: Achieving smooth cornering while maintaining a low cross-track error during sharp turns was also challenging. The robot often exhibited jerky movements or overshot turns.  
  **Solution**: By decreasing the **Kd_linear** and slightly lowering the **Kp_angular** values, the vehicle's response to changes in direction was smoother, reducing overshoot and improving the cornering performance.


## Results

### Self_Spacing 1.0

#### Images

![WhatsApp Image 2025-01-26 at 18 06 16_d7151b4d](https://github.com/user-attachments/assets/ac9c850d-4357-4a9f-a27e-e8e2b9cd7905)

#### Table

| Case Number | Kp_linear | Kd_linear | Kp_angular | Kd_angular | Cross-track Error  | Cornering Behaviour | Logic                                                                 |
|-------------|-----------|-----------|------------|------------|--------------------|---------------------|----------------------------------------------------------------------|
| Case 1      | 10.0      | 0.5       | 6.0        | 0.2         | Slightly High      | Slightly Smooth     | High Kp_linear caused fast approach; Kd_angular mitigated overshoot. |
| Case 2      | 9.0       | 1.0       | 7.0        | 0.1         | Moderate           | Smooth              | Balanced Kp/Kd values improved linear and angular response.          |
| Case 3      | 11.0      | 0.8       | 5.0        | 0.05        | High               | Slightly Rough      | Low Kd_linear allowed jerky corrections despite high Kp_linear.      |
| Case 4      | 8.0       | 0.5       | 4.0        | 0.04        | Low                | Clean               | Optimal Kp values ensured minimal error and clean cornering.         |
| Case 5      | 9.0       | 0.6       | 9.0        | 0.01        | Minimal            | Sharp               | High Kp_angular enabled sharp cornering with minimal error.          |
| Case 6      | 9.5       | 0.3       | 8.0        | 0.05        | Low                | Clean               | Balanced Kp_linear and Kp_angular ensured smooth transitions.        |
| Case 7      | 8.5       | 0.5       | 6.5        | 0.03        | Slightly High      | Slightly Rough      | Low Kd_angular led to uneven adjustments despite moderate Kp values. |

#### Best Outcome Table

| Case Number | Kp_linear | Kd_linear | Kp_angular | Kd_angular | Cross-track Error | Cornering Behaviour   |
|-------------|-----------|-----------|------------|------------|--------------------|----------------------|
| Case 5      | 9.0       | 0.6       | 9.0        | 0.01       | Minimal            | Sharp                |


### Images (Best Outcomes)
##### Turtlesim
![WhatsApp Image 2025-01-27 at 20 50 22_89a46eb5](https://github.com/user-attachments/assets/8f47931f-3136-4d56-896a-7985c8bd9cb1) 
##### Rqt_Plots
![WhatsApp Image 2025-01-27 at 20 54 00_cf5694c4](https://github.com/user-attachments/assets/39453cd1-6d89-4ad5-9e2a-3687d9461eec)
##### Cross-Track-Error (Avg- 0.056)
![WhatsApp Image 2025-01-27 at 20 56 58_348a7ce4](https://github.com/user-attachments/assets/455dedcf-9022-4b61-9cca-a9951808db8a)

---

### Self_Spacing 0.4

#### Images

![WhatsApp Image 2025-01-26 at 18 23 53_41201180](https://github.com/user-attachments/assets/1cfd222c-b524-4726-ab06-c8ffb1758015)

#### Table
| Case Number | Kp_linear | Kd_linear | Kp_angular | Kd_angular | Cross-track Error  | Cornering Behaviour | Logic                                                                 |
|-------------|-----------|-----------|------------|------------|--------------------|---------------------|----------------------------------------------------------------------|
| Case 1      | 7.0       | 0.3       | 8.0        | 0.5        | Moderate           | Clean               | Low Kd_linear limited velocity smoothing, but Kd_angular corrected angles well. |
| Case 2      | 12.0      | 0.6       | 12.0       | 0.015      | Low                | Precise             | Well-balanced values ensured minimal error and precise turns.        |
| Case 3      | 10.0      | 0.4       | 15.0       | 0.01       | Slightly High      | Slightly Rough      | Overcorrection due to high derivative values.                        |
| Case 4      | 15.0      | 0.8       | 12.0       | 0.05       | Minimal            | Smooth              | Higher Kp and Kd values reduced errors across paths.                 |
| Case 5      | 10.0      | 0.6       | 10.0       | 0.012      | Slightly High      | Slightly Rough      | Insufficient Kp_linear caused higher cross-track error.              |
| Case 6      | 9.0       | 0.5       | 11.0       | 0.03       | Minimal            | Very Sharp          | Optimized Kp_angular achieved precise cornering.                     |
| Case 7      | 8.0       | 0.8       | 9.0        | 0.04       | Low                | Slightly Rough      | Balanced Kp values ensured consistent performance.                   |

#### Best Outcome Table

| Case Number | Kp_linear | Kd_linear | Kp_angular | Kd_angular | Cross-track Error | Cornering Behaviour |
|-------------|-----------|-----------|------------|------------|--------------------|----------------------|
| Case 2      | 12.0      | 0.6       | 12.0       | 0.015      | Low                | Precise             |

#### Images (Best Outcomes)
##### Turtlesim
![WhatsApp Image 2025-01-27 at 21 16 34_49725e4b](https://github.com/user-attachments/assets/9210cad8-b467-4e20-b258-33b9d003ad8d)
##### rqt_plot
![WhatsApp Image 2025-01-27 at 21 16 34_9dce3dde](https://github.com/user-attachments/assets/e9b07a25-af2a-461e-a83c-91a0951c7da6)
##### Cross-Track-Error (Avg-0.056)
![WhatsApp Image 2025-01-27 at 21 18 01_69ac8af9](https://github.com/user-attachments/assets/cf0f852c-622d-40dd-93d8-01fce6b383e2)



---
