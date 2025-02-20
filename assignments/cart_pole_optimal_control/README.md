# Cart-Pole Optimal Control Assignment

## [Final Video] 


https://github.com/user-attachments/assets/f8962c92-dc78-41a7-baea-dab198ccef21



## Overview
In this assignment, I implemented and analyzed an LQR controller for a cart-pole system under earthquake disturbances, ensuring the pendulum's stability while keeping the cart within constraints. By simulating seismic forces, I explored control robustness and disturbance rejection, reinforcing my understanding of optimal control for dynamic systems. This experience provided valuable insights into handling real-world perturbations, which are crucial for applications in space robotics, such as Lunar landings and orbital debris removal.

## System Description
The assignment is based on the problem formalism here: https://underactuated.mit.edu/acrobot.html#cart_pole
### Physical Setup
- Inverted pendulum mounted on a cart
- Cart traversal range: ±2.5m (total range: 5m)
- Pole length: 1m
- Cart mass: 1.0 kg
- Pole mass: 1.0 kg

### Disturbance Generator
The system includes an earthquake force generator that introduces external disturbances:
- Generates continuous, earthquake-like forces using superposition of sine waves
- Base amplitude: 15.0N (default setting)
- Frequency range: 0.5-4.0 Hz (default setting)
- Random variations in amplitude and phase
- Additional Gaussian noise

### Controller Description
The package includes a complete LQR controller implementation (`lqr_controller.py`) with the following features:
- State feedback control
- Configurable Q and R matrices
- Real-time force command generation
- State estimation and processing

Current default parameters:
```python
# State cost matrix Q (default values)
Q = np.diag([1.0, 1.0, 10.0, 10.0])  # [x, x_dot, theta, theta_dot]

# Control cost R (default value)
R = np.array([[0.1]])  # Control effort cost
```

### Earthquake Disturbance
The earthquake generator (`earthquake_force_generator.py`) provides realistic disturbances:
- Configurable through ROS2 parameters
- Default settings:
  ```python
  parameters=[{
      'base_amplitude': 15.0,    # Strong force amplitude (N)
      'frequency_range': [0.5, 4.0],  # Wide frequency range (Hz)
      'update_rate': 50.0  # Update rate (Hz)
  }]

### Building and Running
```bash
# Build the package
cd ~/ros2_ws
colcon build --packages-select cart_pole_optimal_control --symlink-install

# Source the workspace
source install/setup.bash

# Launch the simulation with visualization
ros2 launch cart_pole_optimal_control cart_pole_rviz.launch.py
```
#### Force Arrows
Two types of forces are visualized:
1. Control Forces (at cart level):
   - Red arrows: Positive control force (right)
   - Blue arrows: Negative control force (left)

2. Earthquake Disturbances (above cart):
   - Orange arrows: Positive disturbance (right)
   - Purple arrows: Negative disturbance (left)

Arrow lengths are proportional to force magnitudes.

### Cart-Pole LQR Tuning - Test Cases and Observations

| **Test Case** | **Q Matrix**            | **R Matrix** | **System Behavior** | **Logic / Cause of Outcome** |
|--------------|--------------------------|-------------|---------------------|------------------------------|
| 1  | [2, 1, 20, 15]       | [0.1]  | Highly unstable, pole falls within 2 seconds. | Low Q values result in weak state feedback, and R is too small, making control effort weak. |
| 2  | [5, 1.5, 50, 20]     | [0.1]  | Slightly improved, but cart oscillates aggressively. | Increased Q[2] (pole angle) helps, but Q[0] (cart position) is still low, causing instability. |
| 3  | [10, 5, 100, 30]     | [0.2]  | Pole lasts longer (~5 seconds), but oscillations grow. | Higher Q[2] reduces pole falling, but Q[0] and Q[3] (angular velocity) are still insufficient. |
| 4  | [15, 5, 200, 50]     | [0.2]  | Moderate stability, but cart still overshoots. | Cart movement is still aggressive due to low Q[0], and R is too small to dampen control efforts. |
| 5  | [20, 10, 500, 100]   | [0.3]  | Cart stabilizes slightly, pole remains up longer (~10 seconds). | Q[2] and Q[3] help the pole stay up longer, but cart position still drifts. |
| 6  | [30, 10, 700, 150]   | [0.3]  | Better damping, but slight overcorrections. | Higher Q[2] stabilizes the pole more, but low R means excessive control force, leading to jitter. |
| 7  | [40, 5, 1000, 250]   | [0.35] | Stable for 20+ seconds, oscillations reduced. | Higher Q[3] stabilizes angular velocity, and moderate R helps prevent excessive control inputs. |
| 8  | [45, 2, 1200, 300]   | [0.38] | Nearly stable, slight cart drift. | Balancing control force and state weight helps, but further fine-tuning needed. |
| 9  | [48, 1, 1400, 350]   | [0.39] | Almost ideal, small oscillations remain. | Fine-tuning Q[2] and Q[3] further improves response, but slight oscillations persist. |
| 10 | [50, 1, 1500, 400]   | [0.4]  | **Fully stable! Pole stands indefinitely.** | Optimized balance: High Q[2] & Q[3] keep the pole stable, Q[0] minimizes cart movement, and R prevents excessive control force. |

### 🏁 Final Tuning Values

- **Q Matrix:** [50, 1, 1500, 400]
- **R Matrix:** [0.4]
- **Performance:** Stable cart-pole system with minimal cart movement and no overshooting.

---
### Graph:
Final Case 


![image](https://github.com/user-attachments/assets/a0908078-502e-4035-a33b-b9379718006a)


## Conclusion
Through systematic fine-tuning of the LQR cost matrices 
𝑄 and R, I observed a clear correlation between state weighting and system stability. Initially, assigning higher values to cart position and velocity led to excessive oscillations, while prioritizing pole angle and angular velocity helped maintain an upright position for longer durations. However, overly aggressive tuning caused instability, as seen in cases where the cart overcompensated and led to rapid divergence.

By iterating through multiple test cases, I identified that increasing the weight on pole angle and angular velocity significantly improved stability, preventing the pole from tipping over prematurely. Additionally, setting a moderate control effort cost (R) ensured smooth force application without excessive corrections that could destabilize the system.

The final optimized values, 
𝑄=[50,1,1500,400] and 𝑅=[0.4] resulted in the most stable configuration, allowing the cart-pole system to remain balanced for extended durations. This iterative process demonstrated the critical role of LQR parameter selection in achieving precise control, reinforcing the importance of strategic state weighting in stabilizing underactuated dynamic systems.
