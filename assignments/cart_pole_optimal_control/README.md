# Cart-Pole Optimal Control Assignment

## [Final Video] 


https://github.com/user-attachments/assets/5eca81a7-c190-4f94-b20d-1b971700ad82


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

# Cart-Pole LQR Tuning Test Cases

| Test Case | Q Values (x, ẋ, θ, θ̇) | R Value | Stability | Observations |
|-----------|---------------------|--------|-----------|-------------|
| 1  | [0.5, 0.5, 2, 2]  | [0.05]  | Highly Unstable  | Weak control effort, cart moves unpredictably, pole falls instantly.  |
| 2  | [1, 1, 5, 2]      | [0.1]   | Unstable  | Increased weight on pole angle improves response but still oscillates aggressively.  |
| 3  | [2, 2, 10, 5]     | [0.2]   | Slightly Better  | Cart moves less erratically, but overcompensates, leading to instability.  |
| 4  | [3, 3, 15, 7]     | [0.25]  | Oscillatory  | System balances for a short time before diverging due to excessive corrections.  |
| 5  | [4, 4, 20, 10]    | [0.3]   | Semi-Stable  | Improved response, but oscillations increase over time, leading to failure.  |
| 6  | [5, 5, 25, 12]    | [0.35]  | Marginally Stable  | The system stabilizes longer, but the cart overshoots, causing failure.  |
| 7  | [6, 6, 30, 15]    | [0.38]  | Near Stable  | The pole remains upright for an extended period but still oscillates slightly.  |
| 8  | [6.5, 6.5, 35, 17] | [0.39] | Almost Stable  | Small corrections lead to minor oscillations, but overall much better.  |
| 9  | [7, 7, 40, 20]    | [0.4]   | Very Stable  | System remains stable for a long time, oscillations are minimal.  |
| 10 | **[7, 7, 10, 10]** | **[0.1]** | **Optimal Stability** | **Smooth response, minimal corrections, cart and pole remain stable indefinitely.** |




### 🏁 Final Tuning Values

- **Q Matrix:** [7.0,7.0,10.0,10.0]
- **R Matrix:** [0.1]
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
𝑄=[7.0,7.0,10.0,10.0] and 𝑅=[0.1] resulted in the most stable configuration, allowing the cart-pole system to remain balanced for extended durations. This iterative process demonstrated the critical role of LQR parameter selection in achieving precise control, reinforcing the importance of strategic state weighting in stabilizing underactuated dynamic systems.
