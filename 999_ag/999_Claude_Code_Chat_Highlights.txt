================================================================================
SECTION 1: Architecture Overview
================================================================================

**Explaining Upkie's Architecture:**

---

## 1. **Spine** (C++ process)

Think of it as the **"nervous system"** of the robot:
- Runs continuously in the background
- Talks directly to motors/sensors (hardware) OR simulator (PyBullet/Genesis)
- Handles low-level timing-critical tasks (reading sensors, sending commands)
- Written in C++ for **speed** and **real-time performance**

**Why separate?** Hardware control needs precise timing. C++ is faster than Python for this.

---

## 2. **Python Agents** connect to spine

Your **control logic** (the "brain"):
- Written in Python (easier to code than C++)
- Connects to the running spine process
- Sends: desired actions (e.g., "move left wheel at speed X")
- Receives: sensor data (e.g., "robot tilting 5° forward")
- Can be stopped/restarted without restarting the spine

**Flow:**
```
Python Agent → sends actions → Spine → controls motors
Python Agent ← receives state ← Spine ← reads sensors
```

---

## 3. **Gymnasium-compatible RL environments**

Provides a **standardized interface** for reinforcement learning:
- `env.reset()` - Start episode
- `env.step(action)` - Send action, get observation/reward
- Compatible with RL libraries (Stable Baselines3, etc.)

**Benefit:** Same code works for simulation AND real robot!


================================================================================
SECTION 2: Detailed Robot Capabilities
================================================================================

**Comprehensive Breakdown of What Upkie Can Do**

---

# **1. ROBOT TASKS & MOVEMENTS**

### **Balancing**
- **MPC Balancing**: Model Predictive Control, 1-second lookahead, max 3.0 m/s, fall detection at 1.0 rad
- **PD Balancing**: Simple gain-based feedback, lightweight alternative

### **Locomotion**
- **Forward/Backward**: ±1.5 m/s typical speed, smooth acceleration
- **Yaw Rotation**: In-place turning, max 1.0 rad/s

### **Posture Control**
- **Standing**: IK-based neutral pose at 200 Hz
- **Crouching**: 0.08m max height change, 0.05 m/s velocity
- **Leaning**: ±0.02m side-to-side shift

### **Trajectory Execution**
- **Playback**: CSV files with time-scaling, combines with balancing

### **Remote Control**
- **Joystick**: Left stick=move, right stick=turn, D-pad=crouch/lean, emergency stop

---

# **2. CONTROL STRATEGIES**

### **MPC (Model Predictive Control)**
- Wheeled inverted pendulum model (4 states)
- ProxQP solver, 50 timestep horizon (1 sec)
- Real-time optimization with fall detection

### **PID/PD**
- C++ implementation for speed
- Pitch/position/velocity gains
- Integral clamping, gain scaling when turning

### **Inverse Kinematics (IK)**
- Pink library (Pinocchio-based)
- 200 Hz updates, frame + posture tasks
- Controls leg configuration for height/lean

### **Reinforcement Learning**
- PPO algorithm via Stable Baselines3
- Vectorized training environments
- Domain randomization support

### **Torque-Based**
- Direct feedforward commands
- moteus protocol: τ = τ_ff + kp×error + kd×velocity_error
- 200-1000 Hz updates

---

# **3. SENSORS & OBSERVATIONS**

### **IMU**
- pi3hat onboard with UKF fusion
- Orientation (quaternion), angular velocity, linear acceleration
- Gravity-compensated outputs

### **Joint Encoders** (6 joints)
- Position (rad), velocity (rad/s), torque (N·m)
- Temperature (°C), voltage (V)
- Hips/knees: 16 Nm, wheels: 1.7 Nm

### **Observers**
- **Base Orientation**: Pitch angle, angular velocity
- **Wheel Odometry**: Ground position/velocity from wheel rotation
- **Floor Contact**: Fusion of wheel contact + leg torque signals

### **Human Interface**
- Joystick (gamepad), keyboard (arrows/WASD)

### **System**
- CPU temperature monitoring

---

# **4. PLATFORMS**

### **Real Hardware**
- Raspberry Pi 4/5 + pi3hat r4.5
- mjbots servos (4× qdd100, 2× mj5208)
- 1000 Hz control loop, real-time scheduling

### **PyBullet Simulator**
- GPU-accelerated physics
- GUI visualization, contact rendering
- Supports randomization

### **Genesis Simulator**
- Next-gen physics engine
- Differentiable, photorealistic rendering
- Faster than PyBullet

### **Mock Backend**
- No physics, instant steps
- For testing/CI/CD

---

# **5. CONFIGURATION**

### **Robot Parameters**
- Leg length: 0.58m, mass: 5.34kg, wheel radius: 0.06m
- User override: `~/.config/upkie/config.yml`

### **Spine Config**
- Floor/wheel contact thresholds
- Odometry signed radius per wheel

### **Agent Config** (Gin files)
- Height controller: crouch/lean limits
- Remote control: max velocities
- Wheel controller: fall threshold, gain scaling
- Per-robot configs by hostname

### **Environments**
- **Pendulum**: 1D action (ground velocity), 4D obs
- **Servos**: 6-joint dict action (position/velocity/torque/gains), full joint state obs

### **Joint Limits**
- Hips: ±1.26 rad, knees: ±2.51 rad
- Max velocities: 28.8 rad/s (hips/knees), 111 rad/s (wheels)

---

# **6. ADVANCED FEATURES**

### **Domain Randomization**
- Initial state perturbations (position/velocity/orientation)
- Inertia variation (±30%)
- Sensor noise (control + measurement)
- External force disturbances

### **Wrappers**
- Action: differentiate, noisify, add lag
- Observation: noisify, add action history
- Disturbance: random push

### **Logging**
- MessagePack binary format
- Full obs-action-reward history
- Microsecond timestamps

### **Optimization**
- CPU isolation, SCHED_FIFO priority
- Frequency regulation (200-1000 Hz)

### **Visualization**
- MeshCat for IK
- PyBullet/Genesis GUI modes

### **Testing**
- pytest (Python), Bazel (C++)
- Ruff + cpplint
- Mock backend for CI

### **Extensibility**
- Custom observers, controllers, sensors
- Plugin architecture
- Online reconfiguration

### **Tools**
- Raspunzel: remote build/deploy to Pi
- Bazelisk: version management


================================================================================
SECTION 3: Understanding Balancing Strategies
================================================================================

**Deep Dive: Three Balancing Approaches**

---

## **1. Torque Balancing** (Direct Control - Simplest)

**What it is:**
- Uses **feedforward torque** commands directly to wheel motors
- Simple proportional control: `torque = GAIN × pitch`
- No velocity feedback (open-loop on wheels)

**How it works:**
```python
GAIN = 10.0  # N·m/rad
pitch = observation["base_orientation"]["pitch"]
left_wheel_torque = +GAIN × pitch
right_wheel_torque = -GAIN × pitch
```

**Characteristics:**
- **Most direct**: Commands torque (N·m) to motors
- **Open-loop**: No velocity error feedback
- **Pure feedforward**: Just proportional to pitch
- **Simple**: Easiest to understand

**Example:** `examples/pybullet/torque_balancing.py`
- Gain: 10.0 N·m/rad
- Disables velocity feedback (kd_scale = 0.0)
- Legs held extended with position control

**Advantages:**
- Most direct control (no servo feedback interference)
- Easy to understand forces applied
- Good for learning control basics

**Disadvantages:**
- No built-in velocity damping
- Can be less stable than PD
- Requires good torque sensing/estimation

---

## **2. PD Balancing** (Velocity Control - Simple)

**What it is:**
- **Proportional-Derivative** feedback control
- Direct reaction: "If tilting forward → spin wheels forward"
- Pure feedback, no prediction

**How it works:**
```
wheel_torque = Kp × pitch_error + Kd × angular_velocity_error
```

**Characteristics:**
- **Fast**: Instant computation
- **Simple**: Just multiply sensor values by gains
- **Reactive**: Responds to current state only
- **Tuning**: Find good Kp, Kd gains through trial/error

**Example:** `examples/pybullet/pd_balancing.py`
- Commands velocity to wheels
- Derivative term for damping

---

## **3. MPC Balancing** (Predictive - Advanced)

**What it is:**
- **Model Predictive Control**
- Solves optimization problem: "What actions keep me balanced for next 1 second?"
- Looks ahead, plans optimal trajectory

**How it works:**
1. Predict future (50 timesteps = 1 sec)
2. Minimize cost: deviation from upright + control effort
3. Apply first action from optimal sequence
4. Repeat every timestep (receding horizon)

**Characteristics:**
- **Predictive**: Anticipates what will happen
- **Optimal**: Minimizes cost function mathematically
- **Constraints-aware**: Respects velocity/acceleration limits
- **Computationally expensive**: Solves QP (quadratic program) each cycle

**Model used:** Wheeled Inverted Pendulum
- 4 states: [position, pitch, velocity, angular_velocity]
- 1 input: ground acceleration

---

## **Key Differences:**

| Aspect | Torque | PD | MPC |
|--------|--------|----|----|
| **Control Output** | Direct torque (N·m) | Velocity (rad/s) | Ground accel (m/s²) |
| **Computation** | Instant (multiply) | Instant (multiply) | Heavy (optimization) |
| **Lookahead** | None (reactive) | None (reactive) | 1 second ahead |
| **Feedback Loops** | Pitch only | Pitch + velocity | Full state prediction |
| **Optimality** | Heuristic gain | Heuristic gains | Mathematically optimal |
| **Constraints** | Hard to enforce | Hard to enforce | Built-in (velocity limits) |
| **Tuning** | Single gain | Multiple gains | Design cost function |
| **Performance** | Basic balancing | Good for simple tasks | Best for complex motions |
| **Stability** | Lower (no damping) | Better (velocity damping) | Best (predictive) |
| **CPU Usage** | Minimal | Minimal | Higher (ProxQP solver) |

---

## **When to use each?**

**Torque Balancing:**
- Learning control theory basics
- Understanding force application
- Prototyping simplest approach
- When you need precise torque control

**PD Balancing:**
- Learning/prototyping with better stability
- Low-power systems
- Simple balancing in place
- When velocity damping is needed

**MPC Balancing:**
- Moving while balancing (locomotion)
- Respecting physical limits
- Optimal performance needed
- Research applications
- Complex multi-objective control


================================================================================
SECTION 4: PD Control Deep Dive - How It Works & Failure Cases
================================================================================

## **How IMU Measures Pitch**

**Key Concept: Pitch = ANGLE, NOT Angular Velocity**

**IMU provides TWO separate measurements:**
- **Accelerometer** → Detects gravity direction → Orientation ANGLE (pitch)
- **Gyroscope** → Detects rotation rate → Angular velocity

**Critical: Even static tilted robot has pitch ≠ 0!**
- Accelerometer senses gravity sideways → calculates tilt angle
- Robot leaning at 30° while motionless still reads pitch = 30°

**Data Flow:**
```
IMU Hardware (pi3hat r4.5)
  ↓ UKF sensor fusion (40 kHz)
Orientation (quaternion) + Angular velocity
  ↓ BaseOrientation Observer (upkie/cpp/observers/BaseOrientation.cpp)
Pitch Angle (rad)
  ↓
observation[0] in Pendulum env
```

---

## **PD Code Breakdown**

**File:** `examples/pd_balancing.py`

**Environment:** `Upkie-PyBullet-Pendulum` at 1000 Hz

**Observation Space (4D):**
```python
observation[0] = pitch (rad)           # Base tilt angle
observation[1] = ground_position (m)   # Distance traveled
observation[2] = angular_velocity      # NOT USED in this code
observation[3] = ground_velocity (m/s) # Speed
```

**Control Law:**
```python
action = np.clip(
    a=[10.0 * pitch              # P term: pitch correction
       + 1.0 * ground_position   # P term: position regulation
       + 0.1 * ground_velocity], # D term: velocity damping
    a_min=-0.99, a_max=0.99
)
```

**Term-by-Term:**
1. **10.0 × pitch**: Main balancing - "lean forward → wheels forward"
2. **1.0 × ground_position**: Return to origin - "drifted away → push back"
3. **0.1 × ground_velocity**: Damping - "moving fast → slow down" (prevents oscillation)

**Why These Gains?**
- 10.0 (pitch): Strong response, primary balancing
- 1.0 (position): Moderate, keeps near origin
- 0.1 (velocity): Gentle damping, stability without sluggishness
- Empirically tuned for this robot

---

## **Failure Case 1: Robot Against Wall**

**Scenario:** Robot at 30° against wall, motors off → switch on

**What Happens:**
```
IMU reads: pitch = 30° (accelerometer detects tilt!)
           angular_velocity = 0 (not rotating)

Controller: action = 10.0 × 0.524 = 5.24
           "Tilt 30° forward → spin wheels forward strongly!"

Result: Wheels try to spin → wall blocks motion
        → Robot stays at 30°, fighting wall indefinitely
```

**Problem:** Controller "blind" to external constraints
- Only sees pitch angle from IMU
- Doesn't know wall exists
- Keeps commanding forward motion despite no progress

**If wall removed:** Robot suddenly accelerates forward, likely overshoots and crashes

---

## **Failure Case 2: Vertical Torso, Bent Legs**

**Scenario:** Legs at 30° forward, hip bent -30° back → torso vertical but unstable

```
Configuration:
    [Torso] ← Vertical (pitch = 0°)
      /
     / ← Hip -30°
    /
   / ← Legs +30°
  /
[Wheels] ← Center of mass NOT above wheels!
```

**What Happens:**
```
IMU reads: pitch = 0° (torso IS vertical)
           angular_velocity = 0

Controller: action = 10.0 × 0 + 1.0 × 0 + 0.1 × 0 = 0
           "Pitch is zero, everything is fine!"

Result: No action commanded
        Robot in unstable equilibrium
        Tiny disturbance → starts falling
        Controller reacts only after fall begins
        Usually too late → CRASH
```

**Problem:** Simple pitch monitoring insufficient
- Torso vertical ≠ stable robot
- Center of mass shifted from wheels
- No awareness of joint positions
- Unstable configuration undetected

---

## **What's Missing in Simple PD Control**

**Controller doesn't know:**
- Joint positions (leg configuration)
- Center of mass location
- External forces/constraints (walls, objects)
- Configuration stability

**Better Approaches:**

**1. Full-State Control:**
```python
# Monitor joint angles too
if hip_angle < -20° or knee_angle > 20°:
    # Unstable detected, take action
```

**2. MPC with CoM:**
- Calculates center of mass from joint states
- Knows "pitch = 0 but CoM unstable"
- Proactively stabilizes

**3. Height Controller (IK):**
- Used in `agents/mpc_balancer/height_controller.py`
- Maintains desired leg configuration
- Prevents unstable postures
- Keeps legs extended via inverse kinematics

**File Reference:** `upkie/cpp/controllers/WheelBalancer.cpp` for C++ PD implementation

---

## **Key Takeaways**

1. **Pitch = angle from accelerometer**, detectable even when static
2. **Simple PD** balances torso pitch only, ignores joint configuration
3. **External constraints** (walls) undetectable without force sensing
4. **Vertical torso ≠ stable robot** - need joint position awareness
5. **Real agents** (MPC balancer) use IK to maintain stable postures
