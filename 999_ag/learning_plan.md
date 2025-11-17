# 5-Week Learning Plan: PD → MPC for Wheeled Balancing

**Goal:** Master wheeled balancing control from fundamentals to advanced MPC

**Application:** Tri-ped wheelchair with stand-up capability

---

## **WEEK 1: PD Fundamentals & Code Deep Dive**

**Goals:** Master simple PD, understand every component

### **Day 1-2: Code Analysis**
1. **Read & run:** `examples/pd_balancing.py`
   - Modify each gain (10.0, 1.0, 0.1) one at a time
   - Document what breaks, what improves
   - Try extreme values (0, 100, negative)

2. **Experiments:**
   - Remove position term → observe drift
   - Remove velocity term → observe oscillation
   - Remove pitch term → watch it fall

### **Day 3-4: Observation Pipeline**
1. **Study sensors:**
   - Read: `upkie/cpp/observers/BaseOrientation.cpp` (pitch calculation)
   - Read: `upkie/cpp/observers/WheelOdometry.cpp` (position/velocity)
   - Understand data flow: IMU → observers → Python

2. **Print everything:**
   ```python
   print(f"Pitch: {pitch:.3f}, Pos: {ground_position:.3f}, Vel: {ground_velocity:.3f}")
   print(f"Action: {action}")
   ```
   - Watch values in real-time
   - Understand the relationship

### **Day 5-7: C++ PD Controller → Incremental Python Implementation**

**Goal:** Replicate production C++ controller features one-by-one in Python

**Reference Code:** `upkie/cpp/controllers/WheelBalancer.cpp`

---

#### **Phase 1: Understand the C++ Code (Day 5 Morning)**

**1. Read WheelBalancer.cpp completely:**
   - Identify all constants (lines 12-16)
   - Understand the `read()` function (lines 36-87)
   - Understand the `write()` function (lines 89-111)

**2. Map C++ features to your Python baseline:**

| Feature | Python Baseline | C++ Production | Status |
|---------|----------------|----------------|--------|
| Controller type | PD | **PI** (with integral) | ❌ Missing |
| Air handling | None | Soft-reset with low-pass filter | ❌ Missing |
| Integral clamping | N/A | ±10 m/s | ❌ Missing |
| Target distance limit | None | ±1m from current | ❌ Missing |
| Non-minimum phase trick | No | Negative velocity | ❌ Missing |
| Gain scaling (turning) | N/A | 2× → 4× when turning | ❌ Missing |
| Fall detection | Reset on terminate | Early exit if pitch > threshold | ⚠️ Partial |

**3. Create annotated copy:**
   - Copy `pd_balancing_ag.py` → `pd_balancing_pi.py`
   - Add comments mapping to C++ lines
   - Ready for incremental changes

---

#### **Phase 2: Add Integral Term - PI Controller (Day 5 Afternoon)**

**File:** `examples/pd_balancing_pi.py`

**What to add:**
```python
# Initialize before main loop
integral_position = 0.0
dt = 0.001  # 1ms at 1000 Hz

# Inside control loop
position_error = 0.0 - ground_position  # Target is origin
integral_position += position_error * dt
integral_position = np.clip(integral_position, -10.0, +10.0)

# Modified action
action = np.clip(
    a=[
        10.0 * pitch                    # P: pitch
        + 1.0 * ground_position         # P: position
        + 0.1 * ground_velocity         # D: velocity
        + 0.5 * integral_position       # I: accumulated error (NEW!)
    ],
    a_min=-0.99,
    a_max=0.99,
)
```

**C++ Reference:** Lines 64-66

**Experiment 1: Test integral windup**
- Remove clamping: `# integral_position = np.clip(...)`
- Push robot hard → integral explodes
- Robot goes unstable
- **Learning:** Why clamping is essential

**Experiment 2: Test steady-state error elimination**
- Tilt simulator floor (if possible) or add constant bias
- PD version: settles with offset
- PI version: drives error to zero
- **Learning:** Integral eliminates steady-state error

**Experiment 3: Tune integral gain**
- Try Ki = 0.1, 0.5, 1.0, 2.0
- Too low: slow convergence
- Too high: oscillation
- **Learning:** Integral gain tuning trade-offs

**Deliverable:** Working PI controller, understand integral behavior

---

#### **Phase 3: Add Air Handling (Day 6 Morning)**

**What to add:**
```python
# Get floor contact from observation
floor_contact = info["spine_observation"]["floor_contact"]["contact"]

# Inside control loop
if floor_contact:
    # Normal operation: accumulate integral
    integral_position += position_error * dt
    integral_position = np.clip(integral_position, -10.0, +10.0)
else:
    # In air: soft-reset integral with exponential decay
    decay_rate = 0.95  # Equivalent to 1s time constant
    integral_position *= decay_rate
```

**C++ Reference:** Lines 63-80 (floor contact check and low-pass filter)

**Note:** C++ uses `low_pass_filter()` function, we use simpler exponential decay for now

**Experiment 1: Test without air handling**
- Comment out the `else` block
- Lift robot (push up) → wheels spin freely
- Integral accumulates incorrectly
- Landing is unstable
- **Learning:** Why air detection matters

**Experiment 2: Compare instant vs soft reset**
- Instant reset: `integral_position = 0.0`
- Soft reset: `integral_position *= 0.95`
- Soft is smoother, handles sensor glitches
- **Learning:** Soft-reset robustness

**Experiment 3: False positive handling**
- Simulate brief contact loss (flicker)
- Soft-reset preserves state
- **Learning:** Filter design matters

**Deliverable:** PI controller with air handling

---

#### **Phase 4: Add Fall Detection (Day 6 Afternoon)**

**What to add:**
```python
# Constants
FALL_PITCH_THRESHOLD = 1.0  # rad (~57 degrees)

# Inside control loop, BEFORE calculating action
if abs(pitch) > FALL_PITCH_THRESHOLD:
    # Robot is falling, don't try to control
    action = np.array([0.0])
    observation, reward, terminated, truncated, info = env.step(action)
    if terminated or truncated:
        observation, info = env.reset()
        integral_position = 0.0  # Reset integral on episode reset
    continue  # Skip rest of control loop
```

**C++ Reference:** Lines 47-50

**Experiment 1: Test fall threshold**
- Try different thresholds: 0.5, 1.0, 1.5 rad
- Too low: gives up too early
- Too high: fights losing battle
- **Learning:** When to give up gracefully

**Experiment 2: Integral reset on fall**
- Don't reset integral on fall/reset
- Next episode starts with wrong integral
- **Learning:** State management matters

**Deliverable:** Robust fall detection and recovery

---

#### **Phase 5: Add Target Distance Limiting (Day 7 Morning)**

**What to add:**
```python
# Initialize
target_ground_position = 0.0
MAX_TARGET_DISTANCE = 1.0  # meters

# When updating target (if you add velocity commands later)
target_ground_position += target_ground_velocity * dt
target_ground_position = np.clip(
    target_ground_position,
    ground_position - MAX_TARGET_DISTANCE,
    ground_position + MAX_TARGET_DISTANCE
)

# Use target in error calculation
position_error = target_ground_position - ground_position
```

**C++ Reference:** Lines 67-70

**Note:** Current Python uses fixed target (origin). This prepares for future joystick control.

**Experiment:**
- Set target_ground_position = 10.0 (far away)
- Without limiting: robot fights hard, unstable
- With limiting: gracefully approaches max distance
- **Learning:** Reachability constraints

**Deliverable:** Understand target limiting (even if not actively used yet)

---

#### **Phase 6: Non-Minimum Phase Trick (Day 7 Afternoon)**

**What to add:**
```python
# Current: action = ... (normal calculation)

# Add trick: negate the target velocity term
target_ground_velocity = 0.0  # Currently always zero
trick_velocity = -target_ground_velocity  # Negative!

# Recalculate action with trick
action = np.clip(
    a=[
        trick_velocity                  # Note: was +target_velocity
        - 10.0 * pitch                  # Signs flip with trick
        - 1.0 * ground_position
        - 0.1 * ground_velocity
        - 0.5 * integral_position
    ],
    a_min=-0.99,
    a_max=0.99,
)
```

**C++ Reference:** Lines 83-84

**Experiment:**
- Compare with/without trick (change sign)
- Measure settling time, overshoot
- Subtle difference, improves stability
- **Learning:** Advanced control theory tricks

**Note:** This is the most subtle feature. Don't worry if impact isn't obvious in simulation.

**Deliverable:** Complete PI controller matching C++ features

---

#### **Phase 7 (Optional): Gain Scaling for Turning**

**Challenge:** Pendulum environment has 1D action (ground velocity only), no yaw control

**Options:**
1. **Understand conceptually** (recommended):
   - Read C++ lines 101-110
   - Understand: turning increases leg stiffness
   - Skip implementation (not applicable to Pendulum)

2. **Upgrade to Servos environment:**
   - Switch to `Upkie-PyBullet-Servos` (6-joint control)
   - Implement differential wheel velocities for turning
   - Add gain scaling to hip/knee joints
   - **Time:** +3-4 hours

**Deliverable:** Conceptual understanding, skip implementation for now

---

#### **Day 7 Summary: Compare & Document**

**Create comparison table:**

| Feature | Python Baseline | Python Enhanced | C++ Production |
|---------|----------------|-----------------|----------------|
| Integral term | ❌ | ✅ | ✅ |
| Integral clamping | ❌ | ✅ | ✅ |
| Air handling | ❌ | ✅ | ✅ |
| Fall detection | ⚠️ | ✅ | ✅ |
| Target limiting | ❌ | ✅ | ✅ |
| NMP trick | ❌ | ✅ | ✅ |
| Gain scaling | ❌ | 📝 Conceptual | ✅ |

**Final experiment:**
- Run Python baseline vs Python enhanced side-by-side
- Push, tilt, disturb robot
- Enhanced version much more robust!

**Week 1 Deliverable:**
- ✅ Complete PI controller in Python
- ✅ Understand all C++ features
- ✅ Can explain why each feature exists
- ✅ Ready for Week 2 (systematic tuning)

---

## **WEEK 2: Advanced PD & Limitations**

**Goals:** Push PD to its limits, understand when it fails

### **Day 1-3: Gain Tuning Systematically**
1. **Manual tuning method:**
   - Start all gains at 0
   - Add pitch gain until barely stable
   - Add velocity damping until smooth
   - Add position until returns to origin
   - Document the process

2. **Try different robots:**
   - Modify robot mass in config
   - Modify leg length
   - See how gains need to change

### **Day 4-5: Torque Balancing**
1. **Run:** `examples/pybullet/torque_balancing.py`
2. **Compare:** Torque vs velocity commands
3. **Understand:** Why disable velocity feedback (`kd_scale = 0`)

### **Day 6-7: Failure Modes**
1. **Reproduce Section 4 failures:**
   - Simulate wall constraint
   - Try bent leg configurations
   - Document why PD can't handle these

2. **Try to fix with PD:**
   - Can you add terms to handle these?
   - Where does simple PD fundamentally break?

**Week 2 Deliverable:** Know exactly where PD fails, why you need something better

---

## **WEEK 3: MPC Theory & Math**

**Goals:** Understand the optimization problem

### **Day 1-2: MPC Concept**
1. **Resources to study:**
   - Search: "Model Predictive Control tutorial"
   - Understand: prediction horizon, receding horizon
   - Key concept: "solve optimization every timestep"

2. **Simple example:**
   - Pencil & paper: 3-step prediction
   - Cost = sum of (state error² + control effort²)
   - Solve by hand for simple case

### **Day 3-4: Inverted Pendulum Model**
1. **Study the model:**
   - Read about inverted pendulum dynamics
   - Understand state: [position, pitch, velocity, angular_velocity]
   - Understand input: ground acceleration
   - Equations of motion (linearized)

2. **Why this model?**
   - Simplification of full robot
   - Fast enough for real-time
   - Captures essential balancing dynamics

### **Day 5-7: Quadratic Programming (QP)**
1. **Understand QP:**
   - Form: minimize ½x'Hx + f'x subject to constraints
   - What is H (Hessian), what is f?
   - How constraints work (Ax ≤ b)

2. **ProxQP solver:**
   - What library Upkie uses
   - Why QP is fast enough for real-time
   - Warm-starting concept

**Week 3 Deliverable:** Can write down MPC optimization problem on paper

---

## **WEEK 4: Upkie MPC Code**

**Goals:** Understand production MPC implementation

### **Day 1-3: Core MPC**
1. **Read:** `upkie/controllers/mpc_balancer/mpc_balancer.py`
   - Find where model is defined
   - Find where cost function is set
   - Find where constraints are added
   - Find where QP is solved

2. **Key variables:**
   ```python
   nb_timesteps = 50  # Horizon length
   timestep = 0.02    # 20ms steps
   # Cost weights
   stage_state_cost_weight
   stage_input_cost_weight
   terminal_cost_weight
   ```

3. **Trace execution:**
   - Add print statements
   - Watch predicted trajectory
   - See control input sequence
   - Observe how it changes each timestep

### **Day 4-5: MPC Agent**
1. **Read:** `agents/mpc_balancer/agent.py`
   - How it initializes MPC controller
   - How it combines MPC (wheels) + IK (legs)
   - Configuration via Gin files

2. **Run it:**
   ```bash
   python -m agents.mpc_balancer
   ```
   - Compare to PD behavior
   - Smoother? Faster response?
   - Try pushing robot (in sim)

### **Day 6-7: Height Controller (IK)**
1. **Read:** `agents/mpc_balancer/height_controller.py`
   - Understand inverse kinematics
   - How Pink library works
   - FrameTask (wheel positions) vs PostureTask (joint defaults)

2. **Why separate controllers?**
   - MPC for wheels (balancing)
   - IK for legs (posture)
   - They work independently

**Week 4 Deliverable:** Can modify MPC parameters and predict effect

---

## **WEEK 5: Experiments & Customization**

**Goals:** Make MPC do what you want

### **Day 1-2: Modify Cost Function**
1. **Experiment with weights:**
   - Increase terminal cost → more aggressive reaching goal
   - Increase input cost → smoother, gentler control
   - Increase state cost → tighter tracking

2. **Try different horizons:**
   - nb_timesteps = 20 (shorter, faster)
   - nb_timesteps = 100 (longer, more optimal but slower)

### **Day 3-4: Custom Objectives**
1. **Modify target state:**
   ```python
   # Instead of always pitch=0, try:
   target_pitch = 0.1  # Lean forward
   # Modify cost to pull toward this target
   ```

2. **Add new state to track:**
   - Can you add yaw to the model?
   - Try controlling turning with MPC

### **Day 5-7: Compare PD vs MPC**
1. **Quantitative comparison:**
   - Same scenario in both controllers
   - Measure settling time
   - Measure overshoot
   - Measure energy (sum of control effort)

2. **Qualitative:**
   - Which feels smoother?
   - Which handles disturbances better?
   - Which is easier to tune?

**Week 5 Deliverable:** Clear understanding of when/why to use MPC over PD

---

## **Daily Study Structure**

**Each day (2-3 hours):**

1. **Read code** (30 min)
   - Choose 1 file from week's list
   - Understand every function

2. **Run & experiment** (60 min)
   - Modify code
   - Test hypothesis
   - Break things intentionally

3. **Document** (30 min)
   - Write notes on what you learned
   - Add to highlights file?
   - Draw diagrams if helpful

---

## **Key Files to Master**

### **Week 1-2 (PD):**
- ✅ `examples/pd_balancing.py`
- ✅ `upkie/cpp/observers/BaseOrientation.cpp`
- ✅ `upkie/cpp/observers/WheelOdometry.cpp`
- ✅ `upkie/cpp/controllers/WheelBalancer.cpp`

### **Week 3-4 (MPC):**
- ✅ `upkie/controllers/mpc_balancer/mpc_balancer.py`
- ✅ `agents/mpc_balancer/agent.py`
- ✅ `agents/mpc_balancer/height_controller.py`
- ✅ `agents/mpc_balancer/wheel_controller.py`

### **Week 5 (Integration):**
- Review all of the above
- Start thinking about tri-ped modifications

---

## **After 5 Weeks: Ready for Design**

**You'll understand:**
- ✅ How balancing works (PD intuition)
- ✅ Why MPC is better for complex tasks
- ✅ How to modify control objectives
- ✅ How Upkie's architecture works
- ✅ How to extend to tri-ped

**Then move to:**
- Tri-ped modeling
- Multi-modal control (wheelchair vs standing)
- Stair climbing strategy
- Your custom application

---

## **Application Context**

**Goal:** Wheelchair with transformer-style stand-up capability
- Users can stand, walk, climb stairs
- Wheeled feet for efficiency
- Tri-ped configuration (3 contact points) for stability
- Multi-modal operation: wheelchair mode → standing mode

**Why this learning path:**
- PD gives foundation and intuition
- MPC needed for complex multi-phase motion
- Understanding both helps you choose right tool for each mode
- Upkie codebase provides 80% of needed functionality

**Design considerations for later:**
- Wheelchair mode: Simple tricycle control
- Standing mode: Wheeled balancing with 3rd leg support
- Stair climbing: Third leg does heavy work, wheels assist
- Safety: Constraint enforcement critical (human onboard)

---

## **Notes & Tips**

1. **Don't rush Week 1-2:** PD understanding is crucial foundation
2. **Week 3 theory is hard:** Take time, draw diagrams, work examples by hand
3. **Week 4-5 are fun:** You'll see everything click into place
4. **Ask questions:** Use highlights file to document confusion
5. **Experiment freely:** PyBullet simulation is safe - break things!

**Pace yourself:** This is aggressive (5 weeks). Can extend to 8-10 weeks if needed.

Good luck! 🚀
