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

### **Day 5-7: C++ PD Controller**
1. **Read:** `upkie/cpp/controllers/WheelBalancer.cpp`
   - More sophisticated than Python example
   - PI controller (integral term)
   - Gain scaling when turning
   - Air handling logic

2. **Compare:** What's different from simple Python version?
   - Why integral term?
   - Why gain scaling?
   - How does it handle being airborne?

**Week 1 Deliverable:** Can explain every line of PD code, predict behavior from gains

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
