# Relativistic Omni-Hybrid Vehicle (RoverDrone)

[![Godot Engine](https://img.shields.io/badge/Godot-v4.x--.NET-blue?logo=godotengine&logoColor=white)](https://godotengine.org)
[![.NET](https://img.shields.io/badge/.NET-v8.0-purple?logo=dotnet&logoColor=white)](https://dotnet.microsoft.com/)
[![C#](https://img.shields.io/badge/Language-C%23_12-green?logo=csharp&logoColor=white)](https://docs.microsoft.com/en-us/dotnet/csharp/)
[![License](https://img.shields.io/badge/License-Apache2-yellow.svg)](LICENSE)
[![Channel](https://img.shields.io/badge/YouTube-NoCapTheorem-red?logo=youtube&logoColor=white)](https://www.youtube.com/@NoCapTheorem)

> **A high-fidelity, relativistic hybrid vehicle simulation built in C# and Godot 4.**
> Modeled with continuous topological state modification between an Ackermann-steered land vehicle (Car) and a 6-DoF quadcopter (Drone). Features N-body gravitational fields, non-Newtonian fluid drag, progressive non-linear suspension, sub-stepped tensile grappling, and gyroscopic angular momentum conservation.

---

## Quick Start & Installation

### Prerequisites
* **Godot Engine v4.x** (specifically the **.NET edition**)
* **.NET 8.0 SDK** (or higher)

### Build & Run
Clone the repository and compile/run the C# solution:

```bash
# Build C# solution and execute in Godot
dotnet build && godot --headless --build-solutions --verbose Main.tscn
```

---

## License & Channel

Distributed under the **Apache2 License**. See `LICENSE` for more information.

* **No-Cap Theorem Platform:** For deep-dive architectural breakdowns, formal specs, and video walkthroughs on systems engineering and simulation physics, check out **No-Cap Theorem**.

---

---

# Technical Specification & Documentation

## 1. What Is It Simulating?

This codebase simulates a **Relativistic Omni-Hybrid Vehicle** capable of continuous topological state modification operating within a dynamic, non-linear environment. Rather than relying on simplified arcade movement, it models complex physical interactions spanning fluid dynamics, local reference frames, and conservation of angular momentum.

### Key Systems Simulated

* **Topological Gravity & N-Body Superposition:** Local dynamic reference frames derived from $N$-body gravitational fields, incorporating Plummer softened anomalies and surface-normal alignment for wall riding.
* **Non-Newtonian Fluid Dynamics:** Volumetric atmospheric drag models where fluid density scales across non-linear altitude thresholds (e.g., submersion) and dynamic cross-sectional area projection.
* **Non-Linear Progressive Suspension & Tire Grip:** Independent suspension struts evaluated via localized ray/sphere sweeps combining cubic spring rates and linear velocity-damping. Incorporates simplified Pacejka lateral slip modeling.
* **Gyroscopic Precession & CMG (Control Moment Gyroscopes):** Physical simulation of angular momentum ($L = I\omega$) stored in rotor bodies. Includes gyroscopic cross-coupling torques and active momentum discharge for kinetic braking.
* **Sub-Stepped Tensile Grappling:** High-stiffness spring solving for tether attachments, utilizing decoupled sub-stepping integrators to prevent numerical explosion.

---

## 2. Technical Breakdown

### Component 1: Topological Gravity & N-Body Superposition

The net gravitational acceleration field vector $\vec{g}_{\text{net}}$ acting on the chassis mass is modeled as a non-homogeneous superposition of a local directional field (subject to surface geometry alignment) and discrete point-mass fields (Plummer spheres).

$$\vec{g}_{\text{net}} = \vec{g}_{\text{base}} + \sum_{i} \frac{G \cdot M_i}{\Vert{}\vec{r}_i\Vert{}^2} \hat{r}_i + \frac{G \cdot M_{\text{anomaly}}}{\Vert{}\vec{r}_{\text{anomaly}}\Vert{}^2 + \epsilon^2} \hat{r}_{\text{anomaly}}$$

To prevent discontinuous orientation flipping within the solver, the system integrates the active gravity frame orientation via a spherical linear interpolation (Slerp) proxy across frames:

$$\vec{g}_{t} = \text{Lerp}\left(\vec{g}_{t-\Delta t},\, \vec{g}_{\text{target}},\, 8.0 \cdot \Delta t\right)$$

### Component 2: Volumetric Fluid Dynamics (Non-Newtonian Drag)

The atmospheric vehicle drag model departs from a static coefficient by continuously computing a profile area projection based on the velocity vector's orientation relative to the chassis alignment.

$$F_{\text{drag}} = \frac{1}{2} \rho_{\text{fluid}} \cdot \Vert{}\vec{v}\Vert{}^2 \cdot C_d \cdot A_{\text{dynamic}}$$

The cross-sectional reference area updates based on flight pitch or skid angles:

$$A_{\text{dynamic}} = \text{Lerp}\left(W_{\text{track}} \cdot 0.4,\, W_{\text{track}} \cdot L_{\text{wheelbase}},\, \left\vert{}\hat{y}_{\text{chassis}} \cdot \hat{v}\right\vert{}\right)$$

### Component 3: Non-Linear Progressive Suspension

Each intact wheel calculates a restoring vertical force $F_{\text{suspension}}$ that is non-linear, introducing a cubic spring rate alongside a linear velocity-damping term:

$$F_{\text{suspension}} = \max\left(0,\, \left(x \cdot k_{\text{base}} + x^3 \cdot k_{\text{progressive}} + \dot{x} \cdot c\right) \cdot (1 - \gamma)\right)$$

Where $\gamma \in [0, 1]$ represents the state transition progress variable from Car to Drone. Lateral grip relies on a simplified Pacejka magic formula to determine traction slip angles and apply appropriate restorative lateral forces.

### Component 4: Sub-Stepped Tensile Grappling Mechanics

When the tether lock activates, the system solves the high-stiffness spring equation using an internal loop with a smaller time step $\Delta t_{\text{sub}} = \frac{\Delta t}{10}$ to prevent numerical explosion.

$$\text{For } i = 0 \rightarrow 9: \quad \vec{r} = \vec{x}_{\text{sim}} - \vec{x}_{\text{anchor}}$$

$$\vec{F}_{\text{tether}} = \begin{cases} \vec{0} & \text{if } \Vert{}\vec{r}\Vert{} \le L_{\text{rest}} \\ -k_{\text{tether}} \left(\Vert{}\vec{r}\Vert{} - L_{\text{rest}}\right)\hat{r} - c_{\text{tether}}\left(\vec{v}_{\text{sim}} \cdot \hat{r}\right)\hat{r} & \text{if } \Vert{}\vec{r}\Vert{} > L_{\text{rest}} \end{cases}$$

### Component 5: Gyroscopic Precession & Control Moment Gyroscopes (CMG)

The four rotor bodies act as angular momentum storage units. Spinning rotors create an implicit angular momentum vector $\vec{L}_{\text{rotors}}$.

$$\vec{L}_{\text{rotors}} = \sum_{n=1}^{4} I_{\text{rotor}} \cdot \omega_n \cdot \hat{y}_{\text{chassis}}$$

When the vehicle undergoes global rigid body angular rotation, it experiences a gyroscopic precession torque. During a CMG braking event, momentum is transferred directly into the chassis by exponentially reducing the rotor angular velocities:

$$\vec{\tau}_{\text{impulse}} = \Delta \vec{L}_{\text{rotors}} = \sum_{n=1}^{4} I_{\text{rotor}} \left(\omega_{n,\,\text{initial}} - \omega_{n,\,\text{target}}\right)\hat{y}_{\text{chassis}}$$

---

## 3. Control Loop & Multi-Mode Execution

The system uses four standalone integral-bounded PID controllers to calculate stabilization forces.

```text
                +------------------------+
                |    Input Reference     |
                +------------------------+
                             |
         +-------------------+-------------------+
         |                   |                   |
         v                   v                   v
   [Target Yaw]       [Target Pitch/Roll]  [Target Vert Vel]
         |                   |                   |
         v                   v                   v
     (Yaw PID)       (Pitch/Roll PIDs)    (Vert Vel PID)
         |                   |                   |
         v                   v                   v
     {Yaw Cmd}        {Pitch/Roll Cmds}    {Thrust Base}
         |                   |                   |
         +-------------------+-------------------+
                             |
                             v
                 +-----------------------+
                 |  Mixer Allocation     |
                 +-----------------------+
                             |
         +-----------+-------+-------+-----------+
         |           |               |           |
         v           v               v           v
     [Fl Motor]  [Fr Motor]      [Rl Motor]  [Rr Motor]
```

The system mixes these control signals across the four motor channels utilizing an explicitly clamped allocation formulation mapping $F_{\text{base}}$, $\tau_{\text{pitch}}$, $\tau_{\text{roll}}$, and $\tau_{\text{yaw}}$.

---

## 4. How to Control It and Push It to Its Limits

### Controls Reference Table

| Category | Input / Key | Action |
| --- | --- | --- |
| **Mode Switch** | `TAB` | Continuous transition between Car & Drone modes. |
| **Throttle** | `W` / `S` | Car: Drive Fwd/Rev. Drone: Pitch Down/Up. |
| **Steering** | `A` / `D` | Car: Ackermann Steer L/R. Drone: Roll L/R. |
| **Yaw Control** | `Q` / `E` | Drone: Yaw Rate L/R. |
| **Vertical Base** | `SPACE` | Car: Suspension Jump. Drone: Increase Target Vert Vel. |
| **Descend** | `SHIFT` | Drone: Decrease Target Vertical Velocity. |
| **Evasive Man.** | `Z` / `C` | Drone: Apply lateral Dodge Impulse (Left / Right). |
| **CMG Brake** | `B` | Universal: Discharge stored rotor momentum into chassis. |
| **Grapple** | `F` | Universal: Fire/Release Sub-stepped Tensile Tether. |
| **Damage State** | `1`, `2`, `3`, `4` | Toggles FL, FR, RL, RR hub integrity / structural failure. |
| **Reset** | `R` | Universal: Resets position, velocities, and error states. |
| **UI** | `ESC` | Release/Capture Mouse Cursor. |

---

### Extreme Stress Tests & Edge Cases

#### 1. Kinetic Wall Riding under Gravitational Shift
* **Action:** Accelerate to $> 18.0\,\text{m/s}$ in Car mode and drive into the quarter-pipe structure or vertical wall segments.
* **What Happens:** Structural speed combined with surface normal alignment triggers a base gravity mutation ($\vec{g}_{\text{base}}$ shifts to $-\hat{n} \cdot 9.81\,\text{m/s}^2$). The vehicle will adhere to the wall dynamically. Slowing down below $10.0\,\text{m/s}$ causes instantaneous loss of adhesion and chaotic tumbling as gravity normalizes.

#### 2. N-Body Orbital Capture
* **Action:** Fly the Drone mode near the Gravitational Anomaly (purple sphere) floating in the level.
* **What Happens:** The $N$-Body superposition applies severe gravitational forces scaled by inverse distance squared. At $<50.0\,\text{m}$, the anomaly's warping influence lerps the core gravitational vector toward the singularity, forcing the drone's PID controllers to stabilize relative to a shifting "Down" vector, creating an orbital horizon effect.

#### 3. High-Velocity CMG Braking (The "B" Key)
* **Action:** In Drone mode, reach maximum angular velocity and RPM ($> 8000$), then suddenly initiate a sharp turn and press `B`.
* **What Happens:** The script rapidly collapses rotor angular velocity to $10\%$, transferring immense stored angular momentum as a torque impulse directly into the chassis. This causes instantaneous, physics-driven "whip" braking against rotational inertia, bypassing aerodynamic limits.

#### 4. Sub-Stepped Slingshot Traversal
* **Action:** Fire the tether (`F`) at the roof of an arch or wall while strafing laterally at high speed.
* **What Happens:** The solver subdivides the frame's $\Delta t$ by $10$, preventing the tensile spring from blowing up numerical limits. The vehicle effectively turns into a highly stiffened pendulum, pulling massive centripetal G-forces (visible on the cockpit pendulum telemetry) and redirecting momentum flawlessly.

