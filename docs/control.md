# Attitude Control: B-dot and 3-Axis PID

The CubeSat employs a hierarchical control strategy to manage its attitude during different mission phases.

## Control Modes

### 1. Detumbling (B-dot)
When the spacecraft is deployed or after a fault, it usually has high angular rates. The **B-dot controller** uses magnetorquers to reduce these rates by interacting with the Earth's magnetic field.
- **Law**: $m = -k \dot{B}$
- **Goal**: Minimize $\omega$ until it is below a safe threshold ($< 0.1^\circ/s$).

### 2. 3-Axis Pointing (PID)
Once detumbled, the **AttitudeController** uses Reaction Wheels (RW) to achieve precise pointing (e.g., Sun or Nadir pointing).
- **Architecture**: A 3-axis decoupled PID controller.
- **State Error**: Computed using quaternion error (shortest path) between current attitude and target.
- **Gain Scheduling**: The controller switches between **Nominal** and **Large Error** gains to handle significant slew maneuvers without saturation.

## Implementation Details

### Gain Scheduling
The controller computes a gain factor based on the attitude error magnitude:
- **Small Error (< 10°)**: Uses high-bandwidth nominal gains for precision tracking.
- **Large Error (> 30°)**: Uses damped, lower-gain settings to prevent actuator saturation and overshoot during large slews.

### Rate Limiting and Damping
-   **Torque Rate Limiting**: Ensures that commanded torque changes do not exceed the structural or actuator capabilities ($0.5\, \text{Nm/s}$).
-   **Direct Rate Feedback**: A damping term $-k_{rate} \omega$ is added to the PID output to suppress oscillations.

## Stability Considerations

- **Quaternion Error**: The error quaternion $q_{err} = q_{curr}^{-1} \otimes q_{target}$ is used to avoid singularities.
- **Anti-Windup**: The PID controller includes integral clamping to prevent integrator windup during saturation.
- **Actuator Limits**: All outputs are clamped to the physical limits of the magnetorquers and reaction wheels.
