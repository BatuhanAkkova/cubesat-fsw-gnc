# Attitude Estimation: Multiplicative EKF (MEKF)

The CubeSat uses a **Multiplicative Extended Kalman Filter (MEKF)** for attitude determination.

## Why MEKF?

Standard Extended Kalman Filters (EKF) use additive state updates ($x_{k+1} = x_k + \Delta x$). While this works for linear spaces, it is problematic for attitude quaternions because:
1.  **Unit Norm Constraint**: Adding a correction to a quaternion doesn't guarantee the result is a unit quaternion.
2.  **Singularity & Redundancy**: A quaternion has 4 parameters but only 3 degrees of freedom. Additive updates lead to a singular covariance matrix.

The **MEKF** solves this by representing the attitude error as a 3-component rotation vector ($\delta \theta$) and applying it via quaternion multiplication:
$q_{est} = \delta q(\delta \theta) \otimes q_{nominal}$

## Filter State

The filter tracks 6 states:
-   **$\delta \theta$ (3 axes)**: Small-angle attitude error.
-   **$\beta$ (3 axes)**: Gyroscope bias estimates.

## Prediction Step

1.  **Nominal State Propagation**:
    The quaternion is propagated using the gyroscope measurement ($\omega$):
    $\dot{q} = \frac{1}{2} q \otimes \omega_{quat}$
2.  **Covariance Propagation**:
    The state transition matrix $\Phi$ is computed from the linearized dynamics:
    $F = \begin{bmatrix} -[\hat{\omega}\times] & -I \\ 0 & 0 \end{bmatrix}$
    $P_{k+1} = \Phi P_k \Phi^T + Q$

## Update Step

When a measurement (e.g., from Star Tracker or Magnetometer) is available:
1.  **Residual Computation**: The difference between the measured and estimated attitude is computed in the error space.
2.  **Kalman Gain**: $K = P H^T (H P H^T + R)^{-1}$
3.  **State Update**:
    -   $\beta_{new} = \beta_{old} + \Delta\beta$
    -   $q_{new} = \text{normalize}(\delta q(\Delta\theta) \otimes q_{old})$
4.  **Covariance Update**: Uses the Joseph form for numerical stability:
    $P = (I - KH) P (I - KH)^T + K R K^T$

## Noise Assumptions

-   **Gyro Noise**: Gaussian white noise for rate and random walk for bias.
-   **Measurement Noise**: Modeled based on sensor specification (e.g., Star Tracker $\approx 0.01^\circ$).
