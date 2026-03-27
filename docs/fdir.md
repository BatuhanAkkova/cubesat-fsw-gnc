# FDIR: Fault Detection, Isolation, and Recovery

The **FDIR Manager** ensures mission continuity by monitoring system health and executing autonomous recovery actions.

## Fault Detection

The system uses `SensorHealthMonitor` templates to detect anomalies in real-time:
- **Stuck Sensing**: Detects if a sensor output remains frozen (constant value) for a specified duration.
- **Range Checks**: Verifies that measurements stay within physical bounds (e.g., Magnetometer field magnitude).
- **Spike Detection**: Monitors the rate of change between samples to identify unphysical sensor jumps ("glitches").

## Redundancy Management

The FDIR system supports primary/backup sensor pairs:
- If the **Primary Sensor** (e.g., Gyro 1) is marked as `FAILED`, the system automatically fails over to the **Backup Sensor** (Gyro 2).
- Continuous monitoring of both sensors determines availability.

## Recovery Actions

Based on the severity of the fault, the `FDIRManager` triggers specific actions:

| Fault Severity | Action | Recovery Target |
| :--- | :--- | :--- |
| **Degraded** | Transition to `DEGRADED` mode | Continue mission with reduced confidence. |
| **Critical** | Failover to backup sensor | Maintain `NOMINAL` mode if backup is healthy. |
| **Fatal (Critical Failure)** | Transition to `SAFE` mode | Detumble and wait for ground instruction. |
| **Estimation Divergence** | Reset MEKF state | Re-converge using current measurements. |

## Common Failure Modes

1.  **Gyro Slew Spike**: Often caused by thermal transients or radiation effects. Detected by the rapid change threshold.
2.  **Magnetometer Corruption**: Occurs during High Torque commands (magnetic interference). Filtered by the Range Check.
3.  **MEKF Divergence**: Detected if the covariance trace exceeds a safety limit.
