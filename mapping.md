# FTC Robot Hardware Mapping

## Hardware Configuration

### Control Hub Orientation
- **USB Port**: Facing LEFT
- **Sticker (Logo)**: Facing BACKWARD
- **IMU Calibration**: Set for this exact orientation

---

## Motors

### Drive Motors (Mecanum)
| Motor | Name | Purpose | Direction |
|-------|------|---------|-----------|
| FL | `FLWheelMotor` | Front Left Wheel | FORWARD (Inverted) |
| FR | `FRWheelMotor` | Front Right Wheel | FORWARD |
| BL | `BLWheelMotor` | Back Left Wheel | FORWARD (Inverted) |
| BR | `BRWheelMotor` | Back Right Wheel | FORWARD |

### Subsystem Motors
| Motor | Name | Purpose | Direction | Power |
|-------|------|---------|-----------|-------|
| Shooter Left | `ShooterMotorL` | Launch balls (shared shaft) | REVERSE | 80% (configurable) |
| Shooter Right | `ShooterMotorR` | Launch balls (shared shaft) | FORWARD | 80% (configurable) |
| Turret | `TurretMotor` | Rotate turret | FORWARD | Variable (Auto-Aim) |
| Intake | `IntakeMotor` | Collect balls | REVERSE | Variable |

---

## Servos

| Servo | Name | Purpose | Range | Default |
|-------|------|---------|-------|---------|
| Stopper | `TurretStopper` | Open/close ball gate | 0.0 - 1.0 | 0.0 (Closed) |
| Hood | `TurretHood` | Adjust shooting angle | 0.0 - 1.0 | 0.5 |

### Servo Positions
- **Stopper**: `0.0` = Closed, `1.0` = Open
- **Hood**: `0.0` = Min angle, `1.0` = Max angle

---

## Sensors

| Sensor | Name | Purpose |
|--------|------|---------|
| IMU | `imu` | Robot heading (Field-centric drive) |
| Limelight | `LimeCam` | Vision tracking (AprilTags) |
| Encoders | Built-in | Motor velocity feedback |

---

## Tunable Parameters (via FTC Dashboard)

### Drive Settings (`DriveConfig`)
| Parameter | Default | Purpose |
|-----------|---------|---------|
| `NORMAL_SPEED_SCALE` | 1.0 | Full speed multiplier |
| `SLOW_MODE_SPEED_SCALE` | 0.4 | Slow mode multiplier |
| `TURN_SCALE` | 0.8 | Rotation speed multiplier |
| `JOYSTICK_DEADZONE` | 0.08 | Deadzone to prevent drift |

### PID Tuning (`DriveConfig`)
| Parameter | Default | Purpose |
|-----------|---------|---------|
| `HEADING_P` | 0.015 | Heading correction gain |
| `HEADING_I` | 0.0 | Integral term |
| `HEADING_D` | 0.001 | Derivative term |

### Shooter Settings (`ShooterConfig`)
| Parameter | Default | Purpose |
|-----------|---------|---------|
| `SHOOTER_POWER` | 0.80 | Motor power (0.0-1.0) |
| `SHOOTER_P` | 0.001 | Motor PID gain |
| `SHOOTER_I` | 0.0 | Motor integral term |
| `SHOOTER_D` | 0.0001 | Motor derivative term |
| `MAX_LAUNCHER_RPM` | 5000 | Velocity control ceiling (RPM) |

---

## Control Map

### Controller 1 (Driver)
| Input | Action |
|-------|--------|
| **L Joystick** | Move (Forward/Strafe) |
| **R Joystick X** | Rotate Robot |
| **R Bumper** | Slow Mode (uses SLOW_MODE_SPEED_SCALE) |
| **L Bumper** | Shoot (opens stopper + auto-intake) |
| **X Button** | Toggle Manual/Auto-Aim |
| **D-Pad Up/Down** | Adjust hood angle (manual mode only) |
| **D-Pad Left/Right** | Turret rotation (manual mode only) |
| **B Button** | Swap controller priority (G1 ↔ G2) |
| **A Button** | Emergency reverse (intake reverse + shooter reverse) |

### Controller 2 (Operator)
| Input | Action |
|-------|--------|
| **Y Button** | Toggle Shooter RPM Hold Mode |
| **D-Pad Up/Down** | Tune Shooter Hold RPM |
| **D-Pad Left/Right** | Tune Hood Angle |
| **R Bumper (hold)** | Fine Tuning Step (RPM + Hood) |
| **A Button** | Emergency reverse |

---

## Notes

- **Field-Centric Drive**: Automatically uses IMU heading for orientation
- **Deadzone**: 0.08 (adjustable) prevents stick drift from causing movement
- **Slow Mode**: Reduces speed to 40% for precise positioning
- **Auto-Aim**: Tracks AprilTag #21 when enabled (requires vision)
