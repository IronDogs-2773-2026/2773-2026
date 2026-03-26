# Swerve Drive — Motor & Encoder Behavior

## Architecture Overview

The drive system consists of four independent swerve modules arranged in a square chassis. Each module independently controls its own wheel direction and speed, enabling full holonomic motion (translate and rotate simultaneously in any direction).

```
         Front
   [FL] -------- [FR]
    |               |
    |    (center)   |
    |               |
   [BL] -------- [BR]
         Back
```

**Module hardware:**
- Two brushless motors (drive + steer), each on a separate motor controller
- One absolute magnetic encoder (on the steer axle)
- One relative encoder (built into the drive motor controller)

---

## Module Locations

All positions are in meters, measured from the robot center:

| Module | X (fwd+) | Y (left+) |
|--------|----------|-----------|
| Front Left  | +0.283 | +0.281 |
| Front Right | +0.283 | −0.281 |
| Back Left   | −0.283 | +0.281 |
| Back Right  | −0.283 | −0.281 |

---

## Drive Motor

Controls wheel speed (forward/backward). There is no closed-loop feedback on the drive motor — it is driven open-loop by a normalized voltage.

**Speed conversion:**

```
voltage_fraction = desired_speed_m_per_s / MaxDriveSpeed
```

Where `MaxDriveSpeed = 0.6 m/s`. Output is clamped to `[−1.0, 1.0]`.

**Distance tracking** (relative encoder):

```
distance_m = raw_encoder_ticks / 0.047964 × (1/6.75) × π × 0.1016
```

- `0.047964` — encoder resolution constant (42 pulses/revolution / 1024 ticks)
- `6.75` — gear reduction ratio (motor turns 6.75× per one wheel rotation)
- `π × 0.1016` — wheel circumference in meters (4-inch wheel diameter)

**Velocity** (for odometry):

```
velocity_m_per_s = raw_velocity_rpm / 0.047964 × (1/6.75) × π × 0.1016 / 60
```

---

## Steer Motor

Controls which direction the wheel faces. Driven by a proportional-only controller (P = 0.30, I = 0, D = 0). Output is clamped to ±0.70 (70% of bus voltage) to protect the mechanism.

```
steer_output = clamp(P × angle_error, −0.70, +0.70)
```

The setpoint is always 0 — the `angle_error` is calculated externally and passed in as the measured value.

---

## Absolute Encoder (Wheel Heading)

The absolute encoder returns normalized rotations in `[0, 1)`, counter-clockwise positive. Each module has a **calibration offset** (`alpha`) determined at installation to correct for physical mounting angle.

**Adjusted heading (used during driving):**

```
raw      = encoder.getAbsolutePosition()   // [0, 1) rotations, CCW+
adjusted = raw − alpha                     // remove mounting offset
adjusted = wrap(adjusted, −0.5, +0.5)      // normalize
heading  = −adjusted × 2π                 // convert to radians; negate for CW+ convention
```

**Adjusted heading (used for odometry):**

Same as above, then applies an additional `−π/2` rotation and a final negation to align with the WPILib coordinate frame:

```
heading_odometry = −(−π/2 + heading_driving)
```

**Calibration offsets per module:**

| Module | Alpha |
|--------|-------|
| Front Left  | +0.35522 |
| Front Right | −0.48657 |
| Back Left   | +0.3686  |
| Back Right  | −0.1597  |

---

## Steering Optimization

Before applying any motion command, the module checks whether it would be more efficient to reverse the drive direction and steer to the opposite heading (reducing the required rotation by up to 180°).

```
if angle_error > +π/2:
    flip drive direction
    angle_error −= π

if angle_error < −π/2:
    flip drive direction
    angle_error += π
```

This ensures the steer motor never rotates more than 90° to reach a new heading.

---

## API Reference

### `SwerveDriveModule`

> `src/main/java/frc/robot/SwerveSubsystems/SwerveDriveModule.java`

Represents one swerve module (one drive motor, one steer motor, one absolute encoder).

---

#### `directionalDrive(double speed, double angle)`

Drives the module at the given speed toward the given field-relative angle.

- `speed` — normalized drive output, `[−1.0, 1.0]`
- `angle` — desired wheel heading in radians

**Behavior:**
1. Reads the absolute encoder and subtracts `alpha` offset.
2. Computes the angular error to the target heading.
3. Applies steering optimization (flip drive if error > 90°).
4. Runs a P-only controller on the remaining error to produce steer motor output.
5. Sets the drive motor to `speed × direction` (direction is ±1 after optimization).

---

#### `setDesiredState(SwerveModuleState state)`

Drives the module to the given `SwerveModuleState` (WPILib format). Used by PathPlanner and autonomous routines.

- `state.speedMetersPerSecond` — desired wheel speed
- `state.angle` — desired wheel heading as a `Rotation2d`

**Behavior:**
1. Calls `SwerveModuleState.optimize()` using the odometry-adjusted heading.
2. Computes angular error using `MathUtil.angleModulus`.
3. Applies P-only steering controller with ±0.70 clamp.
4. Converts speed from m/s to voltage fraction via `speed / MaxDriveSpeed`.

---

#### `drive(double speed, double rotate)`

Raw open-loop control. Sets drive and steer motors directly without any feedback.

- `speed` — drive motor output, `[−1.0, 1.0]`
- `rotate` — steer motor output, `[−1.0, 1.0]`

No encoder feedback is used.

---

#### `reset()`

Runs the steering PID once from the current absolute heading toward zero error, then writes that output to the steer motor. Equivalent to a one-shot nudge toward the home/straight position.

---

#### `distanceEncoderPosition() → double`

Returns the cumulative distance the wheel has traveled in meters since the last reset.

---

#### `steerAngle() → double`

Returns the current absolute wheel heading in radians, corrected for the module's calibration offset. Used internally by `directionalDrive`.

---

#### `steerAngleWPILib() → double`

Returns the current absolute wheel heading in radians, corrected for the calibration offset **and** the WPILib odometry frame transformation. Used internally by `setDesiredState` and `getSwervePosition`.

---

#### `getSwervePosition() → SwerveModulePosition`

Returns the current module state for odometry:
- `distanceMeters` — accumulated drive distance
- `angle` — current absolute heading (driving convention)

---

#### `getSwerveState() → SwerveModuleState`

Returns the current module velocity state:
- `speedMetersPerSecond` — current wheel speed derived from drive motor velocity
- `angle` — current absolute heading (odometry convention)

---

#### `setPIDValues(double p, double i, double d)`

Replaces the steering PID gains at runtime. Allows live tuning without redeploying.

---

### `DriveSubsystem`

> `src/main/java/frc/robot/SwerveSubsystems/DriveSubsystem.java`

Coordinates all four modules. Provides the primary drive API consumed by commands and autonomous routines.

---

#### `directionalDrive(double speed, double angle)`

Sends all four modules to the same heading at the same speed. Translation only; no rotation.

- `speed` — normalized drive output
- `angle` — target heading in radians (robot-relative)

---

#### `directionalDrive(double speed, double angle, double rotation)`

Full holonomic drive. Combines a translation vector (speed + angle) with a rotation rate by vector addition.

- `speed` — translation magnitude, normalized
- `angle` — translation direction in radians
- `rotation` — rotation rate, normalized; positive = counter-clockwise

Each module's command is the vector sum of the translation vector and a tangential rotation vector pointing in the direction of that module's contribution to turning. This is equivalent to what WPILib's kinematics produces, but computed manually via polar arithmetic.

---

#### `driveRobotRelative(ChassisSpeeds speeds)`

Converts a `ChassisSpeeds` (vx, vy, ω) into individual `SwerveModuleState` objects via `SwerveDriveKinematics`, desaturates if any module exceeds `MaxDriveSpeed`, and calls `setDesiredState` on each module.

Used by PathPlanner.

---

#### `getRobotRelativeSpeeds() → ChassisSpeeds`

Reads current module states via `getSwerveState()` on each module and converts them back to a robot-relative `ChassisSpeeds` using kinematics. Provides velocity feedback to PathPlanner.

---

#### `rotate(double speed)`

Points all four wheels tangentially (45° offsets from center) and drives them at `speed`, causing the robot to spin in place.

- `speed` — rotation power, normalized; sign controls direction

---

#### `carDrive(double rotationFactor, double speed)`

Ackermann (car-style) steering. Each wheel is assigned an angle and speed based on a computed turn radius, mimicking how a car turns. This is distinct from holonomic rotation.

- `rotationFactor` — curvature; `0` is straight, large values are tight turns
- `speed` — forward speed, normalized

---

#### `resetMotors()`

Runs `reset()` on each module — applies a single PID correction step toward zero heading on each steer motor.

---

#### `stop()`

Cuts power to all motors immediately (calls `stopMotor()` on each module).

---

#### `averageDistanceEncoder() → double`

Returns the mean of the four modules' cumulative drive distances in meters. Useful as a scalar distance estimate when traveling in a straight line.

---

#### `getPositions() → SwerveModulePosition[]`

Returns `[FL, FR, BL, BR]` positions (distance + heading) for odometry input.

---

#### `getStates() → SwerveModuleState[]`

Returns `[FL, FR, BL, BR]` velocity states (speed + heading) for odometry and PathPlanner feedback.

---

#### `setPID(double p, double i, double d)`

Broadcasts new steering PID gains to all four modules simultaneously.

---

#### `initAutoBuilder(OdometrySubsystem odomSub)`

Registers this subsystem with PathPlanner's `AutoBuilder`. Configures:
- Robot mass: 50 kg, MOI: 6.0 kg·m²
- Wheel radius: 0.0508 m, max speed: 0.5 m/s, coefficient of friction: 1.2
- Drive gearing: 6.75:1, current limit: 40 A
- Translation PID: P = 0.5, Rotation PID: P = 0.5

The drive consumer converts PathPlanner's field-relative `ChassisSpeeds` to robot-relative before passing to `driveRobotRelative`.

---

### `OdometrySubsystem`

> `src/main/java/frc/robot/Information/OdometrySubsystem.java`

Maintains robot pose estimate by fusing module positions with gyroscope data. Optionally incorporates vision measurements.

---

#### Periodic behavior

Every robot loop (~20 ms):
1. Reads the NavX gyro angle in radians (negated to match WPILib convention).
2. Calls `m_poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions)`.
3. Publishes `X`, `Y` to SmartDashboard and updates the `Field2d` widget.

---

#### `getPose() → Pose2d`

Returns the most recent estimated robot pose (x meters, y meters, heading radians).

---

#### `resetPose(Pose2d newPose)`

Resets the pose estimator to `newPose`. Also re-seeds the estimator with the current gyro angle and module positions.

---

#### `resetGyro()`

Zeros the NavX IMU and re-seeds the pose estimator without changing the stored pose. Called at the start of each manual drive command.

---

#### `addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs)`

Injects an AprilTag-derived pose measurement into the estimator with associated standard deviations. The estimator Kalman-filters this against accumulated wheel odometry to reduce drift.

---

#### `getGyroAngle() → double`

Returns the current gyro angle in radians.

---

#### `getX() / getY() → double`

Returns the current estimated X or Y position in meters from the pose estimator.

---

## Control Flow Summary

```
Teleop input (joystick/xbox)
  └─► DriveSubsystem.directionalDrive(speed, angle, rotation)
        └─► per module: SwerveDriveModule.directionalDrive(speed, angle)
              ├─ Reads absolute encoder → steerAngle()
              ├─ Computes error, applies optimization (flip if >90°)
              ├─ P-controller → steer motor
              └─ Speed × direction → drive motor

Autonomous (PathPlanner)
  └─► DriveSubsystem.driveRobotRelative(ChassisSpeeds)
        ├─ kinematics.toSwerveModuleStates()
        ├─ desaturateWheelSpeeds()
        └─► per module: SwerveDriveModule.setDesiredState(state)
              ├─ SwerveModuleState.optimize() using odometry heading
              ├─ P-controller on angular error → steer motor
              └─ speed / MaxDriveSpeed → drive motor

Odometry (every loop)
  └─► OdometrySubsystem.periodic()
        ├─ Gyro angle (NavX, negated)
        ├─ Module positions (distance + absolute heading)
        └─► SwerveDrivePoseEstimator.updateWithTime()
              └─ Optional: addVisionMeasurement() from AprilTags
```
