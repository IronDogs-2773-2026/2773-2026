# Swerve Drive — Motor & Encoder Behavior

## Architecture Overview

The drive system consists of four independent swerve modules arranged in a square chassis. Each module independently controls its own wheel direction and speed, enabling full holonomic motion (translate and rotate simultaneously in any direction).

```
         Front (motors 17, 19)
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

| Module | X (fwd+) | Y (left+) | Drive CAN | Steer CAN | Encoder CAN | Alpha offset |
|--------|----------|-----------|-----------|-----------|-------------|--------------|
| Front Left  | +0.283 | +0.281 | 17 | 16 | 52 | +0.35522 |
| Front Right | +0.283 | −0.281 | 19 | 12 | 53 | −0.48657 |
| Back Left   | −0.283 | +0.281 | 10 | 11 | 54 | +0.3686  |
| Back Right  | −0.283 | −0.281 | 22 | 23 | 55 | −0.1597  |

---

## Drive Motor

Controls wheel speed (forward/backward). Driven open-loop — no closed-loop feedback.

**Speed conversion:**

```
voltage_fraction = desired_speed_m_per_s / MaxDriveSpeed
```

`MaxDriveSpeed = 0.6 m/s`. Output is clamped to `[−1.0, 1.0]`.

**Distance tracking** (relative encoder):

```
distance_m = raw_encoder_ticks / 0.047964 × (1/6.75) × π × 0.1016
```

- `0.047964` — encoder resolution constant (42 pulses/rev ÷ 1024 ticks)
- `6.75` — gear reduction (motor turns 6.75× per wheel rotation)
- `π × 0.1016` — wheel circumference in meters (4-inch wheel diameter)

**Velocity** (for odometry):

```
velocity_m_per_s = raw_velocity_rpm / 0.047964 × (1/6.75) × π × 0.1016 / 60
```

---

## Steer Motor

Controls which direction the wheel faces. Driven by a P-only controller (P = 0.30, I = 0, D = 0). Output is clamped to ±0.70 (70% of bus voltage).

```
steer_output = clamp(0.30 × angle_error, −0.70, +0.70)
```

The PID setpoint is always 0 — the error is computed externally and passed as the measured value.

---

## Absolute Encoder (Wheel Heading)

The absolute encoder returns normalized rotations in `[0, 1)`, CCW positive. Each module has a calibration offset (`alpha`) to correct for physical mounting angle.

**`steerAngle()` — used for driving:**

```
raw      = encoder.getAbsolutePosition()   // [0, 1) rotations, CCW+
adjusted = raw − alpha                     // remove mounting offset
adjusted = wrap(adjusted, −0.5, +0.5)      // normalize to half-rotation
heading  = −adjusted × 2π                 // to radians; negate for CW+ convention
```

**`steerAngleWPILib()` — used for odometry and PathPlanner:**

Same normalization, then applies a frame transformation to align with WPILib's CCW-positive robot frame:

```
angle = −adjusted × 2π
angle = −π/2 + angle
return −angle          // = π/2 + adjusted × 2π
```

Both `getSwervePosition()` and `getSwerveState()` use `steerAngleWPILib()`, ensuring the pose estimator and PathPlanner receive angles in the correct coordinate frame.

---

## Steering Optimization

Before applying any motion command, the module checks whether reversing the drive direction (and steering to the opposite heading) would require less than 90° of rotation.

```
if angle_error > +π/2:
    flip drive direction
    angle_error −= π

if angle_error < −π/2:
    flip drive direction
    angle_error += π
```

The steer motor never rotates more than 90° to reach a new heading.

---

## API Reference

### `SwerveDriveModule`

> `src/main/java/frc/robot/SwerveSubsystems/SwerveDriveModule.java`

Represents one swerve module (one drive motor, one steer motor, one absolute encoder).

---

#### `directionalDrive(double speed, double angle)`

Drives the module at the given speed toward the given heading using closed-loop steering.

- `speed` — normalized drive output, `[−1.0, 1.0]`
- `angle` — desired wheel heading in radians

**Behavior:**
1. Reads `steerAngle()` and computes error to target heading.
2. Applies steering optimization (flip drive if error > 90°).
3. Runs P-only controller on remaining error → steer motor (clamped ±0.70).
4. Sets drive motor to `speed × direction`.

---

#### `setDesiredState(SwerveModuleState state)`

Drives the module to a WPILib `SwerveModuleState`. Used by PathPlanner.

- `state.speedMetersPerSecond` — desired wheel speed
- `state.angle` — desired wheel heading as a `Rotation2d`

**Behavior:**
1. Calls `SwerveModuleState.optimize()` using `steerAngleWPILib()`.
2. Computes angular error via `MathUtil.angleModulus`.
3. Applies P-only steering controller, clamped ±0.70.
4. Converts speed to voltage fraction via `speed / MaxDriveSpeed`.

---

#### `drive(double speed, double rotate)`

Raw open-loop control. Sets both motors directly with no encoder feedback.

- `speed` — drive motor output, `[−1.0, 1.0]`
- `rotate` — steer motor output, `[−1.0, 1.0]`

---

#### `distanceEncoderPosition() → double`

Returns cumulative wheel travel in meters since last encoder reset.

---

#### `steerAngle() → double`

Returns current wheel heading in radians, offset-corrected. Used by `directionalDrive`.

---

#### `steerAngleWPILib() → double`

Returns current wheel heading in radians in WPILib's CCW-positive robot frame. Used by `setDesiredState`, `getSwervePosition`, and `getSwerveState`.

---

#### `getSwervePosition() → SwerveModulePosition`

Returns module state for the pose estimator:
- `distanceMeters` — accumulated drive distance
- `angle` — current heading via `steerAngleWPILib()`

---

#### `getSwerveState() → SwerveModuleState`

Returns current module velocity for PathPlanner feedback:
- `speedMetersPerSecond` — current wheel speed from drive motor velocity
- `angle` — current heading via `steerAngleWPILib()`

---

#### `stop()`

Cuts power to both motors immediately.

---

### `DriveSubsystem`

> `src/main/java/frc/robot/SwerveSubsystems/DriveSubsystem.java`

Coordinates all four modules. Primary drive API for commands and autonomous.

---

#### `directionalDrive(double speed, double angle)`

Sends all four modules to the same heading at the same speed. Translation only, no rotation.

- `speed` — normalized drive output
- `angle` — target heading in radians (robot-relative)

---

#### `directionalDrive(double speed, double angle, double rotation)`

Full holonomic drive. Combines a translation vector with a rotation rate via polar vector addition.

- `speed` — translation magnitude, normalized
- `angle` — translation direction in radians
- `rotation` — rotation rate, normalized

Each module's command is the vector sum of the translation vector and a tangential rotation vector for that module's position around the robot center.

---

#### `drive(double speed, double rotate)`

Raw open-loop control broadcast to all four modules simultaneously. No steering feedback.

---

#### `driveRobotRelative(ChassisSpeeds speeds)`

Converts `ChassisSpeeds` to `SwerveModuleState[]` via kinematics, desaturates to `MaxDriveSpeed`, and calls `setDesiredState` on each module. Used by PathPlanner.

---

#### `getRobotRelativeSpeeds() → ChassisSpeeds`

Converts current module states back to robot-relative `ChassisSpeeds` via kinematics. Provides velocity feedback to PathPlanner.

---

#### `rotate(double speed)`

Points all wheels tangentially (±45° and ±135° from center) and drives them at `speed`, spinning the robot in place.

---

#### `carDrive(double rotationFactor, double speed)`

Ackermann (car-style) steering. Computes a turn radius from `rotationFactor` and assigns each wheel its own angle and speed accordingly.

- `rotationFactor` — curvature; near 0 is straight
- `speed` — forward speed, normalized

---

#### `stop()`

Cuts power to all four modules immediately.

---

#### `averageDistanceEncoder() → double`

Returns the mean of all four modules' accumulated distances in meters. Used as a scalar distance estimate for straight-line travel.

---

#### `getPositions() → SwerveModulePosition[]`

Returns `[FL, FR, BL, BR]` module positions (distance + heading) for odometry input.

---

#### `getStates() → SwerveModuleState[]`

Returns `[FL, FR, BL, BR]` module velocity states for PathPlanner feedback.

---

#### `getPID() → PIDController`

Returns the subsystem's `PIDController` instance (P = 0.63). Used by drive commands to compute drive speed from distance error.

---

#### `initAutoBuilder(OdometrySubsystem odomSub)`

Registers with PathPlanner's `AutoBuilder`. Configuration:
- Robot mass: 50 kg, MOI: 6.0 kg·m²
- Wheel radius: 0.0508 m, max speed: 0.5 m/s, COF: 1.2
- Drive gearing: 6.75:1, current limit: 40 A
- Translation PID: P = 0.5 / Rotation PID: P = 0.5

The drive consumer converts PathPlanner's field-relative `ChassisSpeeds` to robot-relative before calling `driveRobotRelative`.

---

### `OdometrySubsystem`

> `src/main/java/frc/robot/Information/OdometrySubsystem.java`

Maintains the robot pose estimate by fusing wheel odometry with gyroscope data. Accepts optional vision corrections.

---

#### Periodic behavior

Every robot loop (~20 ms):
1. Reads NavX gyro angle in radians (negated for WPILib convention).
2. Calls `m_poseEstimator.updateWithTime(timestamp, gyroAngle, modulePositions)`.
3. Publishes raw `pose.getX()` / `pose.getY()` to SmartDashboard and updates `Field2d`.

---

#### `getPose() → Pose2d`

Returns the latest estimated pose. Used directly by PathPlanner.

---

#### `resetPose(Pose2d newPose)`

Resets the pose estimator to `newPose` using the current gyro angle and module positions. Called by PathPlanner at path start.

---

#### `getX() / getY() → double`

Returns the estimated position in meters. `getY()` negates `pose.getY()` to match the field coordinate convention used by commands.

---

#### `getGyroAngle() → double`

Returns the gyro heading in radians, wrapped to `[−π, π)`.

---

#### `setPose(double x, double y, double rotation)`

Resets the pose estimator to an explicit position and heading. Called by the vision system when a reliable AprilTag fix is available.

---

#### `resetGyro()`

Zeros the NavX IMU and re-seeds the pose estimator at the current pose. Called at the start of each manual drive command.

---

#### `addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3,N1> stdDevs)`

Injects an AprilTag pose measurement into the estimator with standard deviations. The Kalman filter blends this with accumulated wheel odometry to reduce drift.

---

#### `getSwerveAngles() → double[]`

Returns the current `steerAngleWPILib()` for each of the four modules as a raw array. Order: `[FL, FR, BL, BR]`.

---

## Control Flow Summary

```
Teleop input (joystick/xbox)
  └─► DriveSubsystem.directionalDrive(speed, angle, rotation)
        └─► per module: SwerveDriveModule.directionalDrive(speed, angle)
              ├─ steerAngle() → compute error → optimization → P-controller → steer motor
              └─ speed × direction → drive motor

Autonomous (PathPlanner)
  └─► DriveSubsystem.driveRobotRelative(ChassisSpeeds)
        ├─ kinematics.toSwerveModuleStates()
        ├─ desaturateWheelSpeeds()
        └─► per module: SwerveDriveModule.setDesiredState(state)
              ├─ optimize() using steerAngleWPILib()
              ├─ P-controller on angular error → steer motor
              └─ speed / MaxDriveSpeed → drive motor

Odometry (every loop)
  └─► OdometrySubsystem.periodic()
        ├─ NavX gyro angle (negated)
        ├─ getPositions() → [steerAngleWPILib() + distance] per module
        └─► SwerveDrivePoseEstimator.updateWithTime()
              └─ optional: addVisionMeasurement() from AprilTags
```
