Qwen default instructions

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

**`steerAngle()` — used everywhere:**

```
raw      = encoder.getAbsolutePosition()   // [0, 1) rotations, CCW+
adjusted = raw − alpha                     // remove mounting offset
adjusted = wrap(adjusted, −0.5, +0.5)      // normalize to half-rotation
heading  = adjusted × 2π                  // to radians, CCW-positive (WPILib convention)
```

All callers — `directionalDrive`, `setDesiredState`, `getSwervePosition`, `getSwerveState`, and `getSwerveAngles` — use this single method.

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
1. Calls `SwerveModuleState.optimize()` using `steerAngle()`.
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

Returns current wheel heading in radians, offset-corrected, CCW-positive (WPILib convention). Used by `directionalDrive`, `setDesiredState`, `getSwervePosition`, and `getSwerveState`.

---

#### `getSwervePosition() → SwerveModulePosition`

Returns module state for the pose estimator:
- `distanceMeters` — accumulated drive distance
- `angle` — current heading via `steerAngle()`

---

#### `getSwerveState() → SwerveModuleState`

Returns current module velocity for PathPlanner feedback:
- `speedMetersPerSecond` — current wheel speed from drive motor velocity
- `angle` — current heading via `steerAngle()`

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

Returns the current `steerAngle()` for each of the four modules as a raw array. Order: `[FL, FR, BL, BR]`.

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
              ├─ optimize() using steerAngle()
              ├─ P-controller on angular error → steer motor
              └─ speed / MaxDriveSpeed → drive motor

Odometry (every loop)
  └─► OdometrySubsystem.periodic()
        ├─ NavX gyro angle (negated)
        ├─ getPositions() → [steerAngle() + distance] per module
        └─► SwerveDrivePoseEstimator.updateWithTime()
              └─ optional: addVisionMeasurement() from AprilTags
```

---

# Shooter Mechanism

## Components

| Component | CAN ID | Type | Function |
|-----------|--------|------|----------|
| **Flywheel 1** | 13 | REV NEO (Brushless) | Primary shooting wheel |
| **Flywheel 2** | 24 | REV NEO (Brushless) | Counter-rotating wheel (follows #13 inverted) |
| **Feeder Motor** | 25 | REV NEO (Brushless) | Pushes game piece into flywheels |
| **Arm Motor** | 49 | REV NEO (Brushless) | Positions shooter mechanism |
| **Intake Motor** | 29 | REV NEO (Brushless) | Collects game pieces |

## Motor Configuration

- **Flywheel 1**: Direct control (master)
- **Flywheel 2**: Follows Flywheel 1 with **inverted** direction (counter-rotation)
- **Current Limit**: 40A for all motors

## Control Methods

| Method | Purpose |
|--------|---------|
| `directRun(speed)` | Set flywheel speed directly (-1.0 to 1.0) |
| `pidRun(targetRPM)` | Use PID to reach target RPM |
| `runFeeder(speed)` | Run feeder motor |
| `setArmSpeed(speed)` | Control arm position |
| `runIntake(speed)` | Run intake motor |
| `stop()` | Stop all shooter motors |

## PID Configuration

```java
Flywheel PID: P=0.1, I=0.0, D=0.0
Tolerance: ±50 RPM for at-setpoint detection
```

## Shoot Sequence

The `ShootSequenceCommand` executes:
1. **Spin Up**: Set flywheels to target speed
2. **Wait**: Delay for flywheels to reach speed (configurable, e.g., 0.5-1.5s)
3. **Shoot**: Run feeder to launch game piece (configurable duration)
4. **Stop**: Halt all motors

---

# Control System

## Driver Station Controllers

| Controller | Port | Type | Purpose |
|------------|------|------|---------|
| **Driver** | 0 | Xbox Controller | Swerve drive control |
| **Operator** | 2 | Xbox Controller | Shooter mechanism control |

## Driver Controls (Xbox Port 0)

| Input | Action |
|-------|--------|
| **Left Stick** | Field-relative translation |
| **Right Stick** | Rotation (unless in setpoint mode) |
| **D-Pad** | POV rotation to cardinal directions (0°, 90°, 180°, 270°) |
| **D-Pad Up/Down** | Increase/decrease sensitivity |
| **Buttons 7+8** | Reset drive heading offset (does not reset odometry) |
| **Button 1 (A)** | Toggle rotation setpoint mode |
| **Button A** | Pathfind to and follow "Red1" path |
| **Button Y** | Follow "New New Path" |
| **Button X** | Run "New Auto" autonomous routine |
| **Button B** | Drive 1 meter forward (field-relative) |
| **Right Bumper** | Execute shoot sequence (0.6 flywheel, 0.5 feeder, 1s shoot, 0.5s spinup) |

## Operator Controls (Xbox Port 2)

| Input | Action |
|-------|--------|
| **Left Stick Y** | Flywheel speed (manual) |
| **Left Stick X** | Feeder speed (manual) |
| **Triggers** | Arm control (up/down) |
| **Right Stick X** | Intake control |
| **Right Bumper** | Flywheel spin (0.6 speed) |

## Drive Features

- **Field-Relative Drive**: Automatically compensates for robot heading
- **Sensitivity Control**: Adjustable via D-Pad (0.05 to 1.0)
- **Deadzone**: 0.07 for controller inputs
- **Max Speeds**:
  - Drive: 0.6 (60%)
  - Rotation: 0.50 (50%)
  - Autonomous: 0.3 (30%)

---

# Odometry & Vision System

## OdometrySubsystem

Uses `SwerveDrivePoseEstimator` to fuse:
- Wheel encoder positions (from all 4 modules)
- Gyro heading (NavX AHRS)
- Vision measurements (AprilTag poses)

**Key Features:**
- Pose updated every 20ms in `periodic()`
- Gyro angle negated for correct coordinate system: `gyro.getRotation2d().times(-1)`
- Y-coordinate negated for field orientation: `getY()` returns `-pose.getY()`
- Field visualization via `Field2d` on SmartDashboard

## VisionSubsystem (PhotonVision)

**Camera Configuration:**
- Camera Name: "AprilCam"
- Robot-to-Camera Transform: `(0.01, 0.2, 1.0)` meters, rotation `(0, π/6, π)` radians

**Pose Estimation Strategy:**
1. Try multi-tag pose estimation first (most accurate)
2. Fall back to lowest-ambiguity single-tag pose
3. Calculate dynamic standard deviations based on:
   - Number of visible tags (1 vs multiple)
   - Average distance to tags
   - Tag geometry quality

**Standard Deviation Heuristics:**
```java
Single Tag:     (0.1, 0.1, 10°)
Multiple Tags:  (0.05, 0.05, 5°)
Minimum:        (0.25, 0.25, 10°)
Far single tag (>4m): Effectively ignored (very high std devs)
```

## PhotonSubsystem (Alternative/Backup)

Similar functionality to VisionSubsystem with additional debugging:
- Logs target yaw, pitch, and area
- Tracks last good pose and timestamp
- Provides `getPose2d()` and `getStdDevs()` methods

---

# Autonomous System

## PathPlanner Integration

**Library**: PathPlannerLib 2026.1.2

**AutoBuilder Configuration:**
```java
Robot Mass: 50.0 kg
Robot MOI: 6.0 kg·m²
Wheel Radius: 0.0508 m
Max Speed: 1.0 m/s (conservative for auto)
Wheel COF: 1.2
Motor: NEO (1 per module)
Drive Gearing: 6.75:1
Current Limit: 40A

Translation PID: P=0.1, I=0.0, D=0.0
Rotation PID: P=0.1, I=0.0, D=0.0
```

## Path Files

Located in `src/main/deploy/pathplanner/paths/`:

| Path | Purpose |
|------|---------|
| `Example Path.path` | Test path with multiple waypoints |
| `New Path.path` | Custom path |
| `New New Path.path` | Custom path |
| `Red1.path` | Red alliance starting position path |
| `Go to Shoot.path` | Position for shooting |
| `Collect my Balls.path` | Collection path |
| `Robot to Ball.path` | Approach game piece path |

## Auto Files

Located in `src/main/deploy/pathplanner/autos/`:

| Auto | Purpose |
|------|---------|
| `New Auto.auto` | Custom autonomous routine |
| `Gunner's Auto.auto` | Custom autonomous routine |

## Named Commands

Commands registered for use in PathPlanner GUI:
- `Shoot Sequence 5`: 5-second shoot sequence
- `Shoot Sequence 10`: 10-second shoot sequence
- `Shoot Sequence 15`: 15-second shoot sequence

## Autonomous Selection

- **UI**: SendableChooser on SmartDashboard ("AutoChooser")
- **Options**: None, Example Path, New Path, Red1, New New Path, New Auto
- **Execution**: Command scheduled in `autonomousInit()`, cancelled in `teleopInit()`

---

# Command-Based Framework

## Architecture

- **Framework**: WPILib Command-Based (2026)
- **Robot Class**: Extends `TimedRobot`
- **Scheduler**: `CommandScheduler.getInstance().run()` in `robotPeriodic()`

## Subsystem Lifecycle

```java
constructor() → configure hardware, set defaults
periodic() → runs every 20ms
```

## Command Lifecycle

```java
initialize() → called once when scheduled
execute() → called every 20ms while scheduled
isFinished() → returns true when command should end
end(interrupted) → cleanup when finished
```

## Command Composition

- **Sequences**: `Commands.sequence()` for ordered execution
- **Parallel**: `Commands.parallel()` for simultaneous execution
- **Timeouts**: `.withTimeout(seconds)` for time limits
- **Triggers**: `JoystickButton`, `Trigger` for button bindings

## Default Commands

| Subsystem | Default Command | Purpose |
|-----------|-----------------|---------|
| DriveSubsystem | XBOXDriveCommand | Field-relative teleop drive |
| ShooterSubsystem | ShooterDefaultCommand | Manual shooter/feeder/arm/intake control |

---

# Key Constants & Specifications

## Physical Specifications

| Parameter | Value | Unit |
|-----------|-------|------|
| Robot Mass | 50.0 | kg |
| Moment of Inertia | 6.0 | kg·m² |
| Robot Width | 0.9 | m |
| Robot Length | 0.9 | m |
| Wheel Radius | 0.0508 | m |
| Wheelbase (X) | 0.566 | m |
| Track Width (Y) | 0.562 | m |

## Motor Specifications

| Motor Type | Stall Current | Free Speed | Notes |
|------------|---------------|------------|-------|
| REV NEO | 106A | 5676 RPM | Brushless |

## Control Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `MaxDriveSpeed` | 0.6 | Maximum drive motor output |
| `MaxRotationSpeed` | 0.50 | Maximum rotation motor output |
| `MaxAutoDriveSpeed` | 0.3 | Maximum autonomous speed |
| `ControllerDeadzone` | 0.07 | Joystick deadzone |

## Vision Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| `SingleTagStdDevs` | (0.1, 0.1, 10°) | Single AprilTag measurement confidence |
| `MultiTagStdDevs` | (0.05, 0.05, 5°) | Multiple AprilTag measurement confidence |
| `MinVisionStdDevs` | (0.25, 0.25, 10°) | Minimum confidence threshold |
| `TimeAllowance` | 3.0 | Seconds |
| `TrustCoefficient` | 0.3 | Vision weighting factor |

## PID Constants

| Controller | P | I | D | Purpose |
|------------|---|---|---|---------|
| Module Rotation | 0.30 | 0 | 0 | Swerve module steering |
| Drive Translation | 0.63 | 0 | 0 | Field-relative drive |
| PathPlanner Translation | 0.1 | 0 | 0 | Autonomous path following |
| PathPlanner Rotation | 0.1 | 0 | 0 | Autonomous rotation |
| Flywheel | 0.1 | 0 | 0 | Flywheel velocity control |
| POV Rotation | 0.2 | 0 | 0 | D-Pad heading control |

---

# Deployment & Simulation

## Build Configuration

- **Java Version**: 17
- **WPILib Version**: 2026.2.1
- **Desktop Simulation**: Disabled (`includeDesktopSupport = false`)

## Deploy Path

- **Java Code**: `/home/lvuser/deploy/`
- **PathPlanner Paths**: `/home/lvuser/pathplanner/paths/` (configured in settings.json)

## SmartDashboard Items

| Item | Type | Purpose |
|------|------|---------|
| `AutoChooser` | SendableChooser | Select autonomous routine |
| `Field` | Field2d | Visualize robot pose |
| `X`, `Y` | Number | Robot position coordinates |

## Shuffleboard Tabs

| Tab | Items |
|-----|-------|
| Odometry | Robot X, Robot Y |

---

# Troubleshooting

## Common Issues

**Robot spins during autonomous:**
- Check gyro sign convention (`gyro.getRotation2d().times(-1)`)
- Verify module encoder offsets are correct
- Test with straight-line path first

**Modules fight each other:**
- Run PathPlanner module verification routine
- Recalibrate CANcoder offsets
- Check motor inversion settings

**Vision causes position jumps:**
- Increase standard deviations for vision measurements
- Check camera mounting position and angle
- Verify AprilTag field layout matches competition field

**Path tracking is poor:**
- Tune translation PID (start P=1.0, increase until no oscillation)
- Tune rotation PID (start P=0.5, add D if overshooting)
- Verify RobotConfig parameters (mass, MOI, wheel radius)

**Autonomous doesn't start:**
- Check AutoChooser selection on SmartDashboard
- Verify path files exist in deploy directory
- Ensure AutoBuilder is initialized before path commands

---

# Resources

## Documentation

- [WPILib Documentation](https://docs.wpilib.org/)
- [PathPlanner Documentation](https://pathplanner.dev/)
- [PhotonVision Documentation](https://docs.photonvision.org/)
- [REV Robotics NEO Documentation](https://docs.revrobotics.com/brushless/neo/)
- [CTRE Phoenix 6 Documentation](https://v6.docs.ctr-electronics.com/)

## Team Resources

- Team 2773 IronDogz GitHub
- PathPlanner Discord (#pathplanner channel)
- FRC Discord (vendor support channels)

---

*Last Updated: March 30, 2026*
*FRC Team 2773 - IronDogz*



