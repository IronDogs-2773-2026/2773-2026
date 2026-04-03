// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.SwerveSubsystems;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Information.OdometrySubsystem;

/**
 * Swerve drive subsystem controlling four MAXSwerve modules.
 * 
 * <p>Each module consists of a NEO drive motor, a NEO rotation (steering) motor,
 * and a CTRE CANCoder for absolute steering position. Module angle offsets are
 * calibrated at construction time.
 * 
 * <p>Drive modes supported:
 * <ul>
 *   <li><b>directionalDrive</b> — Polar coordinate drive (speed + angle + rotation)</li>
 *   <li><b>driveRobotRelative</b> — ChassisSpeeds-based drive (for PathPlanner)</li>
 *   <li><b>carDrive</b> — Ackermann-style curved driving (deprecated)</li>
 * </ul>
 */
public class DriveSubsystem extends SubsystemBase {
  /** Back-left swerve module. */
  public SwerveDriveModule blMotor = new SwerveDriveModule(Constants.backLeftModuleDriveCANID,
      Constants.backLeftModuleRotateCANID, Constants.backLeftModuleEncoderCANID, 0.3686);

  /** Back-right swerve module. */
  public SwerveDriveModule brMotor = new SwerveDriveModule(Constants.backRightModuleDriveCANID,
      Constants.backRightModuleRotateCANID, Constants.backRightModuleEncoderCANID, -0.1597);

  /** Front-right swerve module. */
  public SwerveDriveModule frMotor = new SwerveDriveModule(Constants.frontRightModuleDriveCANID,
      Constants.frontRightModuleRotateCANID, Constants.frontRightModuleEncoderCANID, -0.48657);

  /** Front-left swerve module. */
  public SwerveDriveModule flMotor = new SwerveDriveModule(Constants.frontLeftModuleDriveCANID,
      Constants.frontLeftModuleRotateCANID, Constants.frontLeftModuleEncoderCANID, 0.35522);

  /** PID controller for drive speed (currently unused). */
  public double p = 0.63;
  public double i = 0;
  public double d = 0;
  public PIDController pid = new PIDController(p, i, d);

  /** Cached alliance state (updated every periodic cycle from FMS). */
  private Optional<DriverStation.Alliance> alliance;

  /** Swerve kinematics for converting ChassisSpeeds to module states. */
  public SwerveDriveKinematics kinematics = Constants.kinematics;

  /**
   * Returns the current position of all four swerve modules.
   * Order: [front-left, front-right, back-left, back-right]
   * 
   * @return Array of SwerveModulePosition for odometry updates
   */
  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        flMotor.getSwervePosition(), frMotor.getSwervePosition(),
        blMotor.getSwervePosition(), brMotor.getSwervePosition()
    };
  }

  public DriveSubsystem() {
  }

  public SwerveDriveModule[] modules = { flMotor, frMotor, blMotor, brMotor };

  /**
   * Sets the cached alliance state for manual override.
   * 
   * @param alliance The alliance to cache (Blue or Red)
   */
  public void setAlliance(DriverStation.Alliance alliance) {
    this.alliance = java.util.Optional.of(alliance);
  }

  @Override
  public void periodic() {
    alliance = DriverStation.getAlliance();
  }

  /**
   * Drives all modules in a straight line (tank-style, no rotation).
   * 
   * @param speed Drive speed (-1.0 to 1.0)
   * @param rotate Rotation factor (unused in this mode)
   * @deprecated Use {@link #directionalDrive(double, double, double)} instead
   */
  public void drive(double speed, double rotate) {
    blMotor.drive(speed, rotate);
    brMotor.drive(speed, rotate);
    frMotor.drive(speed, rotate);
    flMotor.drive(speed, rotate);
  }

  /**
   * Drives the robot in a specified direction with optional rotation.
   * Each module is oriented to the given angle and driven at the given speed,
   * with rotation vectors added to each module based on its position.
   * 
   * @param speed Drive speed (0.0 to 1.0)
   * @param angle Drive direction in radians (field-relative)
   */
  public void directionalDrive(double speed, double angle) {
    blMotor.directionalDrive(speed, angle);
    brMotor.directionalDrive(speed, angle);
    frMotor.directionalDrive(speed, angle);
    flMotor.directionalDrive(speed, angle);
  }

  /**
   * @deprecated Use {@link #driveRobotRelative(ChassisSpeeds)} instead.
   * This method incorrectly converts ChassisSpeeds to polar coordinates and ignores
   * the rotation component (omega). It does not use swerve kinematics and will not
   * produce correct module states for holonomic drive.
   */
  @Deprecated(since = "2026", forRemoval = true)
  public void chassisDrive(ChassisSpeeds chassisSpeeds) {
    directionalDrive(
        Constants.powerVelocityRatio
            * Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2) + Math.pow(chassisSpeeds.vyMetersPerSecond, 2)),
        Constants.powerTwistRatio * Math.atan2(chassisSpeeds.vyMetersPerSecond, chassisSpeeds.vxMetersPerSecond));
  }

  static class Vec {
    double phi;
    double r;

    Vec(double r, double phi) {
      this.phi = phi;
      this.r = r;
    }

    Vec add(Vec a) {
      double x = this.r * Math.cos(this.phi);
      double y = this.r * Math.sin(this.phi);
      x += a.r * Math.cos(a.phi);
      y += a.r * Math.sin(a.phi);
      return new Vec(
          Math.sqrt(x * x + y * y),
          Math.atan2(y, x));
    }
  }

  /**
   * Drives the robot with full field-relative control: translation speed, direction, and rotation.
   * Uses vector addition to combine translation and rotation components for each module.
   * 
   * @param speed Translation speed (0.0 to 1.0)
   * @param angle Translation direction in radians (field-relative)
   * @param rotation Rotation speed (-1.0 to 1.0), positive = counter-clockwise
   */
  public void directionalDrive(double speed, double angle, double rotation) {
    Vec bl = new Vec(speed, angle).add(new Vec(rotation, -3 * Math.PI / 4));
    Vec br = new Vec(speed, angle).add(new Vec(rotation, 3 * Math.PI / 4));
    Vec fr = new Vec(speed, angle).add(new Vec(rotation, Math.PI / 4));
    Vec fl = new Vec(speed, angle).add(new Vec(rotation, -Math.PI / 4));
    blMotor.directionalDrive(bl.r, bl.phi);
    brMotor.directionalDrive(br.r, br.phi);
    frMotor.directionalDrive(fr.r, fr.phi);
    flMotor.directionalDrive(fl.r, fl.phi);
  }

  /**
   * Stops all four swerve modules.
   */
  public void stop() {
    blMotor.stop();
    brMotor.stop();
    frMotor.stop();
    flMotor.stop();
  }

  /**
   * Rotates the robot in place (all modules oriented for pure rotation).
   * 
   * @param speed Rotation speed (-1.0 to 1.0), positive = counter-clockwise
   */
  public void rotate(double speed) {
    frMotor.directionalDrive(speed, Math.PI / 4);
    brMotor.directionalDrive(speed, 3 * Math.PI / 4);
    blMotor.directionalDrive(speed, -3 * Math.PI / 4);
    flMotor.directionalDrive(speed, -Math.PI / 4);
  }

  public void carDrive(double rotationFactor, double speed) {
    final double HALF_WHEEL_DISTANCE = Constants.DistanceBetweenWheels;
    double distance = 1 / (rotationFactor + 1e-7);

    speed *= Math.copySign(1, distance);

    double rl = Math.sqrt(
        HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE +
            (distance - HALF_WHEEL_DISTANCE) * (distance - HALF_WHEEL_DISTANCE));
    double rr = Math.sqrt(
        HALF_WHEEL_DISTANCE * HALF_WHEEL_DISTANCE +
            (distance + HALF_WHEEL_DISTANCE) * (distance + HALF_WHEEL_DISTANCE));

    double flAngle = Math.atan2(distance + HALF_WHEEL_DISTANCE, -HALF_WHEEL_DISTANCE);
    double frAngle = Math.atan2(distance - HALF_WHEEL_DISTANCE, -HALF_WHEEL_DISTANCE);
    double blAngle = Math.atan2(distance + HALF_WHEEL_DISTANCE, HALF_WHEEL_DISTANCE);
    double brAngle = Math.atan2(distance - HALF_WHEEL_DISTANCE, HALF_WHEEL_DISTANCE);

    double kl = 1, kr = 1;
    if (distance < 0) {
      kl = rr / rl;
    } else {
      kr = rl / rr;
    }

    frMotor.directionalDrive(kr * speed, frAngle);
    brMotor.directionalDrive(kr * speed, brAngle);
    blMotor.directionalDrive(kl * speed, blAngle);
    flMotor.directionalDrive(kl * speed, flAngle);
  }

  /**
   * Returns the drive PID controller.
   * 
   * @return The PIDController used for drive speed control
   */
  public PIDController getPID() {
    return pid;
  }

  /**
   * Returns the average distance traveled by all four drive modules.
   * Used for PID-based drive speed control.
   * 
   * @return Average encoder distance across all modules
   */
  public double averageDistanceEncoder() {
    return (flMotor.distanceEncoderPosition() + frMotor.distanceEncoderPosition() + blMotor.distanceEncoderPosition()
        + brMotor.distanceEncoderPosition()) / 4;
  }

  /**
   * Returns the current state of all four swerve modules.
   * Order: [front-left, front-right, back-left, back-right]
   * 
   * @return Array of SwerveModuleState for kinematics calculations
   */
  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
        flMotor.getSwerveState(),
        frMotor.getSwerveState(),
        blMotor.getSwerveState(),
        brMotor.getSwerveState()
    };
  }

  /**
   * Converts current module states to robot-relative chassis speeds.
   * 
   * @return ChassisSpeeds with vx, vy (m/s) and omega (rad/s) relative to robot frame
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  /**
   * Drives the robot using robot-relative ChassisSpeeds.
   * Required for PathPlanner AutoBuilder integration.
   * 
   * @param robotRelativeSpeeds Target chassis speeds (vx, vy in m/s, omega in rad/s)
   */
  public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    // Convert ChassisSpeeds to SwerveModuleStates
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(robotRelativeSpeeds);

    // Desaturate wheel speeds to stay within limits
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MaxDriveSpeed);

    // Set each module to its target state
    flMotor.setDesiredState(states[0]);
    frMotor.setDesiredState(states[1]);
    blMotor.setDesiredState(states[2]);
    brMotor.setDesiredState(states[3]);
  }

  /**
   * Initializes PathPlanner AutoBuilder with this subsystem's configuration.
   * Must be called after OdometrySubsystem is created.
   * 
   * <p>Configures:
   * <ul>
   *   <li>Pose supplier and reset consumer for odometry</li>
   *   <li>Robot-relative speed supplier and drive consumer</li>
   *   <li>Holonomic drive controller with translation/rotation PID constants</li>
   *   <li>RobotConfig from PathPlanner GUI settings</li>
   *   <li>Alliance supplier for automatic path mirroring</li>
   * </ul>
   * 
   * @param odomSub The odometry subsystem for pose tracking
   */
  public void initAutoBuilder(OdometrySubsystem odomSub) {
    System.out.println("DriveSubsystem: Starting AutoBuilder initialization...");

    try {
      System.out.println("DriveSubsystem: Loading RobotConfig from GUI settings...");

      // Load RobotConfig from pathplanner/settings.json
      RobotConfig config = RobotConfig.fromGUISettings();

      System.out.println("DriveSubsystem: RobotConfig loaded, configuring AutoBuilder...");

      // Configure AutoBuilder
      AutoBuilder.configure(
        odomSub::getPose,                    // Pose supplier
        odomSub::resetPose,                  // Pose consumer (reset odometry)
        this::getRobotRelativeSpeeds,        // Robot-relative ChassisSpeeds supplier
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
          new PIDConstants(0.1, 0.0, 0.0),   // Translation PID - lowered from 1.00
          new PIDConstants(0.1, 0.0, 0.0)    // Rotation PID - lowered from 1.0
        ),
        config,
        () -> {
          // Get current FMS alliance
          var currentAlliance = DriverStation.getAlliance();
          
          // Check if cached alliance differs from current (indicates manual override)
          if (alliance.isPresent() && currentAlliance.isPresent()) {
            if (alliance.get() != currentAlliance.get()) {
              // Cached differs from FMS - manual override is active
              return alliance.get() == DriverStation.Alliance.Red;
            }
          }
          
          // No override - use FMS
          return currentAlliance.isPresent() && currentAlliance.get() == DriverStation.Alliance.Red;
        },
        this                                   // Reference to this subsystem
      );
      
      System.out.println("DriveSubsystem: AutoBuilder configured successfully!");
    } catch (Exception e) {
      String errorMsg = "AutoBuilder init failed: " + e.getMessage();
      DriverStation.reportError(errorMsg, true);
      System.err.println(errorMsg);
      e.printStackTrace();
    }
  }
}
