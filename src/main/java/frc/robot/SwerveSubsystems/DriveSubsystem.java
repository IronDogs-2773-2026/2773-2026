// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.SwerveSubsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Information.OdometrySubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;

public class DriveSubsystem extends SubsystemBase {
  public SwerveDriveModule blMotor = new SwerveDriveModule(Constants.backLeftModuleDriveCANID,
      Constants.backLeftModuleRotateCANID, Constants.backLeftModuleEncoderCANID, 0.3686);
  public SwerveDriveModule brMotor = new SwerveDriveModule(Constants.backRightModuleDriveCANID,
      Constants.backRightModuleRotateCANID, Constants.backRightModuleEncoderCANID, -0.1597);
  public SwerveDriveModule frMotor = new SwerveDriveModule(Constants.frontRightModuleDriveCANID,
      Constants.frontRightModuleRotateCANID, Constants.frontRightModuleEncoderCANID, -0.48657);
  public SwerveDriveModule flMotor = new SwerveDriveModule(Constants.frontLeftModuleDriveCANID,
      Constants.frontLeftModuleRotateCANID, Constants.frontLeftModuleEncoderCANID, 0.35522);
  public double p = 0.63;
  public double i = 0;
  public double d = 0;
  public PIDController pid = new PIDController(p, i, d);

  // Swerve kinematics for converting ChassisSpeeds to module states
  public SwerveDriveKinematics kinematics = Constants.kinematics;

  public SwerveModulePosition[] getPositions() {
    return new SwerveModulePosition[] {
        flMotor.getSwervePosition(), frMotor.getSwervePosition(),
        blMotor.getSwervePosition(), brMotor.getSwervePosition()
    };
  }

  public DriveSubsystem() {
  }

  public SwerveDriveModule[] modules = { flMotor, frMotor, blMotor, brMotor };

  @Override
  public void periodic() {

  }

  public void drive(double speed, double rotate) {
    blMotor.drive(speed, rotate);
    brMotor.drive(speed, rotate);
    frMotor.drive(speed, rotate);
    flMotor.drive(speed, rotate);
  }

  public void directionalDrive(double speed, double angle) {
    blMotor.directionalDrive(speed, angle);
    brMotor.directionalDrive(speed, angle);
    frMotor.directionalDrive(speed, angle);
    flMotor.directionalDrive(speed, angle);
  }

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

  public void stop() {
    blMotor.stop();
    brMotor.stop();
    frMotor.stop();
    flMotor.stop();
  }

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

  public PIDController getPID() {
    return pid;
  }

  public double averageDistanceEncoder() {
    return (flMotor.distanceEncoderPosition() + frMotor.distanceEncoderPosition() + blMotor.distanceEncoderPosition()
        + brMotor.distanceEncoderPosition()) / 4;
  }

  public SwerveModuleState[] getStates() {
    return new SwerveModuleState[] {
        flMotor.getSwerveState(),
        frMotor.getSwerveState(),
        blMotor.getSwerveState(),
        brMotor.getSwerveState()
    };
  }

  /**
   * Get current robot-relative chassis speeds from module states
   */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(getStates());
  }

  /**
   * Drive using robot-relative ChassisSpeeds (required for PathPlanner)
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
   * Initialize AutoBuilder for PathPlanner
   * Call this after OdometrySubsystem is created
   */
  public void initAutoBuilder(OdometrySubsystem odomSub) {
    System.out.println("DriveSubsystem: Starting AutoBuilder initialization...");
    
    try {
      System.out.println("DriveSubsystem: Creating RobotConfig...");
      
      // Create RobotConfig inline (more reliable than fromGUISettings)
      // PathPlannerLib 2026 API - module locations in FL, FR, BL, BR order
      RobotConfig config = new RobotConfig(
        50.0,           // Mass: 50 kg
        6.0,            // MOI: 6.0 kg·m²
        new com.pathplanner.lib.config.ModuleConfig(
          0.0508,       // Wheel radius: 0.0508m (from 0.1016m diameter)
          0.5,          // Max speed: 0.5 m/s
          1.2,          // Wheel COF: 1.2
          DCMotor.getNEO(1),  // Motor: 1 NEO per module
          6.75,         // Drive gearing: 6.75
          40            // Current limit: 40A
        ),
        Constants.kfrontLeftLocation,    // Front Left
        Constants.kfrontRightLocation,   // Front Right
        Constants.kbackLeftLocation,     // Back Left
        Constants.kbackRightLocation     // Back Right
      );

      System.out.println("DriveSubsystem: RobotConfig created, configuring AutoBuilder...");

      // Configure AutoBuilder
      AutoBuilder.configure(
        odomSub::getPose,                    // Pose supplier
        odomSub::resetPose,                  // Pose consumer (reset odometry)
        this::getRobotRelativeSpeeds,        // Robot-relative ChassisSpeeds supplier
        (speeds, feedforwards) -> driveRobotRelative(speeds),
        new PPHolonomicDriveController(
          new PIDConstants(0.5, 0.0, 0.0),   // Translation PID - lowered from 1.0
          new PIDConstants(0.5, 0.0, 0.0)    // Rotation PID - lowered from 1.0
        ),
        config,
        () -> {
          // Flip paths for red alliance
          var alliance = DriverStation.getAlliance();
          return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
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
