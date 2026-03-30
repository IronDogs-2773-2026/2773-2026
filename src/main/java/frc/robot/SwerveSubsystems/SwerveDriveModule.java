// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveSubsystems;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;

/** Add your docs here. */
public class SwerveDriveModule {
  
  // Variables O:
  private static final double ROTATION_LIMIT_SPEED = 0.7;

  public SparkMax driveMotor;
  public SparkMax rotateMotor;
  public CANcoder encoder;
  public RelativeEncoder distanceEncoder;
  public double alpha;
  private PIDController pidRotate;

  private static double DriveMotorWheelGearRatio = 1.0 / 6.75;
  private static double EncoderMagicRevolutionNumber = 0.047964; // 42/1024 = resolution/1024

  public SwerveDriveModule(int driveId, int rotateId, int encoderId, double alpha) {
    driveMotor = new SparkMax(driveId, frc.robot.Constants.motorType);
    rotateMotor = new SparkMax(rotateId, frc.robot.Constants.motorType);
    encoder = new CANcoder(encoderId);
    distanceEncoder = driveMotor.getEncoder();
    this.alpha = alpha;
    this.pidRotate = new PIDController(0.30, 0, 0);
    distanceEncoder.setPosition(0);
  }

  // Gets the position of the robot based on how far it has moved
  public double distanceEncoderPosition() {
    return distanceEncoder.getPosition() / EncoderMagicRevolutionNumber * DriveMotorWheelGearRatio
        * Constants.WheelCircumference;
  }

  // turns on the motors based on the speed and rotation
  public void drive(double speed, double rotate) {
    driveMotor.set(speed);
    rotateMotor.set(rotate);
  }

  
  public void directionalDrive(double speed, double angle) {
    pidRotate.setSetpoint(0);
    double pos = -steerAngle() - angle;

    // -PI =< pos < PI
    while (pos < -Math.PI)
      pos += 2 * Math.PI;
    while (pos >= Math.PI)
      pos -= 2 * Math.PI;

    double direction = 1.0;

    // negates direction if position is outside of +- pi / 2
    if (pos < -Math.PI / 2) {
      direction = -1.0;
      pos += Math.PI;
    }
    if (pos > Math.PI / 2) {
      direction = -1.0;
      pos -= Math.PI;
    }
    double speedOfRotation = pidRotate.calculate(pos);
    speedOfRotation = MathUtil.clamp(speedOfRotation, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
    rotateMotor.set(speedOfRotation);

    driveMotor.set(speed * direction);
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // WPILib angle convention: 0 = forward, CCW+
    // Our directionalDrive convention: equilibrium steerAngle() = -angle, so angle = -wpilib_angle
    // TODO: verify steerAngle() = 0 means physical forward on the robot. If not, a fixed
    // offset (e.g. -Math.PI/2) must be added: directionalDrive(speed, -angle - Math.PI/2)
    double speed = desiredState.speedMetersPerSecond / Constants.MaxDriveSpeed;
    directionalDrive(MathUtil.clamp(speed, -1.0, 1.0), -desiredState.angle.getRadians());
  }

  public double steerAngle() {
    // position [-0.5..0.5)
    double value = encoder.getAbsolutePosition().getValueAsDouble() - alpha;
    if (value < -0.5)
      value += 1.0;
    if (value >= 0.5)
      value -= 1.0;
    return value * 2 * Math.PI;
  }

  public void stop() {
    driveMotor.stopMotor();
    rotateMotor.stopMotor();
  }

  public SwerveModulePosition getSwervePosition() {
    return new SwerveModulePosition(
        distanceEncoderPosition(), new Rotation2d(steerAngle()));
  }

  public SwerveModuleState getSwerveState() {
    return new SwerveModuleState(
        distanceEncoder.getVelocity() / EncoderMagicRevolutionNumber * DriveMotorWheelGearRatio
            * Constants.WheelCircumference / 60.0, new Rotation2d(steerAngle()));
  }
}
