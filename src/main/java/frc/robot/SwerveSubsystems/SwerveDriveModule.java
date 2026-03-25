// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveSubsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  public int id;
  public double alpha;
  private PIDController pidRotate;

  private static double DriveMotorWheelGearRatio = 1.0 / 6.75;
  private static double EncoderMagicRevolutionNumber = 0.047964; // 42/1024 = resolution/1024

  // unused variables (maybe figure out what they were supposed to do)
  private double oldDistance = 0.0;
  private double velocity = 0.0;

  // Constructer :D
  public SwerveDriveModule(int driveId, int rotateId, int encoderId, double alpha) {
    driveMotor = new SparkMax(driveId, frc.robot.Constants.motorType);
    rotateMotor = new SparkMax(rotateId, frc.robot.Constants.motorType);
    encoder = new CANcoder(encoderId);
    distanceEncoder = driveMotor.getEncoder();
    id = encoderId;
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
    // this is likely dangerous, revert to canCoderPositionAdjusted() if there are issues
    double pos = canCoderPositionAdjusted() - angle;

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
    // 1. Optimize the state to prevent spinning more than 90 degrees.
    // This makes sure that if the wheel needs to go to 180°, it just 
    // stays at 0° and reverses the drive motor instead.
    SwerveModuleState optimizedState = SwerveModuleState.optimize(
        desiredState, 
        Rotation2d.fromRadians(canCoderPositionAdjusted())
    );

    // 2. Calculate the rotation output using your PID controller.
    // MathUtil.angleModulus ensures we take the shortest path (no 360-degree spins).
    double rotationError = MathUtil.angleModulus(
        optimizedState.angle.getRadians() - canCoderPositionAdjusted()
    );

    double rotationOutput = pidRotate.calculate(rotationError, 0);
    rotationOutput = MathUtil.clamp(rotationOutput, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
    rotateMotor.set(rotationOutput);

    // 3. Set the drive motor speed.
    // Convert m/s from the state to a percentage (-1.0 to 1.0) for the SparkMax.
    double driveOutput = optimizedState.speedMetersPerSecond / Constants.MaxDriveSpeed;
    driveMotor.set(driveOutput);
  }

  public double canCoderPositionAdjusted() {
    // position [-0.5..0.5)
    double value = encoder.getAbsolutePosition().getValueAsDouble() - alpha;
    if (value < -0.5)
      value += 1.0;
    if (value >= 0.5)
      value -= 1.0;
    return -value * 2 * Math.PI;

  }

  public double canCoderPositionAdjustedForOdometry() {
    // position [-0.5..0.5)
    double value = encoder.getAbsolutePosition().getValueAsDouble() - alpha;
    if (value < -0.5)
      value += 1.0;
    if (value >= 0.5)
      value -= 1.0;
    // return -value * 2 * Math.PI;
    double angle = -value * 2 * Math.PI;
    // x = -x;
    angle = -Math.PI / 2 + angle;
    return -angle;

  }

  public void reset() {
    pidRotate.setSetpoint(0);
    double s = pidRotate.calculate(canCoderPositionAdjusted());
    s = MathUtil.clamp(s, -ROTATION_LIMIT_SPEED, ROTATION_LIMIT_SPEED);
    rotateMotor.set(s);
  }

  public void stop() {
    driveMotor.stopMotor();
    rotateMotor.stopMotor();
  }

  public void setPIDValues(double p, double i, double d) {
    pidRotate.setPID(p, i, d);
  }

  public SwerveModulePosition getSwervePosition() {
    return new SwerveModulePosition(
        distanceEncoderPosition(), new Rotation2d(canCoderPositionAdjusted()));
  }

  public SwerveModuleState getSwerveState() {
    return new SwerveModuleState(
        distanceEncoder.getVelocity(), new Rotation2d(canCoderPositionAdjusted()));
  }
}
