// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

/**
 * Shooter subsystem controlling the flywheel, feeder, arm, and intake mechanisms.
 * 
 * <p>This subsystem manages five motors:
 * <ul>
 *   <li><b>Flywheel 1 & 2</b> — Counter-rotating flywheel motors for launching game pieces.
 *       Motor 2 follows Motor 1 in inverted mode for opposite rotation.</li>
 *   <li><b>Feeder Motor</b> — Pushes game pieces from the intake into the flywheel.</li>
 *   <li><b>Arm Motor</b> — Adjusts the shooter arm angle for different shot distances.</li>
 *   <li><b>Intake Motor</b> — Pulls game pieces into the robot from the field.</li>
 * </ul>
 * 
 * <p>Flywheel control supports two modes:
 * <ul>
 *   <li><b>Direct (open-loop)</b> — Set motor speed directly via {@link #directRun(double)}</li>
 *   <li><b>PID (closed-loop)</b> — Target RPM via {@link #pidRun(double)} using encoder feedback</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase {
  private SparkMax flyWheelOne = new SparkMax(Constants.flyWheel1, MotorType.kBrushless);
  private SparkMax flyWheelTwo = new SparkMax(Constants.flyWheel2, MotorType.kBrushless);
  private SparkMax feederMotor = new SparkMax(Constants.feederMotor, MotorType.kBrushless);
  private SparkMax armMotor = new SparkMax(Constants.armMotor, MotorType.kBrushless);
  private SparkMax intakeMotor = new SparkMax(Constants.intakeMotor, MotorType.kBrushless);
  
  private PIDController pidController = new PIDController(0.1, 0.0, 0.0);

  public ShooterSubsystem() {
    // Set PID tolerance for at-setpoint detection (±50 RPM)
    pidController.setTolerance(50.0);

    // Configure current limits (40A typical for NEOs)
    var config = new SparkMaxConfig();
    config.smartCurrentLimit(40);

    try {
      // Flywheel 1 - normal direction
      flyWheelOne.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      
      // Flywheel 2 - inverted (opposite direction for counter-rotating flywheels)
      var config2 = new SparkMaxConfig();
      config2.smartCurrentLimit(40);
      config2.follow(Constants.flyWheel1, true);
      flyWheelTwo.configure(config2, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

      // Feeder motor - normal direction
      feederMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
      
      // Arm motor - normal direction
      armMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to configure shooter motor current limits", true);
    }
    stop();
  }

  @Override
  public void periodic() {
    // Runs every 20ms
  }

  /**
   * Directly sets the flywheel motor speed (open-loop control).
   * Only motor 1 is driven; motor 2 follows as a follower.
   * 
   * @param speed Motor speed from -1.0 (full reverse) to 1.0 (full forward)
   */
  public void directRun(double speed) {
    // Only run one flywheel motor
    flyWheelOne.set(speed);
  }

  /**
   * Runs the flywheel motors to a target RPM using PID control.
   * Averages both flywheel encoder readings for accurate speed feedback.
   * 
   * @param targetRPM Target flywheel velocity in RPM
   */
  public void pidRun(double targetRPM) {
    // Average both flywheel encoders for accurate speed reading
    // flyWheelTwo is inverted, so negate its velocity to get positive RPM
    double currentRPM = (flyWheelOne.getEncoder().getVelocity() +
                         -flyWheelTwo.getEncoder().getVelocity()) / 2.0;
    double output = pidController.calculate(currentRPM, targetRPM);
    output = MathUtil.clamp(output, -1.0, 1.0);
    directRun(output);
  }

  /**
   * Stops all shooter motors (flywheel, feeder, arm, intake).
   */
  public void stop() {
    flyWheelOne.set(0);
    flyWheelTwo.set(0);
    feederMotor.set(0);
    armMotor.set(0);
    intakeMotor.set(0);
  }

  /**
   * Sets the feeder motor speed.
   * 
   * @param speed Motor speed from -1.0 (full reverse) to 1.0 (full forward)
   */
  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }

  /**
   * Sets the arm motor speed.
   * 
   * @param speed Motor speed from -1.0 (full reverse) to 1.0 (full forward)
   */
  public void runArm(double speed) {
    armMotor.set(speed);
  }

  /**
   * Sets the intake motor speed.
   * 
   * @param speed Motor speed from -1.0 (full reverse) to 1.0 (full forward)
   */
  public void runIntake(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * Set the arm motor speed
   * @param speed Speed from -1.0 to 1.0
   */
  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  /**
   * Checks if the flywheel motors have reached the target PID setpoint.
   * 
   * @return true if within the configured PID tolerance (±50 RPM)
   */
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }
}
