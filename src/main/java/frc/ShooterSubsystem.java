// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

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

public class ShooterSubsystem extends SubsystemBase {
  private SparkMax flyWheelOne = new SparkMax(Constants.flyWheel1, MotorType.kBrushless);
  private SparkMax flyWheelTwo = new SparkMax(Constants.flyWheel2, MotorType.kBrushless);
  private SparkMax feederMotor = new SparkMax(Constants.feederMotor, MotorType.kBrushless);
  private SparkMax armMotor = new SparkMax(Constants.armMotor, MotorType.kBrushless);
  
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
  }

  @Override
  public void periodic() {
    // Runs every 20ms
  }

  /**
   * Direct run function to set flywheel speed directly
   * @param speed Speed from -1.0 to 1.0
   */
  public void directRun(double speed) {
    // Only run one flywheel motor
    flyWheelOne.set(speed);
  }

  /**
   * PID run function to reach target velocity
   * @param targetRPM Target velocity in RPM
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
   * Stop all shooter motors
   */
  public void stop() {
    flyWheelOne.set(0);
    flyWheelTwo.set(0);
    feederMotor.set(0);
    armMotor.set(0);
  }

  /**
   * Run the feeder motor
   * @param speed Speed from -1.0 to 1.0
   */
  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }

  /**
   * Set the arm motor speed
   * @param speed Speed from -1.0 to 1.0
   */
  public void setArmSpeed(double speed) {
    armMotor.set(speed);
  }

  /**
   * Check if flywheels are at target speed
   * @return true if within tolerance
   */
  public boolean atSetpoint() {
    return pidController.atSetpoint();
  }
}
