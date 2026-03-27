// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkAnalogSensor;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private SparkMax flyWheelOne = new SparkMax(Constants.flyWheel1, MotorType.kBrushless);
  private SparkMax flyWheelTwo = new SparkMax(Constants.flyWheel2, MotorType.kBrushless);
  private SparkMax feederMotor = new SparkMax(Constants.feederMotor, MotorType.kBrushless);
  
  private PIDController pidController = new PIDController(0.1, 0.0, 0.0);

  public ShooterSubsystem() {
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
    flyWheelOne.set(speed);
    flyWheelTwo.set(speed);
  }

  /**
   * PID run function to reach target velocity
   * @param targetRPM Target velocity in RPM
   */
  public void pidRun(double targetRPM) {
    double currentRPM = flyWheelOne.getEncoder().getVelocity();
    double output = pidController.calculate(currentRPM, targetRPM);
    directRun(output);
  }

  /**
   * Stop all shooter motors
   */
  public void stop() {
    flyWheelOne.set(0);
    flyWheelTwo.set(0);
    feederMotor.set(0);
  }

  /**
   * Run the feeder motor
   * @param speed Speed from -1.0 to 1.0
   */
  public void runFeeder(double speed) {
    feederMotor.set(speed);
  }
}
