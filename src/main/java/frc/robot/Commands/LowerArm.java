// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterSubsystem;

/**
 * Command that pulses the arm motor for 0.5 seconds three times
 * with 0.5 second breaks in between.
 */
public class LowerArm extends Command {
  private final ShooterSubsystem shooterSub;
  private int pulseCount = 0;
  private double startTime = 0;
  private boolean isPulsing = true;

  /**
   * @param shooterSub Shooter subsystem
   */
  public LowerArm(ShooterSubsystem shooterSub) {
    this.shooterSub = shooterSub;
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
    pulseCount = 0;
    isPulsing = true;
    startTime = System.currentTimeMillis() / 1000.0;
  }

  @Override
  public void execute() {
    double currentTime = System.currentTimeMillis() / 1000.0;
    double elapsedTime = currentTime - startTime;

    if (isPulsing) {
      // Pulse arm motor for 0.5 seconds
      shooterSub.runArm(0.1);
      if (elapsedTime >= 0.5) {
        isPulsing = false;
        startTime = currentTime;
      }
    } else {
      // Stop arm motor for 0.5 seconds
      shooterSub.runArm(0);
      if (elapsedTime >= 0.5) {
        isPulsing = true;
        pulseCount++;
        startTime = currentTime;
      }
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.runArm(0);
  }

  @Override
  public boolean isFinished() {
    return pulseCount >= 3;
  }
}
