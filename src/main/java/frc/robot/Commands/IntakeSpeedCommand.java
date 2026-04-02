// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterSubsystem;

/**
 * Command to run the shooter intake at a set speed.
 * Used by PathPlanner auto routines.
 */
public class IntakeSpeedCommand extends Command {
  private final ShooterSubsystem shooterSub;
  private final double intakeSpeed;

  /**
   * @param shooterSub   Shooter subsystem
   * @param intakeSpeed  Intake speed (-1.0 to 1.0)
   */
  public IntakeSpeedCommand(ShooterSubsystem shooterSub, double intakeSpeed) {
    this.shooterSub = shooterSub;
    this.intakeSpeed = intakeSpeed;
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.runIntake(intakeSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Runs indefinitely until interrupted
  }
}
