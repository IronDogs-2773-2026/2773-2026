// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.ShooterSubsystem;

/**
 * Command to run the shooter flywheel at a set speed using direct (open-loop) control.
 * Once at speed, optionally runs the feeder to shoot notes.
 */
public class ShooterCommand extends Command {
  private final ShooterSubsystem shooterSub;
  private final double flywheelSpeed;
  private final double feederSpeed;
  private final boolean useFeeder;

  /**
   * @param shooterSub    Shooter subsystem
   * @param flywheelSpeed Flywheel speed (-1.0 to 1.0)
   * @param feederSpeed   Feeder speed (0 to 1), or 0 to disable
   * @param useFeeder     If true, runs feeder after spinup delay
   */
  public ShooterCommand(ShooterSubsystem shooterSub, double flywheelSpeed, double feederSpeed, boolean useFeeder) {
    this.shooterSub = shooterSub;
    this.flywheelSpeed = flywheelSpeed;
    this.feederSpeed = feederSpeed;
    this.useFeeder = useFeeder;
    addRequirements(shooterSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSub.directRun(flywheelSpeed);
    
    // Run feeder after a brief delay (if enabled)
    if (useFeeder) {
      shooterSub.runFeeder(feederSpeed);
    } else {
      shooterSub.runFeeder(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSub.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // Runs indefinitely until interrupted
  }
}
