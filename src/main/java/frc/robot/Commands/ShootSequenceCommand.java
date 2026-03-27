// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.ShooterSubsystem;

/**
 * One-button shoot sequence command.
 * Spins up flywheel, waits, then runs feeder to shoot.
 */
public class ShootSequenceCommand extends Command {
  private final Command sequence;
  private final double spinupTime;

  /**
   * @param shooterSub  Shooter subsystem
   * @param flywheelSpeed Flywheel speed (-1.0 to 1.0)
   * @param feederSpeed Feeder speed (0 to 1)
   * @param shootTime   How long to run feeder in seconds
   * @param spinupTime  How long to wait before feeding (seconds)
   */
  public ShootSequenceCommand(ShooterSubsystem shooterSub, double flywheelSpeed, double feederSpeed, double shootTime, double spinupTime) {
    this.spinupTime = spinupTime;
    
    // Build the sequence: spin up → wait → shoot → stop
    this.sequence = Commands.sequence(
      // Spin up flywheel
      Commands.runOnce(() -> shooterSub.directRun(flywheelSpeed), shooterSub),
      
      // Wait for flywheel to spin up
      Commands.waitSeconds(spinupTime),
      
      // Run feeder to shoot the note
      Commands.run(() -> shooterSub.runFeeder(feederSpeed), shooterSub)
        .withTimeout(shootTime),
      
      // Stop all motors
      Commands.runOnce(() -> shooterSub.stop(), shooterSub)
    );
    
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
    sequence.initialize();
  }

  @Override
  public void execute() {
    sequence.execute();
  }

  @Override
  public void end(boolean interrupted) {
    sequence.end(interrupted);
  }

  @Override
  public boolean isFinished() {
    return sequence.isFinished();
  }
}
