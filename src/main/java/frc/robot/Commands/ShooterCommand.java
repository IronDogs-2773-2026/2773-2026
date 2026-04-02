// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ShooterSubsystem;

/**
 * Command to run the shooter flywheel at a set speed using direct (open-loop) control.
 * Once at speed, optionally runs the feeder to shoot notes.
 */
public class ShooterCommand extends SequentialCommandGroup {

  /**
   * @param shooterSub    Shooter subsystem
   * @param flywheelSpeed Flywheel speed (-1.0 to 1.0)
   * @param feederSpeed   Feeder speed (0 to 1), or 0 to disable
   * @param useFeederIntake if true, runs feeder after spinup delay
   * @param pidRun        If true, use PID control; if false, use direct control
   */
  public ShooterCommand(ShooterSubsystem shooterSub, double flywheelSpeed, double feederSpeed, double intakeSpeed, boolean useFeederIntake, boolean pidRun) {
    // Build the sequence: spin up → wait → shoot (if enabled)
    addCommands(
      // Spin up flywheel
      Commands.runOnce(() -> shooterSub.directRun(flywheelSpeed), shooterSub),

      // Run feeder after a brief delay (if enabled)
      useFeederIntake 
        ? Commands.sequence(
            Commands.waitSeconds(1.5),
            Commands.runEnd(
              () -> {
                shooterSub.runFeeder(-feederSpeed);
                shooterSub.runIntake(-intakeSpeed);
              },
              () -> {
                shooterSub.runFeeder(0);
                shooterSub.runIntake(0);
              },
              shooterSub
            )
          )
        : Commands.runOnce(() -> shooterSub.runFeeder(0), shooterSub)
    );
  }
}
