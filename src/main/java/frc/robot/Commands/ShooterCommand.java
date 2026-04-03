// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.ShooterSubsystem;

/**
 * Command to run the shooter flywheel and optionally the feeder/intake.
 * 
 * <p>Sequence:
 * <ol>
 *   <li>Spin up flywheel to target speed</li>
 *   <li>Wait 1.5 seconds for flywheel to reach speed</li>
 *   <li>If {@code useFeederIntake} is true, run feeder and intake motors</li>
 * </ol>
 * 
 * <p>Used for teleop right trigger (hold to run continuously).
 * Motors stop automatically when the command is interrupted.
 */
public class ShooterCommand extends SequentialCommandGroup {

  /**
   * Constructs the ShooterCommand.
   * 
   * @param shooterSub Shooter subsystem
   * @param flywheelSpeed Flywheel speed (-1.0 to 1.0)
   * @param feederSpeed Feeder speed (0 to 1), or 0 to disable
   * @param intakeSpeed Intake speed (0 to 1), or 0 to disable
   * @param useFeederIntake If true, runs feeder after spinup delay
   * @param pidRun If true, use PID control; if false, use direct control (currently unused)
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
