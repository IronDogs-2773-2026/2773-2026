// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ShooterSubsystem;

/**
 * One-button shoot sequence command.
 *
 * <p>Executes the following sequence:
 * <ol>
 *   <li>Spin up flywheel to target speed</li>
 *   <li>Wait for spinup duration</li>
 *   <li>Run feeder to shoot game piece</li>
 *   <li>Stop all shooter motors</li>
 * </ol>
 *
 * <p>Used for both teleop (right trigger) and PathPlanner named commands.
 */
public class ShootSequenceCommand extends SequentialCommandGroup {

  /**
   * Constructs the ShootSequenceCommand.
   *
   * @param shooterSub Shooter subsystem
   * @param flywheelSpeed Flywheel speed (-1.0 to 1.0)
   * @param feederSpeed Feeder speed (-1.0 to 1.0)
   * @param shootTime How long to run feeder in seconds
   * @param spinupTime How long to wait before feeding (seconds)
   */
  public ShootSequenceCommand(ShooterSubsystem shooterSub, double flywheelSpeed, double feederSpeed, double shootTime, double spinupTime) {
    addCommands(
      // Spin up flywheel
      Commands.runOnce(() -> shooterSub.directRun(flywheelSpeed), shooterSub),

      // Wait for flywheel to spin up
      Commands.waitSeconds(spinupTime),

      // Run feeder to shoot the note
      Commands.runEnd(() -> shooterSub.runFeeder(feederSpeed), () -> shooterSub.runFeeder(0), shooterSub)
        .withTimeout(shootTime),

      // Stop all motors
      Commands.runOnce(() -> shooterSub.stop(), shooterSub)
    );
  }
}
