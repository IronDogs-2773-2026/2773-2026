// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.ShooterSubsystem;

/**
 * Command to run intake at 0.5 speed for 3 seconds.
 * Named for PathPlanner auto routines.
 */
public class Intake3 extends SequentialCommandGroup {
  public Intake3(ShooterSubsystem shooterSub) {
    super(
      Commands.runEnd(() -> shooterSub.runIntake(-0.5), () -> shooterSub.runIntake(0), shooterSub)
        .withTimeout(3.0)
    );
  }
}
