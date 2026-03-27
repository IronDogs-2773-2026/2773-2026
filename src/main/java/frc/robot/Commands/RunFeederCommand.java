// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.ShooterSubsystem;

/**
 * Command to run the feeder motor at a constant speed.
 */
public class RunFeederCommand extends Command {
  private final ShooterSubsystem shooterSub;
  private final double speed;

  /**
   * @param shooterSub Shooter subsystem
   * @param speed Feeder speed (0 to 1), positive shoots, negative intakes
   */
  public RunFeederCommand(ShooterSubsystem shooterSub, double speed) {
    this.shooterSub = shooterSub;
    this.speed = speed;
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    shooterSub.runFeeder(speed);
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.runFeeder(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
