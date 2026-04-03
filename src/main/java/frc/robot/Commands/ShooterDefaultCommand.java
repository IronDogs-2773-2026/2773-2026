// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.ShooterSubsystem;

/**
 * Default command for the shooter subsystem providing axis-based manual control.
 * 
 * <p>Controls (shooter operator Xbox controller):
 * <ul>
 *   <li><b>Left Y Axis</b> — Flywheel speed</li>
 *   <li><b>Left X Axis</b> — Feeder speed</li>
 *   <li><b>Right X Axis</b> — Intake speed</li>
 * </ul>
 * 
 * <p>This command runs continuously and is overridden by button-triggered commands.
 */
public class ShooterDefaultCommand extends Command {
  private ShooterSubsystem shoot;
  private XboxController xbox;

  /**
   * Constructs the ShooterDefaultCommand.
   * 
   * @param shoot Shooter subsystem
   * @param xbox Shooter operator Xbox controller
   */
  public ShooterDefaultCommand(ShooterSubsystem shoot, XboxController xbox) {
    this.shoot = shoot;
    this.xbox = xbox;
    addRequirements(shoot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoot.directRun(xbox.getLeftY());
    shoot.runFeeder(xbox.getLeftX());
    shoot.runIntake(xbox.getRightX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
