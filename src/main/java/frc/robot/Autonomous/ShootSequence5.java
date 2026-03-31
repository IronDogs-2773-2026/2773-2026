// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.ShooterSubsystem;
import frc.robot.Commands.ShootSequenceCommand;

/**
 * Runs a shoot sequence with 5 second feed time.
 * Uses command composition to properly finish when the sequence completes.
 */
public class ShootSequence5 extends Command {
  private final Command m_sequence;

  public ShootSequence5(ShooterSubsystem shooterSub) {
    m_sequence = new ShootSequenceCommand(shooterSub, 0.8, 0.7, 5, 1.5);
    addRequirements(shooterSub);
  }

  @Override
  public void initialize() {
    m_sequence.schedule();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_sequence.cancel();
  }

  @Override
  public boolean isFinished() {
    return m_sequence.isFinished();
  }
}
