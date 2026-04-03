// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Commands.XBOXDriveCommand;

/**
 * Command that instantly flips the drive heading 180 degrees.
 * Used for emergency direction correction.
 */
public class EmergencyInvertCommand extends InstantCommand {
  private final XBOXDriveCommand driveCommand;

  public EmergencyInvertCommand(XBOXDriveCommand driveCommand) {
    this.driveCommand = driveCommand;
  }

  @Override
  public void initialize() {
    driveCommand.invertHeading();
  }
}
