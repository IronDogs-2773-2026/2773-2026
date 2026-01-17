// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Commands.*;
import frc.robot.Information.*;

public class RobotContainer {
  public RobotContainer() {

  }

  // Base inits
  // Controllers
  XboxController xbox = new XboxController(2);

  // Subsystems
  DriveSubsystem driveSub = new DriveSubsystem();
  OdometrySubsystem odomSub = new OdometrySubsystem(driveSub);
  TagSubsystem tagSub = new TagSubsystem(odomSub);

  // Commands from files
  XBOXDriveCommand driveCommand = new XBOXDriveCommand(driveSub, xbox, tagSub, odomSub);

  // Command scheduler
  {
    driveSub.setDefaultCommand(driveCommand);

  }

  // Autonomous chooser

  // return new DeltaPoseCommand(0, 1.5, 0, driveSub, odomSub);
}
