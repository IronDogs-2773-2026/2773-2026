// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Commands.*;
import frc.robot.Information.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {
  // Autonomous chooser
  private SendableChooser<Command> autoChooser;

  // Controllers
  XboxController xbox = new XboxController(0);
  Joystick joystick = new Joystick(1);

  // Subsystems
  DriveSubsystem driveSub = new DriveSubsystem();
  OdometrySubsystem odomSub = new OdometrySubsystem(driveSub);
  VisionSubsystem visionSub = new VisionSubsystem(this::acceptEstimatedRobotPose);

  // Commands from files
  XBOXDriveCommand driveCommand = new XBOXDriveCommand(driveSub, xbox, odomSub);
  // JoystickDriveCommand jdriveCommand = new JoystickDriveCommand(driveSub, joystick, odomSub);

  public RobotContainer() {
    System.out.println("RobotContainer: Initializing AutoBuilder...");

    // Initialize AutoBuilder after subsystems are created
    driveSub.initAutoBuilder(odomSub);

    System.out.println("RobotContainer: Building auto chooser...");

    // Create chooser and set default option
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("None", null);

    // Try to load paths manually
    try {
      PathPlannerPath examplePath = PathPlannerPath.fromPathFile("Example Path");
      autoChooser.addOption("Example Path", AutoBuilder.followPath(examplePath));
      System.out.println("RobotContainer: SUCCESS - Added 'Example Path' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'Example Path': " + e.getMessage());
      e.printStackTrace();
    }

    try {
      PathPlannerPath newPath = PathPlannerPath.fromPathFile("New Path");
      autoChooser.addOption("New Path", AutoBuilder.followPath(newPath));
      System.out.println("RobotContainer: SUCCESS - Added 'New Path' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'New Path': " + e.getMessage());
      e.printStackTrace();
    }

    System.out.println("RobotContainer: AutoChooser created with options");
    System.out.println("RobotContainer: Putting AutoChooser on SmartDashboard...");
    SmartDashboard.putData("AutoChooser", autoChooser);
    System.out.println("RobotContainer: Done! Check SmartDashboard for 'AutoChooser'");

    // Command scheduler
    driveSub.setDefaultCommand(driveCommand);
  }

  {
    try {
      PathPlannerPath newPath = PathPlannerPath.fromPathFile("New Path");
      System.out.println("RobotContainer: SUCCESS - Added 'New Path' to chooser");
        new JoystickButton(xbox, XboxController.Button.kA.value)
        .whileTrue(AutoBuilder.followPath(newPath));
    } catch (Exception e) {
      System.err.println("FAILED to load 'New Path': " + e.getMessage());
      e.printStackTrace();
    }
  }

  void acceptEstimatedRobotPose(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    odomSub.addVisionMeasurement(pose, timestamp, estimationStdDevs);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
