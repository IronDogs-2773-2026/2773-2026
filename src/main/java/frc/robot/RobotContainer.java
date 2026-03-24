// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
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
    autoChooser = AutoBuilder.buildAutoChooser();
    
    if (autoChooser == null) {
      System.err.println("ERROR: AutoBuilder.buildAutoChooser() returned null!");
      autoChooser = new SendableChooser<>();
      autoChooser.setDefaultOption("No Auto Available", null);
    } else {
      System.out.println("RobotContainer: AutoChooser built successfully");
      System.out.println("RobotContainer: AutoChooser is ready");
    }
    
    System.out.println("RobotContainer: Putting Auto Chooser on SmartDashboard...");
    SmartDashboard.putData("AutoChooser", autoChooser);
    SmartDashboard.putData("Auto Chooser", autoChooser);
    System.out.println("RobotContainer: Auto chooser created and added to SmartDashboard!");
    System.out.println("RobotContainer: Check SmartDashboard for 'AutoChooser' or 'Auto Chooser'");

    // Command scheduler
    driveSub.setDefaultCommand(driveCommand);
  }

  void acceptEstimatedRobotPose(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    odomSub.addVisionMeasurement(pose, timestamp, estimationStdDevs);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
