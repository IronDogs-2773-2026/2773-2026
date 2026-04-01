// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.ShootSequence10;
import frc.robot.Autonomous.ShootSequence15;
import frc.robot.Autonomous.ShootSequence5;
import frc.robot.Commands.DriveDistanceCommand;
import frc.robot.Commands.LowerArm;
import frc.robot.Commands.RunFeederCommand;
import frc.robot.Commands.ShootSequenceCommand;
import frc.robot.Commands.ShooterCommand;
import frc.robot.Commands.ShooterDefaultCommand;
import frc.robot.Commands.XBOXDriveCommand;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.Information.VisionSubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

public class RobotContainer {
  // Autonomous chooser
  private SendableChooser<Command> autoChooser;

  // Controllers
  XboxController xbox = new XboxController(0);
  XboxController shooterXbox = new XboxController(2);

  // Subsystems
  DriveSubsystem driveSub = new DriveSubsystem();
  OdometrySubsystem odomSub = new OdometrySubsystem(driveSub);
  VisionSubsystem visionSub = new VisionSubsystem(this::acceptEstimatedRobotPose);
  ShooterSubsystem shooterSub = new ShooterSubsystem();

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

    // A button: pathfind to start of "Red1" then follow it
    try {
      PathPlannerPath newPath = PathPlannerPath.fromPathFile("Red1");
      PathConstraints constraints = new PathConstraints(1.0, 1.0, Math.PI, Math.PI);
      new JoystickButton(xbox, XboxController.Button.kA.value)
          .whileTrue(AutoBuilder.followPath(newPath));
    } catch (Exception e) {
      System.err.println("FAILED to load 'Red1' for A button: " + e.getMessage());
    }

    // Y button: follow "New New Path"
    try {
      PathPlannerPath newPath = PathPlannerPath.fromPathFile("New New Path");
      PathConstraints constraints = new PathConstraints(1.0, 1.0, Math.PI, Math.PI);
      new JoystickButton(xbox, XboxController.Button.kY.value)
          .whileTrue(AutoBuilder.followPath(newPath));
    } catch (Exception e) {
      System.err.println("FAILED to load 'New New Path' for Y button: " + e.getMessage());
    }

    // X button: run "New Auto" autonomous routine
    try {
      new JoystickButton(xbox, XboxController.Button.kX.value)
          .whileTrue(AutoBuilder.buildAuto("New Auto"));
    } catch (Exception e) {
      System.err.println("FAILED to load 'New Auto' for X button: " + e.getMessage());
    }

    NamedCommands.registerCommand("Shoot Sequence 5", new ShootSequence5(shooterSub));
    NamedCommands.registerCommand("Shoot Sequence 10", new ShootSequence10(shooterSub));
    NamedCommands.registerCommand("Shoot Sequence 15", new ShootSequence15(shooterSub));
    NamedCommands.registerCommand("Lower Arm", new LowerArm(shooterSub));

    // B button: drive 1 meter forward (field-relative)
    new JoystickButton(xbox, XboxController.Button.kB.value)
        .whileTrue(new DriveDistanceCommand(driveSub, odomSub, -1.0, 0.0, 0.3));

    // Right bumper: one-button shoot sequence (0.6 speed, 0.5 feeder, 1s shoot, 0.5s spinup)
    new JoystickButton(xbox, XboxController.Button.kRightBumper.value)
        .whileTrue(new ShootSequenceCommand(shooterSub, 0.6, 0.5, 1.0, 0.5));

    // === SHOOTER CONTROLLER (Xbox port 2) ===
    // Default command: axis-based manual control for all shooter functions
    shooterSub.setDefaultCommand(new ShooterDefaultCommand(shooterSub, shooterXbox));

    // A button: run feeder only (for testing/intake)
    new JoystickButton(shooterXbox, XboxController.Button.kA.value)
        .whileTrue(new RunFeederCommand(shooterSub, 0.5));

    // B button: stop all shooter motors
    new JoystickButton(shooterXbox, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> shooterSub.stop(), shooterSub));

    // X button: run intake (hold to intake)
    new JoystickButton(shooterXbox, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(() -> shooterSub.runIntake(-0.8), shooterSub));

    // Y button: reverse intake (for clearing jams)
    new JoystickButton(shooterXbox, XboxController.Button.kY.value)
        .whileTrue(new RunCommand(() -> shooterSub.runIntake(0.5), shooterSub));

    // Left bumper: run arm full reverse (hold to lower arm)
    new JoystickButton(shooterXbox, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> shooterSub.runArm(-0.2), shooterSub));

    // Right bumper: run arm full forward (hold to raise arm)
    new JoystickButton(shooterXbox, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> shooterSub.runArm(0.2), shooterSub));

    // Left trigger (>0.5): manual flywheel control (hold to spin)
    new Trigger(() -> shooterXbox.getLeftTriggerAxis() > 0.5)
        .whileTrue(new ShooterCommand(shooterSub, 0.9, 0.0, 0.0, false));

    // Right trigger (>0.5): shoot sequence (spin up, wait, then shoot)
    new Trigger(() -> shooterXbox.getRightTriggerAxis() > 0.5)
        .whileTrue(new ShooterCommand(shooterSub, 0.8, 0.6, 0.5, true));

    // Left stick button: reverse feeder (for clearing jams)
    new JoystickButton(shooterXbox, XboxController.Button.kLeftStick.value)
        .whileTrue(new RunCommand(() -> shooterSub.runFeeder(0.3), shooterSub));

    // Right stick button: fast feeder (for rapid shooting)
    new JoystickButton(shooterXbox, XboxController.Button.kRightStick.value)
        .whileTrue(new RunCommand(() -> shooterSub.runFeeder(-1.0), shooterSub));

    // Start button: zero arm position (calibration)
    new JoystickButton(shooterXbox, XboxController.Button.kStart.value)
        .onTrue(new RunCommand(() -> shooterSub.runArm(0), shooterSub));

    // Back/Select button: toggle PID mode (placeholder - logs to console)
    new JoystickButton(shooterXbox, XboxController.Button.kBack.value)
        .onTrue(new RunCommand(() -> System.out.println("PID mode toggled"), shooterSub));
  }

  void acceptEstimatedRobotPose(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    odomSub.addVisionMeasurement(pose, timestamp, estimationStdDevs);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
