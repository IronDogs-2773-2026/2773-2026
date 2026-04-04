// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Autonomous.ManualBackupAndShoot;
import frc.robot.Autonomous.ShootSequence1;
import frc.robot.Autonomous.ShootSequence10;
import frc.robot.Autonomous.ShootSequence15;
import frc.robot.Autonomous.ShootSequence2;
import frc.robot.Autonomous.ShootSequence3;
import frc.robot.Autonomous.ShootSequence5;
import frc.robot.Commands.EmergencyInvertCommand;
import frc.robot.Commands.Intake1;
import frc.robot.Commands.Intake10;
import frc.robot.Commands.Intake2;
import frc.robot.Commands.Intake3;
import frc.robot.Commands.Intake5;
import frc.robot.Commands.Intake7;
import frc.robot.Commands.LowerArm;
import frc.robot.Commands.ShootSequenceCommand;
import frc.robot.Commands.ShooterDefaultCommand;
import frc.robot.Commands.XBOXDriveCommand;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.Information.VisionSubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/**
 * Main robot configuration class.
 * 
 * <p>This class is responsible for:
 * <ul>
 *   <li>Initializing all subsystems and controllers</li>
 *   <li>Configuring PathPlanner AutoBuilder and autonomous routines</li>
 *   <li>Registering NamedCommands for use in PathPlanner autos</li>
 *   <li>Binding driver and operator controls to commands</li>
 *   <li>Managing alliance selection (FMS or manual override)</li>
 * </ul>
 * 
 * <h3>Controller Layout:</h3>
 * <h4>Driver (Xbox Controller, Port 0)</h4>
 * <table>
 *   <tr><th>Input</th><th>Action</th></tr>
 *   <tr><td>Left Stick</td><td>Field-relative translation</td></tr>
 *   <tr><td>Right Stick</td><td>Field-relative rotation</td></tr>
 *   <tr><td>Right Trigger &gt; 0.5</td><td>Slowdown mode (0.6x speed)</td></tr>
 *   <tr><td>POV Up/Down</td><td>Adjust drive sensitivity</td></tr>
 *   <tr><td>B Button</td><td>Emergency 180° heading invert</td></tr>
 *   <tr><td>Buttons 7+8</td><td>Reset gyro heading offset</td></tr>
 * </table>
 * 
 * <h4>Shooter Operator (Xbox Controller, Port 2)</h4>
 * <table>
 *   <tr><th>Input</th><th>Action</th></tr>
 *   <tr><td>Left Trigger &gt; 0.5</td><td>Spin up flywheel (hold)</td></tr>
 *   <tr><td>Right Trigger &gt; 0.5</td><td>Run feeder + intake combo (hold)</td></tr>
 *   <tr><td>A Button</td><td>Toggle "Wyatt is Dumb" dashboard boolean</td></tr>
 *   <tr><td>B Button</td><td>Stop all shooter motors</td></tr>
 *   <tr><td>POV Up</td><td>Run arm up (hold)</td></tr>
 *   <tr><td>POV Down</td><td>Run arm down (hold)</td></tr>
 *   <tr><td>Left Bumper</td><td>Reverse intake (clear jams)</td></tr>
 *   <tr><td>Right Bumper</td><td>Run intake (hold)</td></tr>
 *   <tr><td>Left Stick</td><td>Reverse feeder (clear jams)</td></tr>
 *   <tr><td>Right Stick</td><td>Fast feeder (rapid shooting)</td></tr>
 *   <tr><td>Start</td><td>Zero arm position (calibration)</td></tr>
 *   <tr><td>Back</td><td>Toggle PID mode (logs to console)</td></tr>
 * </table>
 * 
 * <h3>Autonomous Routines:</h3>
 * <ul>
 *   <li>"Gunner's Auto" (default)</li>
 *   <li>"revised gunner auto"</li>
 *   <li>"noah auto"</li>
 *   <li>"Leftside Neutral Auto"</li>
 *   <li>"Simple Shoot" (non-PathPlanner, shoot-only routine)</li>
 * </ul>
 * 
 * <h3>NamedCommands (for PathPlanner):</h3>
 * <ul>
 *   <li>Shoot Sequence 1/2/3/5/10/15 — flywheel spinup + feed for N seconds</li>
 *   <li>Intake 1/2/3/5/7/10 — run intake at -0.5 for N seconds</li>
 *   <li>Lower Arm — pulse arm motor 3x (0.5s on/off)</li>
 * </ul>
 */
public class RobotContainer {
  /** Dashboard chooser for selecting autonomous routines. */
  private SendableChooser<Command> autoChooser;

  /** Dashboard chooser for alliance selection (FMS, Force Blue, Force Red). */
  private SendableChooser<String> allianceChooser;

  /** Driver Xbox controller (port 0). */
  XboxController xbox = new XboxController(0);

  /** Shooter operator Xbox controller (port 2). */
  XboxController shooterXbox = new XboxController(2);

  /** Swerve drive subsystem. */
  DriveSubsystem driveSub = new DriveSubsystem();

  /** Odometry subsystem with NavX gyro and pose estimation. */
  OdometrySubsystem odomSub = new OdometrySubsystem(driveSub);

  /** Vision subsystem for AprilTag pose estimation. */
  VisionSubsystem visionSub = new VisionSubsystem(this::acceptEstimatedRobotPose);

  /** Shooter subsystem (flywheel, feeder, arm, intake). */
  ShooterSubsystem shooterSub = new ShooterSubsystem();

  /** Default drive command for teleop (field-relative XBOX control). */
  XBOXDriveCommand driveCommand = new XBOXDriveCommand(driveSub, xbox, odomSub);
  // JoystickDriveCommand jdriveCommand = new JoystickDriveCommand(driveSub, joystick, odomSub);

  /**
   * Constructs the RobotContainer and initializes all subsystems, commands, and bindings.
   * 
   * <p>This constructor is called once during robot initialization and performs:
   * <ul>
   *   <li>PathPlanner AutoBuilder configuration</li>
   *   <li>Autonomous routine loading and chooser setup</li>
   *   <li>Alliance selector dashboard widget</li>
   *   <li>NamedCommand registration for PathPlanner</li>
   *   <li>Driver and operator control bindings</li>
   * </ul>
   */
  public RobotContainer() {
    System.out.println("RobotContainer: Initializing AutoBuilder...");

    // Initialize AutoBuilder after subsystems are created
    driveSub.initAutoBuilder(odomSub);

    System.out.println("RobotContainer: Building auto chooser...");

    // Create chooser and set default option
    autoChooser = new SendableChooser<>();
    autoChooser.setDefaultOption("Gunner's Auto", AutoBuilder.buildAuto("Gunner's Auto"));

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

    // Add "Gunner's Auto" full autonomous routine
    try {
      autoChooser.addOption("Gunner's Auto", AutoBuilder.buildAuto("Gunner's Auto"));
      System.out.println("RobotContainer: SUCCESS - Added 'Gunner's Auto' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'Gunner's Auto': " + e.getMessage());
      e.printStackTrace();
    }

    // Add "revised gunner auto" full autonomous routine
    try {
      autoChooser.addOption("revised gunner auto", AutoBuilder.buildAuto("revised gunner auto"));
      System.out.println("RobotContainer: SUCCESS - Added 'revised gunner auto' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'revised gunner auto': " + e.getMessage());
      e.printStackTrace();
    }

    // Add "noah auto" full autonomous routine
    try {
      autoChooser.addOption("noah auto", AutoBuilder.buildAuto("noah auto"));
      System.out.println("RobotContainer: SUCCESS - Added 'noah auto' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'noah auto': " + e.getMessage());
      e.printStackTrace();
    }

    // Add "Leftside Neutral Auto" full autonomous routine
    try {
      autoChooser.addOption("Leftside Neutral Auto", AutoBuilder.buildAuto("Leftside Neutral Auto"));
      System.out.println("RobotContainer: SUCCESS - Added 'Leftside Neutral Auto' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'Leftside Neutral Auto': " + e.getMessage());
      e.printStackTrace();
    }

    // Add "bump and shoot" full autonomous routine
    try {
      autoChooser.addOption("bump and shoot", AutoBuilder.buildAuto("bump and shoot"));
      System.out.println("RobotContainer: SUCCESS - Added 'bump and shoot' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'bump and shoot': " + e.getMessage());
      e.printStackTrace();
    }

    // Add "Do Not Use" full autonomous routine
    try {
      autoChooser.addOption("Do Not Use", AutoBuilder.buildAuto("Do Not Use"));
      System.out.println("RobotContainer: SUCCESS - Added 'Do Not Use' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'Do Not Use': " + e.getMessage());
      e.printStackTrace();
    }

    System.out.println("RobotContainer: AutoChooser created with options");
    System.out.println("RobotContainer: Putting AutoChooser on SmartDashboard...");
    SmartDashboard.putData("AutoChooser", autoChooser);
    System.out.println("RobotContainer: Done! Check SmartDashboard for 'AutoChooser'");

    // Simple shoot-only auto (no PathPlanner)
    try {
      autoChooser.addOption("Simple Shoot", new ShootSequenceCommand(shooterSub, 0.55, -0.75, 9.0, 1.5));
      System.out.println("RobotContainer: SUCCESS - Added 'Simple Shoot' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'Simple Shoot': " + e.getMessage());
      e.printStackTrace();
    }

    // Manual backup and shoot auto (no PathPlanner)
    try {
      autoChooser.addOption("Manual Backup & Shoot", new ManualBackupAndShoot(driveSub, odomSub, shooterSub));
      System.out.println("RobotContainer: SUCCESS - Added 'Manual Backup & Shoot' to chooser");
    } catch (Exception e) {
      System.err.println("FAILED to load 'Manual Backup & Shoot': " + e.getMessage());
      e.printStackTrace();
    }

    // Alliance selector dropdown
    allianceChooser = new SendableChooser<>();
    allianceChooser.setDefaultOption("Use FMS", "fms");
    allianceChooser.addOption("Force Blue", "blue");
    allianceChooser.addOption("Force Red", "red");
    SmartDashboard.putData("Alliance Selector", allianceChooser);
    
    // Update driveSub's alliance cache based on selector
    periodicAllianceUpdate();

    // Command scheduler
    driveSub.setDefaultCommand(driveCommand);

    // === AUTO PATH BUTTONS (COMMENTED OUT FOR COMPETITION) ===
    // A button: pathfind to start of "Red1" then follow it
    /*
    try {
      PathPlannerPath newPath = PathPlannerPath.fromPathFile("Red1");
      PathConstraints constraints = new PathConstraints(1.0, 1.0, Math.PI, Math.PI);
      new JoystickButton(xbox, XboxController.Button.kA.value)
          .whileTrue(AutoBuilder.followPath(newPath));
    } catch (Exception e) {
      System.err.println("FAILED to load 'Red1' for A button: " + e.getMessage());
    }
    */

    // Y button: follow "New New Path"
    /*
    try {
      PathPlannerPath newPath = PathPlannerPath.fromPathFile("New New Path");
      PathConstraints constraints = new PathConstraints(1.0, 1.0, Math.PI, Math.PI);
      new JoystickButton(xbox, XboxController.Button.kY.value)
          .whileTrue(AutoBuilder.followPath(newPath));
    } catch (Exception e) {
      System.err.println("FAILED to load 'New New Path' for Y button: " + e.getMessage());
    }
    */

    // X button: run "New Auto" autonomous routine
    /*
    try {
      new JoystickButton(xbox, XboxController.Button.kX.value)
          .whileTrue(AutoBuilder.buildAuto("New Auto"));
    } catch (Exception e) {
      System.err.println("FAILED to load 'New Auto' for X button: " + e.getMessage());
    }
    */

    NamedCommands.registerCommand("Shoot Sequence 1", new ShootSequence1(shooterSub));
    NamedCommands.registerCommand("Shoot Sequence 2", new ShootSequence2(shooterSub));
    NamedCommands.registerCommand("Shoot Sequence 3", new ShootSequence3(shooterSub));
    NamedCommands.registerCommand("Shoot Sequence 5", new ShootSequence5(shooterSub));
    NamedCommands.registerCommand("Shoot Sequence 10", new ShootSequence10(shooterSub));
    NamedCommands.registerCommand("Shoot Sequence 15", new ShootSequence15(shooterSub));
    NamedCommands.registerCommand("Lower Arm", new LowerArm(shooterSub));
    NamedCommands.registerCommand("Intake 1", new Intake1(shooterSub));
    NamedCommands.registerCommand("Intake 2", new Intake2(shooterSub));
    NamedCommands.registerCommand("Intake 3", new Intake3(shooterSub));
    NamedCommands.registerCommand("Intake 5", new Intake5(shooterSub));
    NamedCommands.registerCommand("Intake 7", new Intake7(shooterSub));
    NamedCommands.registerCommand("Intake 10", new Intake10(shooterSub));

    // // B button: drive 1 meter forward (field-relative)
    // new JoystickButton(xbox, XboxController.Button.kB.value)
    //     .whileTrue(new DriveDistanceCommand(driveSub, odomSub, -1.0, 0.0, 0.3));

    // B button: emergency 180° invert
    new JoystickButton(xbox, XboxController.Button.kB.value)
        .onTrue(new EmergencyInvertCommand(driveCommand));

    // Right bumper: one-button shoot sequence (0.6 speed, 0.5 feeder, 1s shoot, 0.5s spinup)
    // new JoystickButton(xbox, XboxController.Button.kRightBumper.value)
    //     .whileTrue(new ShootSequenceCommand(shooterSub, 0.6, 0.5, 1.0, 0.5));

    // === SHOOTER CONTROLLER (Xbox port 2) ===
    // Default command: axis-based manual control for all shooter functions
    shooterSub.setDefaultCommand(new ShooterDefaultCommand(shooterSub, shooterXbox));

    // A button: run feeder only (for testing/intake)
    new JoystickButton(shooterXbox, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(() -> SmartDashboard.putBoolean("Wyatt is Dumb", true))
            .finallyDo(() -> SmartDashboard.putBoolean("Wyatt is Dumb", false)));

    // B button: stop all shooter motors
    new JoystickButton(shooterXbox, XboxController.Button.kB.value)
        .whileTrue(new RunCommand(() -> shooterSub.stop(), shooterSub));

    // POV Up: run arm up (hold to raise arm)
    new Trigger(() -> shooterXbox.getPOV() == 180)
        .whileTrue(new RunCommand(() -> shooterSub.runArm(0.12))
            .finallyDo(() -> shooterSub.runArm(0)));

    // POV Down: run arm down (hold to lower arm)
    new Trigger(() -> shooterXbox.getPOV() == 0)
        .whileTrue(new RunCommand(() -> shooterSub.runArm(-0.4))
            .finallyDo(() -> shooterSub.runArm(0)));

    // Left bumper: reverse intake + feeder + flywheel outtake (for clearing jams)
    new JoystickButton(shooterXbox, XboxController.Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> {
            shooterSub.runIntake(0.5);
            shooterSub.runFeeder(0.5);
        }, shooterSub).finallyDo(() -> {
            shooterSub.runIntake(0);
            shooterSub.runFeeder(0);
        }));

    // Right bumper: run intake (hold to intake)
    new JoystickButton(shooterXbox, XboxController.Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> shooterSub.runIntake(-0.9), shooterSub));

    // Left trigger (>0.5): spin up flywheel only (hold to run)
    new Trigger(() -> shooterXbox.getLeftTriggerAxis() > 0.5)
        .whileTrue(new RunCommand(() -> shooterSub.directRun(0.9), shooterSub));

    // Right trigger (>0.5): run feeder + intake combo (hold to run)
    // No subsystem requirement so it can run alongside left trigger flywheel
    new Trigger(() -> shooterXbox.getRightTriggerAxis() > 0.5)
        .whileTrue(Commands.runEnd(
            () -> { shooterSub.runFeeder(-0.7); shooterSub.runIntake(-0.6); },
            () -> { shooterSub.runFeeder(0); shooterSub.runIntake(0); }));

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

  /**
   * Callback for vision subsystem to add pose measurements to odometry.
   * 
   * @param pose The estimated robot pose from vision processing
   * @param timestamp The FPGA timestamp of the measurement
   * @param estimationStdDevs Standard deviations of the pose estimate (3x1 vector)
   */
  void acceptEstimatedRobotPose(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    odomSub.addVisionMeasurement(pose, timestamp, estimationStdDevs);
  }

  /**
   * Updates the alliance cache in the drive subsystem based on the dashboard selector.
   * 
   * <p>Priority order:
   * <ol>
   *   <li>Manual override ("Force Blue" or "Force Red")</li>
   *   <li>FMS-reported alliance (if available)</li>
   * </ol>
   */
  void periodicAllianceUpdate() {
    String selection = allianceChooser.getSelected();
    if (selection == null) selection = "fms";
    
    // Set the cached alliance in driveSub based on dropdown
    if (selection.equals("red")) {
      driveSub.setAlliance(DriverStation.Alliance.Red);
    } else if (selection.equals("blue")) {
      driveSub.setAlliance(DriverStation.Alliance.Blue);
    } else {
      // "fms" - sync with actual FMS
      var alliance = DriverStation.getAlliance();
      if (alliance.isPresent()) {
        driveSub.setAlliance(alliance.get());
      }
    }
  }
  
  /**
   * Called every robot periodic cycle to refresh the alliance cache from FMS.
   * 
   * <p>This is invoked from {@code Robot.robotPeriodic()} to ensure the drive
   * subsystem's alliance state stays synchronized with the current match.
   */
  void updateAllianceCache() {
    periodicAllianceUpdate();
  }

  /**
   * Returns the currently selected autonomous command.
   * 
   * @return The command selected on the dashboard AutoChooser, or null if none selected
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
