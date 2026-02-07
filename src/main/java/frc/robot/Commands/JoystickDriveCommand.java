// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.Commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Information.OdometrySubsystem;
// import frc.robot.SwerveSubsystems.DriveSubsystem;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class JoystickDriveCommand extends Command {
//   private final DriveSubsystem driveSub;
//   private final Joystick joystick;
//   private final OdometrySubsystem odomSub;
//   private final PIDController pid;
//   private final PIDController rotPid;
//   /** Creates a new JoystickDriveCommand. */
//   public JoystickDriveCommand(DriveSubsystem driveSub, Joystick joystick, OdometrySubsystem odomSub) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     this.driveSub = driveSub;
//     this.joystick = joystick;
//     this.odomSub = odomSub;
//     this.pid = driveSub.getPID();
//     this.rotPid = new PIDController(0, 0, 0);
//     addRequirements(driveSub);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     odomSub.getGyroAngle();
//     odomSub.resetGyro();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {

//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   private void doDrive() {
//     double XAxis = joystick.getX(), YAxis = joystick.getY(), ZAxis = joystick.getZ();
//     double rawAngle = Math.atan2(YAxis, XAxis);
//     double gyroAngle = odomSub.getGyroAngle();
//     double deltaAngle = rawAngle - gyroAngle;
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
