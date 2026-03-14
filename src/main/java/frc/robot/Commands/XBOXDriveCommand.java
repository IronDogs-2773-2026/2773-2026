// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SwerveSubsystems.*;
import frc.robot.Information.*;
import frc.robot.Constants;

public class XBOXDriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final XboxController xbox;
  private final OdometrySubsystem odomSub;
  private PIDController pid;
  private PIDController rotPID;
  double sensitivity = 0.5;

  // For setpoint
  private boolean Rsetpoint = false;
  private double rx = 0;
  private double ry = 0.5;

  // Drive heading offset — set when driver presses 7+8, does not affect odometry/vision
  private double driveHeadingOffset = 0;

  /** Creates a new DriveCommand. */
  public XBOXDriveCommand(DriveSubsystem driveSub, XboxController xbox, OdometrySubsystem odomSub) {
    this.driveSubsystem = driveSub;
    this.xbox = xbox;
    this.odomSub = odomSub;
    this.pid = driveSub.getPID();
    this.rotPID = new PIDController(0.2, 0, 0);
    addRequirements(driveSub);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveHeadingOffset = odomSub.getGyroAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  double oldT = 0.0;
  double oldG = 0.0;

  @Override
  public void execute() {
    buttonMicroCommands();
    double XAxis = xbox.getLeftX(), YAxis = xbox.getLeftY(), ZAxis = xbox.getRightX();
    double rawAngle = Math.atan2(YAxis, XAxis);
    double gyroAngle = odomSub.getGyroAngle() - driveHeadingOffset;
    if (xbox.getPOV() == 0) {
      sensitivity = MathUtil.clamp(0.05 + sensitivity, 0, 1);
    } else if (xbox.getPOV() == 180) {
      sensitivity = MathUtil.clamp(-0.05 + sensitivity, 0, 1);
    }
    double setDistance = MathUtil.clamp(Math.sqrt(XAxis * XAxis + YAxis * YAxis) * 2, 0, 2);
    double rotSpeed;

    if (!Rsetpoint) {
      rotSpeed = (MathUtil.applyDeadband(ZAxis, Constants.ControllerDeadzone)) * sensitivity
          * Constants.MaxRotationSpeed;
    } else {
      rotSpeed = rotateAroundPoint(rx, ry);
    }

    pid.setSetpoint(setDistance);
    double driveSpeed = pid.calculate((driveSubsystem.averageDistanceEncoder() - oldT) * 11.24)
        * Constants.MaxDriveSpeed * sensitivity;

    if (xbox.getPOV() != -1) {
      rotSpeed = povRotate();
    }

    if (xbox.getPOV() == -1 && Math.abs(XAxis) < Constants.ControllerDeadzone && Math.abs(YAxis) < Constants.ControllerDeadzone
        && Math.abs(ZAxis) < Constants.ControllerDeadzone) {
      driveSubsystem.stop();
    } else {
      if (Math.abs(ZAxis) > 0) {
        odomSub.getGyroAngle();
      }
      driveSubsystem.directionalDrive(driveSpeed, rawAngle - gyroAngle, rotSpeed);
    }
    oldT = driveSubsystem.averageDistanceEncoder();
    oldG = odomSub.getGyroAngle();
  }

  public void buttonMicroCommands() {
    if (buttonPressed(7) && buttonOnPress(8)) {
      driveHeadingOffset = odomSub.getGyroAngle();
      System.out.println("Gyro Reset Manually");
    }
    if (buttonPressed(1)) {
      Rsetpoint = (Rsetpoint == true) ? false : true;
    }
  }

public double povRotate() {
    int pov = xbox.getPOV();
    if (pov == -1) {
        return 0; // no POV pressed
    }

    // Convert gyro radians → degrees
    double currentDeg = Math.toDegrees(odomSub.getGyroAngle());
    currentDeg = MathUtil.inputModulus(currentDeg, 0, 360);

    double targetDeg = pov; // POV already gives 0, 90, 180, 270

    rotPID.enableContinuousInput(0, 360);

    double output = rotPID.calculate(currentDeg, targetDeg);

    // Clamp rotation so it doesn't go crazy
    return MathUtil.clamp(output, -0.6, 0.6);
}


  public boolean buttonPressed(int i) {
    return xbox.getRawButton(i);
  }

  public boolean buttonOnPress(int i) {
    return xbox.getRawButtonPressed(i);
  }

  public double rotateAroundPoint(double rx, double ry) {
    double dx = rx - odomSub.getX();
    double dy = ry - odomSub.getY();
    double angle = Math.atan2(dy, dx);
    rotPID.setSetpoint(angle);
    double error = MathUtil.angleModulus(angle - odomSub.getGyroAngle());
    return rotPID.calculate(error);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
