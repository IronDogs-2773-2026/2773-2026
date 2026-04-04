// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.ShootSequenceCommand;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.ShooterSubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/**
 * Non-PathPlanner autonomous routine: backs up 56 inches from the hub, then shoots.
 */
public class ManualBackupAndShoot extends SequentialCommandGroup {

  private static final double BACKUP_DISTANCE_METERS = 56.0 * 0.0254; // ~1.4224m

  /**
   * Constructs the ManualBackupAndShoot command.
   */
  public ManualBackupAndShoot(DriveSubsystem driveSub, OdometrySubsystem odomSub, ShooterSubsystem shooterSub) {
    final Pose2d[] startPose = {new Pose2d()};

    addCommands(
      // Capture starting pose
      new RunCommand(() -> startPose[0] = odomSub.getPose(), driveSub).withTimeout(0.01),

      // Drive backward until 56 inches reached (use odometry pose delta)
      new RunCommand(() -> {
        driveSub.directionalDrive(0.3, odomSub.getGyroAngle(), 0.0);
      }, driveSub).finallyDo(() -> driveSub.stop()).until(() -> {
        Pose2d current = odomSub.getPose();
        double dist = Math.hypot(current.getX() - startPose[0].getX(), current.getY() - startPose[0].getY());
        return dist >= BACKUP_DISTANCE_METERS;
      }).withTimeout(5.0),

      // Shoot sequence (flywheel 0.7, feeder 0.6, 10s feed, 1.5s spinup)
      new ShootSequenceCommand(shooterSub, 0.65, -0.6, 10.0, 1.5)
    );
  }
}
