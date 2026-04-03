// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.SwerveSubsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Information.TheMissileKnows;

/**
 * Alternative swerve drive subsystem using {@link TheMissileKnows} for odometry.
 * 
 * <p>Currently not used — {@link DriveSubsystem} is the active drive subsystem.
 * Named after the F1 driver Max Verstappen.
 */
public class MaxVerstappen extends SubsystemBase {
  private final TheMissileKnows odomSub;

  private final SwerveDriveModule fl, fr, bl, br;
  private final SwerveDriveModule[] modules;

  public MaxVerstappen(TheMissileKnows odomSub) {
    this.odomSub = odomSub;

    // initialize swerve modules
    fl = new SwerveDriveModule(Constants.frontLeftModuleDriveCANID, Constants.frontLeftModuleRotateCANID, Constants.frontLeftModuleEncoderCANID, 0.35522);
    fr = new SwerveDriveModule(Constants.frontRightModuleDriveCANID, Constants.frontRightModuleRotateCANID, Constants.frontRightModuleEncoderCANID, -0.48657);
    bl = new SwerveDriveModule(Constants.backLeftModuleDriveCANID, Constants.backLeftModuleRotateCANID, Constants.backLeftModuleEncoderCANID, 0.3686);
    br = new SwerveDriveModule(Constants.backRightModuleDriveCANID, Constants.backRightModuleRotateCANID, Constants.backRightModuleEncoderCANID, -0.1597);

    modules = new SwerveDriveModule[] {fl, fr, bl, br};
  }
  

  public void drive(ChassisSpeeds speeds) {
    // 1. Convert ChassisSpeeds to module states (velocity + angle)
    SwerveModuleState[] states = Constants.kinematics.toSwerveModuleStates(speeds);

    // 2. Ensure we don't try to drive faster than physical limits
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.MaxDriveSpeed);

    // 3. Command each module
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  @Override
  public void periodic() {
  }
}
