// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
  /** Creates a new PhotonSubsystem. */

  // create a photon camera object
  PhotonCamera tagCamera = new PhotonCamera("tagCamera");

  // First, we create a new VisionSystemSim to represent our camera and
  // coprocessor running PhotonVision, and moving around our simulated field.
  
  // this be the subsystem
  public PhotonSubsystem() {
    // this.tagCamera = tagCamera;
  }

  // get pose as a Pose2d object, not a pose estimation: use all systems to get pose
  public Pose2d getPosePhoton() {
    var result = tagCamera.getLatestResult();
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      var pose = target.getBestCameraToTarget();
      var translation = pose.getTranslation();
      var rotation = pose.getRotation();
      return new Pose2d (translation.toTranslation2d(), rotation.toRotation2d());
    } else {
      return new Pose2d();
    }
  }

  

  // get rotation as a Rotation2d
  // this method is unused, check for possible uses
  public Rotation2d getRot() {
    var result = tagCamera.getLatestResult();
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      var pose = target.getBestCameraToTarget().getRotation();
      return pose.toRotation2d();
    } else {
      return new Rotation2d(0.0);
    }
  }

  public boolean hasTargets() {
    var result = tagCamera.getLatestResult();
    return result.hasTargets();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}