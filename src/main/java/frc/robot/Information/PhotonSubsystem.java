// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonSubsystem extends SubsystemBase {
  /** Creates a new PhotonSubsystem. */

  // create a photon camera object
  PhotonCamera tagCamera = new PhotonCamera("USB_Camera");

  // this be the subsystem
  public PhotonSubsystem() {
    // this.tagCamera = tagCamera;
  }

  // get pose as a Pose2d object, not a pose estimation: use all systems to get
  // pose
  public Pose2d getPosePhoton() {
    List<PhotonPipelineResult> result = tagCamera.getAllUnreadResults();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      Transform3d pose = target.getBestCameraToTarget();
      Translation3d translation = pose.getTranslation();
      Rotation3d rotation = pose.getRotation();
      return new Pose2d(translation.toTranslation2d(), rotation.toRotation2d());
    } else {
      return new Pose2d();
    }
  }

  private getBestTarget {
    
  }

  // get rotation as a Rotation2d
  // this method is unused, check for possible uses
  public Rotation2d getRot() {
    PhotonPipelineResult result = tagCamera.getLatestResult();
    if (result.hasTargets()) {
      PhotonTrackedTarget target = result.getBestTarget();
      Rotation3d pose = target.getBestCameraToTarget().getRotation();
      return pose.toRotation2d();
    } else {
      return new Rotation2d(0.0);
    }
  }

  public boolean hasTargets() {
    PhotonPipelineResult result = tagCamera.getLatestResult();
    return result.hasTargets();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}