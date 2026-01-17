// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import org.photonvision.PhotonCamera;

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

  // get position for the robot as a double array
  public double[] getPose() {
    var result = tagCamera.getLatestResult();
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      var pose = target.getBestCameraToTarget().getTranslation();
      return new double[] { pose.getX(), pose.getY(), pose.getZ() };
    } else {
      return new double[] { 0.0, 0.0, 0.0 };
    }
  }

  // get rotation as a double
  public double getRot() {
    var result = tagCamera.getLatestResult();
    if (result.hasTargets()) {
      var target = result.getBestTarget();
      var pose = target.getBestCameraToTarget().getRotation();
      return pose.getZ();
    } else {
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}