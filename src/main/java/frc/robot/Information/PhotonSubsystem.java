// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Information.VisionSubsystem.EstimateConsumer;

public class PhotonSubsystem extends SubsystemBase {
  /** Creates a new PhotonSubsystem. */

  private Matrix<N3, N1> curStdDevs;
  private final PhotonPoseEstimator m_photonEstimator;

  // create a photon camera object
  PhotonCamera tagCamera;
  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.14, 0, 0),
      new Rotation3d(0, Math.PI / 6, 0));

  // this be the subsystem
  public PhotonSubsystem() {
    tagCamera = new PhotonCamera(Constants.CameraName);
    m_photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);
  }

  // Does not output, updates curStdDevs to account for vision innaccuracy.
  private void updateStdDevs(Optional<EstimatedRobotPose> estimates, List<PhotonTrackedTarget> targets) {
    if (estimates.isEmpty()) {
      curStdDevs = Constants.SingleTagStdDevs;
    } else {
      Matrix<N3, N1> estStdDevs = Constants.SingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      for (PhotonTrackedTarget target : targets) {
        Optional<Pose3d> tagPose = m_photonEstimator.getFieldTags().getTagPose(target.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose.get().toPose2d().getTranslation()
            .getDistance(estimates.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        curStdDevs = Constants.SingleTagStdDevs;
      } else {
        avgDist /= numTags;
        if (numTags > 1)
          estStdDevs = Constants.SingleTagStdDevs;
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        curStdDevs = estStdDevs;
      }
    }
  }

  Optional<EstimatedRobotPose> estimates;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    estimates = Optional.empty();
    for (PhotonPipelineResult res : tagCamera.getAllUnreadResults()) {
      estimates = m_photonEstimator.estimateLowestAmbiguityPose(res);
      if (estimates.isEmpty()) {
        estimates = m_photonEstimator.estimateLowestAmbiguityPose(res);
      }
      updateStdDevs(estimates, res.getTargets());
    }
  }

  // returns standard deviations as a matrix, passed to addVisionMeasurements to
  // account for vision innaccuracies
  public Matrix<N3, N1> getStdDevs() {
    return curStdDevs;
  }

  // returns pose estimate as a pose2d s
  public Pose2d getPose2d() {
    return estimates.get().estimatedPose.toPose2d();
  }
}