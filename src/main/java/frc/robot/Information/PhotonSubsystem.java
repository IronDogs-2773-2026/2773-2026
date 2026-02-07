// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import java.rmi.server.LogStream;
import java.util.List;
import java.util.NoSuchElementException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
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

  private Matrix<N3, N1> curStdDevs = Constants.SingleTagStdDevs;
  private final PhotonPoseEstimator m_photonEstimator;

  // create a photon camera object
  PhotonCamera tagCamera;
  public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.14, 0, 0),
      new Rotation3d(0, Math.PI / 6, 0));

  // may be worth moving to deprecated 4 part constructor if still an issue
  public PhotonSubsystem() {
    tagCamera = new PhotonCamera(Constants.CameraName);

    // this constructor here ^
    m_photonEstimator = new PhotonPoseEstimator(
        kTagLayout,
        kRobotToCam);
  }

  // Does not output, updates curStdDevs to account for vision innaccuracy.
  // may be stuck at zero causing infinite insignificance, printing output
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
          estStdDevs = VecBuilder.fill(Constants.BigNumber, Constants.BigNumber, Constants.BigNumber);
        else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        }
        var min = Constants.MinVisionStdDevs;

        estStdDevs = VecBuilder.fill(
            Math.max(estStdDevs.get(0, 0), min.get(0, 0)),
            Math.max(estStdDevs.get(1, 0), min.get(1, 0)),
            Math.max(estStdDevs.get(2, 0), min.get(2, 0)));

        curStdDevs = estStdDevs;
      }
    }
  }

  Optional<EstimatedRobotPose> poseEstimatorPose;
  private EstimatedRobotPose lastGoodPose = null;
  private double lastTimestamp = 0.0;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // // get all unread results may return empty, this may not translate in
    // function
    // for (PhotonPipelineResult res : tagCamera.getAllUnreadResults()) {
    // // function call returns an optional wrapped EstimatedRobotPose
    // poseEstimatorPose = m_photonEstimator.estimateLowestAmbiguityPose(res);

    // // should only ever use one photon pipeline result, may be worth printing if
    // broken
    // updateStdDevs(poseEstimatorPose, res.getTargets());
    List<PhotonPipelineResult> results = tagCamera.getAllUnreadResults();
    if (results.isEmpty()) {
      return;
    }
    PhotonPipelineResult res = results.get(results.size() - 1);
    poseEstimatorPose = m_photonEstimator.estimateLowestAmbiguityPose(res);

    System.out.println("Camera connected: " + tagCamera.isConnected());
    System.out.println("Targets: " + res.getTargets().size());
    if (poseEstimatorPose.isPresent()) {
      System.out.println("Vision pose: " +
          poseEstimatorPose.get().estimatedPose.toPose2d());
    }

    if (poseEstimatorPose.isPresent()) {
      lastGoodPose = poseEstimatorPose.get();
      lastTimestamp = lastGoodPose.timestampSeconds;
    }

    updateStdDevs(poseEstimatorPose, res.getTargets());

  }

  // returns standard deviations as a matrix, passed to addVisionMeasurements to
  // account for vision innaccuracies
  public Matrix<N3, N1> getStdDevs() {
    System.out.println(curStdDevs);
    return curStdDevs;
  }

  // returns pose estimate as a pose2d s
  public Pose2d getPose2d() {
    return lastGoodPose == null
        ? null
        : lastGoodPose.estimatedPose.toPose2d();
  }

  double latestTimestamp = 0;

  public double getPhotonTimestamp() {
    return lastTimestamp;
  }
}