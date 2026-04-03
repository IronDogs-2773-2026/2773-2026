// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Information;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/**
 * Alternative combined odometry + vision subsystem.
 * 
 * <p>Currently not used — {@link OdometrySubsystem} and {@link VisionSubsystem}
 * are the active systems. Merges NavX gyro, wheel odometry, and PhotonVision
 * AprilTag detection into a single pose estimator with dynamic standard deviations
 * based on tag ambiguity and distance.
 * 
 * <p>Named as a joke — "The Missile Knows" where the missile is the robot.
 */
public class TheMissileKnows extends SubsystemBase {
  private final DriveSubsystem drive;
  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final SwerveDriveKinematics kinematics;
  private final AHRS gyro;

  private double lastVisionTimestamp = 0D;

  public TheMissileKnows(DriveSubsystem drive) {
    this.drive = drive;
    this.camera = new PhotonCamera(Constants.CameraName);
    this.photonEstimator = new PhotonPoseEstimator(
        Constants.TagLayout,
        Constants.RobotToCam);
    this.kinematics = new SwerveDriveKinematics(
        Constants.kfrontLeftLocation,
        Constants.kfrontRightLocation,
        Constants.kbackLeftLocation,
        Constants.kbackRightLocation);
    this.gyro = new AHRS(NavXComType.kMXP_SPI);

    this.poseEstimator = new SwerveDrivePoseEstimator(
        kinematics,
        gyro.getRotation2d(),
        drive.getPositions(),
        new Pose2d());
  }

  public boolean isWithinTolerance(double current, double target, double tolerance) {
      return Math.abs(current - target) <= tolerance;
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation2dUnbound() {
    return gyro.getRotation2d();
  }

  public Rotation2d getRotation2dBounded() {
    return Rotation2d.fromDegrees(MathUtil.inputModulus(gyro.getAngle(), -180, 180));
  }

  public double getRadians() {
    return getRotation2dUnbound().getRadians();
  }

  public double getDegrees() {
    return getRotation2dUnbound().getDegrees();
  }

  private Matrix<N3, N1> calculateStdDevs(EstimatedRobotPose estimate) {
    // start with confident estimate
    Matrix<N3, N1> stdDevs = Constants.SingleTagStdDevs;
    if (estimate.targetsUsed.isEmpty())
      return stdDevs.times(Constants.BigNumber);

    // get first target
    var target = estimate.targetsUsed.get(0);

    // calculate 3d distance to tag
    double distance = target.getBestCameraToTarget().getTranslation().getNorm();
    if (target.getPoseAmbiguity() > Constants.AmbiguityTrustWall)
      stdDevs = stdDevs.times(Constants.MediumNumber);
    stdDevs = stdDevs.times(Math.pow(MathUtil.clamp(distance, 0.1, 10), 2) * Constants.TrustCoefficient);
    return stdDevs;
  }

  private void masterUpdate() {
    // update chassis odometry first
    poseEstimator.update(gyro.getRotation2d(), drive.getPositions());

    // process vision
    var results = camera.getAllUnreadResults();
    for (var result : results) {
      var optEstimate = photonEstimator.estimateLowestAmbiguityPose(result);
      if (optEstimate.isPresent()) {
        var est = optEstimate.get();
        double ambiguity = est.targetsUsed.get(0).getPoseAmbiguity();

        // filter results
        if (ambiguity < 0.2 && est.timestampSeconds > lastVisionTimestamp) {
          lastVisionTimestamp = est.timestampSeconds;
          poseEstimator.addVisionMeasurement(
              est.estimatedPose.toPose2d(),
              est.timestampSeconds,
              calculateStdDevs(est));
        }
      }
    }
  }

  @Override
  public void periodic() {
    masterUpdate();
  }
}
// https://youtu.be/bZe5J8SVCYQ?si=XoSJxxx8E0qfOTof
// remix song version https://www.youtube.com/watch?v=_LjN3UclYzU