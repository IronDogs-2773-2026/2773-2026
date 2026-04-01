package frc.robot;

import java.lang.reflect.Member;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.*;

@SuppressWarnings({"ConstantName", "MemberName"})
public class Constants {
  public static final int kDriverControllerPort = 0;
  // Measurements of robot
  public static final double DistanceBetweenWheels = 0.616;
  public static final double WheelCircumference = Math.PI * 0.1016;
  public static final double powerVelocityRatio = 1.0; // Represents (motorRelativePower/actualVelocity) aka a
                                                       // conversion
  public static final double powerTwistRatio = 1.0; // Represents (motorRelativePower/rotationVelocity) aka a conversion

  // MotorType of our Neos & PWMs
  public static final MotorType motorType = MotorType.kBrushless;

  // Shooter Motors
  public static final int flyWheel1 = 13;
  public static final int flyWheel2 = 24;
  public static final int feederMotor = 25;
  public static final int armMotor = 49;
  public static final int intakeMotor = 29;

  // CAN IDs for Swerve Modules
  // Drive Motors
  public static final int frontLeftModuleDriveCANID = 17;
  public static final int frontRightModuleDriveCANID = 19;
  public static final int backLeftModuleDriveCANID = 10;
  public static final int backRightModuleDriveCANID = 22;
  // Rotation Motors
  public static final int frontLeftModuleRotateCANID = 16;
  public static final int frontRightModuleRotateCANID = 12;
  public static final int backLeftModuleRotateCANID = 11;
  public static final int backRightModuleRotateCANID = 23;
  // CTRE CANCoders (Encoders)
  public static final int frontLeftModuleEncoderCANID = 52;
  public static final int frontRightModuleEncoderCANID = 53;
  public static final int backLeftModuleEncoderCANID = 54;
  public static final int backRightModuleEncoderCANID = 55;

  public static final double MaxDriveSpeed = 0.6;
  public static final double MaxRotationSpeed = 0.50;
  public static final double MaxTowerSpeed = 0.25;
  public static final double MaxAutoDriveSpeed = 0.3;
  public static final double SlowdownFactor = 0.7;

  public static final double ControllerDeadzone = 0.07;
  public static final double HOTASDeadzone = 0.1;
  public static final double HOTASRotationDeadzone = 0.3;
  public static final Translation2d kfrontLeftLocation = new Translation2d(0.283, 0.281);
  public static final Translation2d kfrontRightLocation = new Translation2d(0.283, -0.281);
  public static final Translation2d kbackLeftLocation = new Translation2d(-0.283, 0.281);
  public static final Translation2d kbackRightLocation = new Translation2d(-0.283, -0.281);
  public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      kfrontLeftLocation,
      kfrontRightLocation,
      kbackLeftLocation,
      kbackRightLocation);

  // Vision
  public static final String CameraName = "AprilCam";
  public static final Transform3d RobotToCam = new Transform3d(new Translation3d(0.01, 0.2, 1),
      new Rotation3d(0, Math.PI/6, Math.PI));
  public static final AprilTagFieldLayout TagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
  public static final Matrix<N3, N1> SingleTagStdDevs = VecBuilder.fill(0.1, 0.1, Math.toRadians(10));
  public static final Matrix<N3, N1> MultiTagStdDevs = VecBuilder.fill(0.05, 0.05, Math.toRadians(5));
  public static final double BigNumber = 9999;
  public static final double MediumNumber = 9;
  public static final Vector<N3> MinVisionStdDevs = VecBuilder.fill(0.25, 0.25, Math.toRadians(10));
  public static final double TimeAllowance = 3.0;
  public static final double TrustCoefficient = 0.3;
  public static final double AmbiguityTrustWall = 0.3;
}