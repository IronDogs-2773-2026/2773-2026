package frc.robot.Information;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveSubsystems.DriveSubsystem;
import frc.robot.SwerveSubsystems.SwerveDriveModule;

/**
 * Odometry subsystem tracking robot pose using swerve drive kinematics and NavX gyro.
 * 
 * <p>Uses a {@link SwerveDrivePoseEstimator} to fuse wheel encoder data with
 * gyro heading for dead-reckoning position tracking. Vision measurements from
 * AprilTag detection can be added via {@link #addVisionMeasurement} to correct drift.
 * 
 * <p>The NavX gyro is <b>not</b> reset at construction time — it calibrates once
 * on power-up (~1-2 seconds). By match time, the gyro reading is stable and
 * trustworthy. All field-relative driving uses delta-based offsets to avoid
 * dependency on absolute zero.
 */
public class OdometrySubsystem extends SubsystemBase {

    DriveSubsystem driveSub;

    // Locations for the swerve drive modules relative to the robot center. Meters?
    Translation2d m_frontLeftLocation = new Translation2d(0.283, 0.281);
    Translation2d m_frontRightLocation = new Translation2d(0.283, -0.281);
    Translation2d m_backLeftLocation = new Translation2d(-0.283, 0.281);
    Translation2d m_backRightLocation = new Translation2d(-0.283, -0.281);

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);
    AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
    SwerveDrivePoseEstimator m_poseEstimator;
    SwerveDriveModule[] modules;
    Pose2d pose = new Pose2d();
    Field2d field = new Field2d();

    /**
     * Constructs the OdometrySubsystem and initializes the pose estimator.
     * 
     * @param driveSub The drive subsystem to read module positions from
     */
    public OdometrySubsystem(DriveSubsystem driveSub) {
        // TODO: removing gyro reset flagged here,
        // gyro.reset();
        this.driveSub = driveSub;
        modules = driveSub.modules;
        m_poseEstimator = new SwerveDrivePoseEstimator(
                m_kinematics,
                gyro.getRotation2d().times(-1),
                driveSub.getPositions(),
                new Pose2d());

        Shuffleboard.getTab("Odometry").addDouble("Robot X", () -> {
            return getX();
        });
        Shuffleboard.getTab("Odometry").addDouble("Robot Y", () -> {
            return getY();
        });

        SmartDashboard.putNumber("X", getX());
        SmartDashboard.putNumber("Y", getY());
    }

    @Override
    public void periodic() {
        Rotation2d gyroAngle = new Rotation2d(gyro.getAngle() * Math.PI / 180);

        m_poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                gyroAngle.times(-1),
                driveSub.getPositions());

        pose = m_poseEstimator.getEstimatedPosition();

        field.setRobotPose(pose);
        SmartDashboard.putNumber("X", pose.getX());
        SmartDashboard.putNumber("Y", pose.getY());
        SmartDashboard.putData("Field", field);
    }

    /*
     * looks weird, don't touch -- if I git diff it and it isn't the
     * same we are going to upload a picture of your dog to the internet
     */
    /**
     * Returns the current gyro angle in radians, normalized to [-π, π].
     * 
     * @return Gyro angle in radians, counter-clockwise positive
     */
    public double getGyroAngle() {
        double angle = (gyro.getAngle() - 0) / 180.0 * Math.PI;
        while (angle > Math.PI) {
            angle -= Math.PI * 2;
        }
        while (angle < -Math.PI) {
            angle += Math.PI * 2;
        }
        return angle;
    }

    /**
     * Returns the current estimated robot pose.
     * 
     * @return Pose2d with x, y (meters) and rotation (radians)
     */
    public Pose2d getPose() {
        return pose;
    }

    /**
     * Resets the robot's estimated pose to a new position on the field.
     * 
     * @param newPose The new pose to set (x, y in meters, rotation in radians)
     */
    public void resetPose(Pose2d newPose) {
        pose = newPose;
        m_poseEstimator.resetPosition(
                gyro.getRotation2d().times(-1), // current gyro heading
                driveSub.getPositions(), // current module positions
                newPose // new pose
        );
    }

    /**
     * Returns the estimated X position of the robot.
     * 
     * @return X position in meters
     */
    public double getX() {
        return pose.getX();
    }

    /**
     * Returns the estimated Y position of the robot (inverted for field coordinates).
     * 
     * @return Y position in meters (inverted from pose estimator)
     */
    public double getY() {
        return -pose.getY();
    }

    /**
     * Sets the robot's estimated pose to specific coordinates.
     * 
     * @param x X position in meters
     * @param y Y position in meters
     * @param rotation Rotation in radians
     */
    public void setPose(double x, double y, double rotation) {
        Pose2d newPose = new Pose2d(x, y, new Rotation2d(rotation));
        pose = newPose;

        m_poseEstimator.resetPosition(
                gyro.getRotation2d().times(-1),
                driveSub.getPositions(),
                newPose);
    }

    /**
     * Adds a vision-based pose measurement to the pose estimator.
     * Used to correct odometry drift from AprilTag detection.
     * 
     * @param pose The vision-estimated robot pose
     * @param timestamp FPGA timestamp of the measurement
     * @param stdDevs Standard deviations of the estimate (3x1: x, y, rotation)
     */
    public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs) {
        m_poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
    }

    /**
     * Resets the NavX gyro to zero and re-syncs the pose estimator.
     * 
     * <p><b>Warning:</b> This should only be called at boot or during disabled periods.
     * Calling this mid-match will desync the pose estimator from the gyro's
     * accumulated angle.
     */
    public void resetGyro() {
        gyro.reset();
        m_poseEstimator.resetPosition(
            gyro.getRotation2d().times(-1),
            driveSub.getPositions(),
            pose
        );
    }

    /**
     * Returns the current steering angles of all four swerve modules.
     * 
     * @return Array of 4 steer angles in radians [FL, FR, BL, BR]
     */
    public double[] getSwerveAngles() {
        double[] pos = new double[4];
        for (int i = 0; i < 4; i++) {
            pos[i] = modules[i].steerAngle();
        }
        return pos;
    }
}
