package frc.robot.Commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Information.OdometrySubsystem;
import frc.robot.SwerveSubsystems.DriveSubsystem;

/**
 * Drives in a constant (dx, dy) field-relative direction at a fixed speed
 * until the robot has traveled the specified displacement, then stops.
 * No rotation is applied.
 */
public class DriveDistanceCommand extends Command {
    private final DriveSubsystem driveSub;
    private final OdometrySubsystem odomSub;
    private final double dx;
    private final double dy;
    private final double speed;

    private double startX;
    private double startY;
    private final double targetDistance;
    private final double driveAngle;

    /**
     * @param driveSub drive subsystem
     * @param odomSub  odometry subsystem
     * @param dx       field-relative X displacement in meters
     * @param dy       field-relative Y displacement in meters
     * @param speed    drive speed (0 to 1)
     */
    public DriveDistanceCommand(DriveSubsystem driveSub, OdometrySubsystem odomSub,
            double dx, double dy, double speed) {
        this.driveSub = driveSub;
        this.odomSub = odomSub;
        this.dx = dx;
        this.dy = dy;
        this.speed = speed;
        this.targetDistance = Math.sqrt(dx * dx + dy * dy);
        this.driveAngle = Math.atan2(dy, dx);
        addRequirements(driveSub);
    }

    @Override
    public void initialize() {
        Pose2d startPose = odomSub.getPose();
        startX = startPose.getX();
        startY = startPose.getY();
    }

    @Override
    public void execute() {
        // Adjust field-relative angle by robot heading so modules point the right way
        double robotRelativeAngle = driveAngle - odomSub.getGyroAngle();
        driveSub.directionalDrive(speed, robotRelativeAngle);
    }

    @Override
    public boolean isFinished() {
        Pose2d current = odomSub.getPose();
        double distX = current.getX() - startX;
        double distY = current.getY() - startY;
        double traveled = Math.sqrt(distX * distX + distY * distY);
        return traveled >= targetDistance;
    }

    @Override
    public void end(boolean interrupted) {
        driveSub.stop();
    }
}