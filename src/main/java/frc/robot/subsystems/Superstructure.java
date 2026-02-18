package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.interpolation.InterpolatingDouble;
import frc.robot.interpolation.LauncherTable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.turret.Turret;
import org.littletonrobotics.junction.AutoLogOutput;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Flywheel flywheel;
  private Hood hood;
  private Turret turret;
  private LauncherTable launcherTable;

  private Timer timer = new Timer();

  private Pose2d desiredHub = FieldConstants.Hub.redHubCenter;
  public Pose2d turretPose;

  public Superstructure(
      Drive drive, Flywheel flywheel, Hood hood, Turret turret, LauncherTable launcherTable) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    launcherTable = new LauncherTable();

    turretPose = drive.getPose().plus(turret.turretOffset);
  }

  public void interpolateShot() {
    hood.setPositionHood(
        launcherTable.hoodMap.getInterpolated(
                new InterpolatingDouble(FieldConstants.getDistanceToHubCenter(drive.getPose())))
            .value);

    flywheel.setVelocityLeader(
        launcherTable.flywheelMap.getInterpolated(
                new InterpolatingDouble(FieldConstants.getDistanceToHubCenter(drive.getPose())))
            .value);
  }

  @AutoLogOutput(key = "Odometry/TurretAngle")
  public double calculateTurretAngle(Pose2d robotPose, Pose2d goalPose) {
    // Calculate difference

    double turretPoseX = drive.getPose().getX() + turret.turretOffset.getX();
    double turretPoseY = drive.getPose().getY() + turret.turretOffset.getY();

    SmartDashboard.putNumber("Turret Pose X", turretPoseX);
    SmartDashboard.putNumber("Turret Pose Y", turretPoseY);

    double deltaY = goalPose.getY() - turretPoseY;
    double deltaX = goalPose.getX() - turretPoseX;

    SmartDashboard.putNumber("Turret Hub X Diff", deltaX);
    SmartDashboard.putNumber("Turret Hub Y Diff", deltaY);

    // Calculate angle in radians (using Math.Atan2 or similar)
    double angleRad = Math.atan(deltaY / deltaX);
    Rotation2d fieldRelativeAngle = Rotation2d.fromRadians(angleRad);
    Rotation2d robotRelativeAngle = fieldRelativeAngle.minus(drive.getPose().getRotation());

    SmartDashboard.putNumber("Field Relative Angle", fieldRelativeAngle.getDegrees());
    SmartDashboard.putNumber("Robot Relative Angle", robotRelativeAngle.getDegrees());
    SmartDashboard.putNumber("Angle Rad", angleRad);

    return -robotRelativeAngle.getRotations();
  }

  /** Returns the desired Turret pose. */
  public void autoAimTurret() {
    turret.setPositionTurret(calculateTurretAngle(drive.getPose(), desiredHub) - 0.238); // .218
    SmartDashboard.putNumber("Robot Pose Angle", drive.getRotation().getRotations());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber(
        "Turret Auto Aim Target", calculateTurretAngle(drive.getPose(), desiredHub) - 0.238);
  }
}
