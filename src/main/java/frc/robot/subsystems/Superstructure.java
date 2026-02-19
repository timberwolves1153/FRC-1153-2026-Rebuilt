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
import org.littletonrobotics.junction.Logger;

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

  @AutoLogOutput(key = "Odometry/TurretRotation")
  public Rotation2d calculateTurretRotation(Pose2d goalPose) {
    // Calculate difference
    double turretPoseX = drive.getPose().getX() + turret.turretOffset.getX();
    double turretPoseY = drive.getPose().getY() + turret.turretOffset.getY();

    SmartDashboard.putNumber("Turret Pose X", turretPoseX);
    SmartDashboard.putNumber("Turret Pose Y", turretPoseY);
    Logger.recordOutput(
        "CalculatedTurretPose", new Pose2d(turretPoseX, turretPoseY, Rotation2d.kZero));
    Logger.recordOutput(
        "RotatedTurretPose", new Pose2d(turretPoseX, turretPoseY, Rotation2d.k180deg));

    double deltaY = goalPose.getY() - turretPoseY;
    double deltaX = goalPose.getX() - turretPoseX;

    SmartDashboard.putNumber("Turret Hub X Diff", deltaX);
    SmartDashboard.putNumber("Turret Hub Y Diff", deltaY);

    // Calculate angle in radians (using Math.Atan2 or similar)
    double angleRad = Math.atan2(deltaY, deltaX);
    SmartDashboard.putNumber("Angle Rad", angleRad);

    return Rotation2d.fromRadians(angleRad);
  }

  /** Returns the adjusted Turret Angle. */
  @AutoLogOutput(key = "Odometry/adjustedTurretRotation")
  public Rotation2d adjustTurretAngle(Rotation2d calculatedTurretAngle) {

    // Turret Map
    //     -90
    // -180     0
    //      90
    //
    // ROBOT INTAKE IS HERE

        //Formula: −calc + 90
    //Calculated		Encoder (actual)
    //90	      0	    0
    //135	     −45	−45 
    //179	     −89	−91
    //−179	   −91	−89 
    //−135	   −135	−135 
    //−90	     −180	−180

    Rotation2d adjustedTurretAngle =
        calculatedTurretAngle.plus(Rotation2d.kCW_90deg); // flip it to get 0 on the horizontal
    double adjustedTurretAngleDegrees = adjustedTurretAngle.getDegrees(); // get a degree value
    double encoderAngle = adjustedTurretAngleDegrees * -1; // We are getting values from -180 to 180 so we need to flip


    encoderAngle = encoderAngle % 360; // Stay on the unit circle (but degrees)
    if (encoderAngle >= 20) {
      return Rotation2d.fromDegrees(encoderAngle - 360);
    } else {
      return Rotation2d.fromDegrees(encoderAngle);
    }
  }

  /** Returns the desired Turret pose. */
  @AutoLogOutput(key = "Odometry/turretPose")
  public Pose2d turretPose() {
    Pose2d robotPose = drive.getPose();
    double turretPoseX = robotPose.getX() + turret.turretOffset.getX();
    double turretPoseY = robotPose.getY() + turret.turretOffset.getY();
    return new Pose2d(turretPoseX, turretPoseY, calculateTurretRotation(desiredHub));
  }

  /** Returns the desired Turret pose. */
  public void autoAimTurret() {
    // turret.setPositionTurret(calculateTurretAngle(desiredHub) - 0.238);
    // SmartDashboard.putNumber("Robot Pose Angle", drive.getRotation().getRotations());
  }

  @Override
  public void periodic() {
    Rotation2d rot = calculateTurretRotation(desiredHub);
    Rotation2d adjustedRot = adjustTurretAngle(rot);
    Logger.recordOutput("adjustedTurretAngleDegrees", adjustedRot.getDegrees());
    SmartDashboard.putNumber("adjustedTurretAngleDegrees", adjustedRot.getDegrees());
  }
}
