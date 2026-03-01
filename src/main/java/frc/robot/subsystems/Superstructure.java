// package frc.robot.subsystems;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.FieldConstants;
// import frc.robot.interpolation.InterpolatingDouble;
// import frc.robot.interpolation.LauncherTable;
// import frc.robot.subsystems.drive.Drive;
// import frc.robot.subsystems.launcher.flywheel.Flywheel;
// import frc.robot.subsystems.launcher.hood.Hood;
// import frc.robot.subsystems.launcher.turret.Turret;
// import org.littletonrobotics.junction.AutoLogOutput;
// import org.littletonrobotics.junction.Logger;

// public class Superstructure extends SubsystemBase {
//   private Drive drive;
//   private Flywheel flywheel;
//   private Hood hood;
//   private Turret turret;
//   private LauncherTable launcherTable;

//   private Timer timer = new Timer();

//   private Pose2d desiredHub = FieldConstants.Hub.redHubCenter;
//   private Pose2d desiredPassingLocation = FieldConstants.Outpost.redOutpostCenter;
//   public Pose2d turretPose;

//   public Superstructure(
//       Drive drive, Flywheel flywheel, Hood hood, Turret turret, LauncherTable launcherTable) {
//     this.drive = drive;
//     this.flywheel = flywheel;
//     this.hood = hood;
//     this.turret = turret;
//     launcherTable = new LauncherTable();

//     turretPose = drive.getPose().plus(turret.turretOffset);
//   }

//   private void interpolateHub() {
//     hood.setPositionHood(
//         launcherTable.hoodMap.getInterpolated(
//                 new InterpolatingDouble(FieldConstants.getDistanceToHubCenter(drive.getPose())))
//             .value);

//     flywheel.setVelocityLeader(
//         launcherTable.flywheelShootingMap.getInterpolated(
//                 new InterpolatingDouble(FieldConstants.getDistanceToHubCenter(drive.getPose())))
//             .value);
//   }

//   public void interpolatePassing() {
//     hood.setPositionHood(-1.90);

//     flywheel.setVelocityLeader(
//         launcherTable.flywheelPassingMap.getInterpolated(
//                 new InterpolatingDouble(FieldConstants.getDistanceToOutpost(drive.getPose())))
//             .value);
//   }

//   /** Returns the desired Turret pose. */
//   @AutoLogOutput(key = "Odometry/turretPose")
//   public Pose2d turretPose() {
//     Pose2d robotPose = drive.getPose();
//     double turretPoseX = robotPose.getX() + turret.turretOffset.getX();
//     double turretPoseY = robotPose.getY() + turret.turretOffset.getY();
//     return new Pose2d(turretPoseX, turretPoseY, calculateTurretRotation(desiredHub));
//   }

//   private void autoAimTurretHub() {
//     boolean isFlipped =
//         DriverStation.getAlliance().isPresent()
//             && DriverStation.getAlliance().get() == Alliance.Red;

//     if (isFlipped) {
//       desiredHub = FieldConstants.Hub.redHubCenter;
//     } else {
//       desiredHub = FieldConstants.Hub.blueHubCenter;
//     }

//     Rotation2d rot = calculateTurretRotation(desiredHub);
//     Rotation2d adjustedRot = new Rotation2d(Units.degreesToRadians(adjustTurretAngle(rot)));
//     Rotation2d robotRot = drive.getRotation();
//     Rotation2d turretRot = adjustedRot.plus(robotRot);

//     if (turretRot.getDegrees() < 0) {
//       turret.setPositionTurret(turretRot.getDegrees() + 360);
//       SmartDashboard.putNumber("autoAimTurretShooting", turretRot.getDegrees() + 360);
//     } else {
//       turret.setPositionTurret(turretRot.getDegrees());
//       SmartDashboard.putNumber("autoAimTurretShooting", turretRot.getDegrees());
//     }
//   }

//   private void autoAimTurretPassing() {
//     boolean isFlipped =
//         DriverStation.getAlliance().isPresent()
//             && DriverStation.getAlliance().get() == Alliance.Red;

//     if (isFlipped) {
//       desiredPassingLocation = FieldConstants.Outpost.redOutpostCenter;
//     } else {
//       desiredPassingLocation = FieldConstants.Outpost.blueOutpostCenter;
//     }

//     Rotation2d rot = calculateTurretRotation(desiredPassingLocation);
//     Rotation2d adjustedRot = new Rotation2d(Units.degreesToRadians(adjustTurretAngle(rot)));
//     Rotation2d robotRot = drive.getRotation();
//     Rotation2d turretRot = adjustedRot.plus(robotRot);

//     if (turretRot.getDegrees() < 0) {
//       turret.setPositionTurret(turretRot.getDegrees() + 360);
//       SmartDashboard.putNumber("autoAimTurretPassing", turretRot.getDegrees() + 360);
//     } else {
//       turret.setPositionTurret(turretRot.getDegrees());
//       SmartDashboard.putNumber("autoAimTurretPassing", turretRot.getDegrees());
//     }
//   }

//   public Command autoAimTurretHubCommand() {
//     return this.run(this::autoAimTurretHub);
//   }

//   public Command autoAimTurretPassingCommand() {
//     return this.run(this::autoAimTurretPassing);
//   }

//   public Command interpolateHubCommand() {
//     return this.run(this::interpolateHub);
//   }

//   public Command interpolatePassingCommand() {
//     return this.run(this::interpolatePassing);
//   }

//   @Override
//   public void periodic() {
//     Rotation2d rot = calculateTurretRotation(desiredHub);
//     Rotation2d adjustedRot = new Rotation2d(Units.degreesToRadians(adjustTurretAngle(rot)));
//     Logger.recordOutput("adjustedTurretAngleDegrees", adjustedRot.getDegrees());
//     SmartDashboard.putNumber("adjustedTurretAngleDegrees", adjustedRot.getDegrees());
//   }
// }
