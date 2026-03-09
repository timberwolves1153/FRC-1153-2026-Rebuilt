package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.turret.Turret;
import java.util.function.Supplier;

public class SuperstructureCommands {
  private Drive drive;
  private Turret turret;
  private Flywheel flywheel;
  private Hood hood;

  public SuperstructureCommands(Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
  }

  // public Supplier<ChassisSpeeds> robotRelativeSpeed() {
  //   ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();
  //   double driveSpeedX = chassisSpeeds.vxMetersPerSecond + turret.turretDisplacementX;
  //   double driveSpeedY = chassisSpeeds.vyMetersPerSecond + turret.turretDisplacementY;
  //   double driveSpeedAngular = chassisSpeeds.omegaRadiansPerSecond;
  //   ChassisSpeeds supplierRobotRelativeDriveSpeed =
  //       new ChassisSpeeds(driveSpeedX, driveSpeedY, driveSpeedAngular);
  //   Supplier<ChassisSpeeds> finalRobotRelativeDriveSpeed = () -> supplierRobotRelativeDriveSpeed;
  //   return finalRobotRelativeDriveSpeed;
  // }

  public Supplier<ChassisSpeeds> robotRelativeSpeed() {
    return () -> drive.getChassisSpeeds();
  }

  public Command autoAimTurretHub() {
    return Commands.parallel(
        turret.setTurretPositionHub(drive::getPose),
        hood.setPositionHoodHub(drive::getPose),
        flywheel.setVelocityHub(drive::getPose));
  }

  public Command interpolateShot() {
    return Commands.parallel(
        hood.setPositionHoodHub(drive::getPose), flywheel.setVelocityHub(drive::getPose));
  }

  public Command autoAimTurretPassing() {
    return Commands.parallel(
        turret.setTurretPositionPassing(drive::getPose),
        hood.setPositionHoodPassing(drive::getPose),
        flywheel.setVelocityPassing(drive::getPose));
  }

  // public Command shootOnTheMoveCommand() {
  //   return turret.shootOnTheMoveCommand(
  //       drive::getPose, drive::getChassisSpeeds, drive::getChassisSpeeds, hood, flywheel);
  // }
}
