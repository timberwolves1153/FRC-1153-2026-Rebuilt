package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.FieldConstants;
import frc.robot.interpolation.InterpolatingDouble;
import frc.robot.interpolation.LauncherTable;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.turret.Turret;

import java.lang.reflect.Field;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SuperstructureCommands {
  private Drive drive;
  private Turret turret;
  private Flywheel flywheel;
  private Hood hood;

  // Shoot on the move stuff
  private double phaseDelay = 0.03;
  public Rotation2d finalTurretAngle;
  public Pose2d turretPose;
  public double turretDisplacementX;
  public double turretDisplacementY;
  public double turretFinalAngularVelocity;
  public Pose2d lookAheadPose;
  public double setMovingTurretAngle;

  public SuperstructureCommands(Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
  }

  public Supplier<ChassisSpeeds> robotRelativeSpeed() {
    ChassisSpeeds chassisSpeeds = drive.getChassisSpeeds();
    double driveSpeedX = chassisSpeeds.vxMetersPerSecond + turret.turretDisplacementX;
    double driveSpeedY = chassisSpeeds.vyMetersPerSecond + turret.turretDisplacementY;
    double driveSpeedAngular = chassisSpeeds.omegaRadiansPerSecond;
    ChassisSpeeds supplierRobotRelativeDriveSpeed =
        new ChassisSpeeds(driveSpeedX, driveSpeedY, driveSpeedAngular);
    Supplier<ChassisSpeeds> finalRobotRelativeDriveSpeed = () -> supplierRobotRelativeDriveSpeed;
    return finalRobotRelativeDriveSpeed;
  }

  public Pose2d shootOnTheMove(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotRelVelocity,
      Supplier<ChassisSpeeds> robotFieldVelocity) {

    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d desiredHub;
    if (isRed) {
      desiredHub = FieldConstants.Hub.redHubCenter;
    } else {
      desiredHub = FieldConstants.Hub.blueHubCenter;
    }

    ChassisSpeeds robotRelativeVelocity = robotRelVelocity.get();
    Translation2d target = desiredHub.getTranslation();

    Pose2d estimatedPose =
        robotPose
            .get()
            .exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Translation2d rotatedOffset =
        turret.turretOffset.getTranslation().rotateBy(estimatedPose.getRotation());

    // new Transform2d(
    //     turret.turretOffset.getX(), turret.turretOffset.getY(), estimatedPose.getRotation());

    // turretPose = estimatedPose.transformBy(rotatedOffset);

    turretPose = estimatedPose.transformBy(turret.turretOffset);

    // estimatedPose.transformBy(rotatedOffset);  //estimatedPose.transformBy(turret.turretOffset);
    // new Pose2d(
    //     estimatedPose.getX() + rotatedOffset.getX(),
    //     estimatedPose.getY() + rotatedOffset.getY(),
    //     adjustedTurretRotation(robotPose, desiredHub));

    // Translation2d turretFieldOffset =
    //     turretPose.getTranslation().minus(estimatedPose.getTranslation());

    double turretToHubDistance = FieldConstants.getDistanceToHubCenter(turretPose);

    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation());

    double robotAngle = estimatedPose.getRotation().getRadians();

    double
        turretVelocityX = // subtract x from the y to tke into the account the robot's rotation when
            // shooting and moving at the same time
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (turret.turretOffset.getY() * Math.cos(robotAngle)
                        - turret.turretOffset.getX() * Math.sin(robotAngle));

    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turret.turretOffset.getX() * Math.cos(robotAngle)
                    - turret.turretOffset.getY() * Math.sin(robotAngle));

    double fuelTimeofFlight;
    lookAheadPose = turretPose;
    for (int i = 0; i < 20; i++) {

      fuelTimeofFlight =
          LauncherTable.flightTimeMap.getInterpolated(new InterpolatingDouble(turretToHubDistance))
              .value;

      turretDisplacementX = turretVelocityX * fuelTimeofFlight;
      turretDisplacementY = turretVelocityY * fuelTimeofFlight;

      lookAheadPose =
          new Pose2d(
              turretPose
                  .getTranslation()
                  .plus(new Translation2d(turretDisplacementX, turretDisplacementY)),
              turretPose.getRotation());

      // SmartDashboard.putNumber("Turret Pose X", turretPose.getX());
      // SmartDashboard.putNumber("Turret Pose Y", turretPose.getY());
      turretToHubDistance = target.getDistance(lookAheadPose.getTranslation());
    }
    // Calculate final turret angle to hub using atan2 for correct quadrant handling
    Rotation2d fieldAngleToHub = target.minus(lookAheadPose.getTranslation()).getAngle();
    // double turretVelocity =
    //     turretFilter.calculate(fieldAngleToHub.minus(fieldAngleToHub).getRadians() / 0.02);
    //                                       loopPeriodSecs ^^^^

    // Convert field angle to motor encoder coordinates using the same transformation as
    // autoAimTurret

    // Set turret position using the same logic as autoAimTurret
    setMovingTurretAngle =
        turret.adjustedTurretRotation(() -> lookAheadPose, desiredHub).getDegrees();

    if (setMovingTurretAngle < 0) {
      setMovingTurretAngle = setMovingTurretAngle + 360;
      SmartDashboard.putNumber("SOTM TurretAngle", setMovingTurretAngle);
    } else {
      SmartDashboard.putNumber("SOTM TurretAngle", setMovingTurretAngle);
    }

    SmartDashboard.putNumber("SOTM Moving Turret Angle", setMovingTurretAngle);
    Logger.recordOutput("SOTM Turret Pose", lookAheadPose);
    Logger.recordOutput("SOTM Target", new Pose2d(target, Rotation2d.kZero));

    return lookAheadPose;
  }

    public Pose2d passOnTheMove(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotRelVelocity,
      Supplier<ChassisSpeeds> robotFieldVelocity) {

    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d desiredPass;

    if (isRed && robotPose.get().getY() > (FieldConstants.fieldWidth / 2)) {
      desiredPass = FieldConstants.Outpost.redOutpostCenter;
    } else if (isRed) {
      desiredPass = FieldConstants.Depot.redDepotCenter;
    } else if (!isRed && robotPose.get().getY() > (FieldConstants.fieldWidth / 2)) {
      desiredPass = FieldConstants.Depot.blueDepotCenter;
    } else {
      desiredPass = FieldConstants.Outpost.blueOutpostCenter;
    }

    ChassisSpeeds robotRelativeVelocity = robotRelVelocity.get();
    Translation2d target = desiredPass.getTranslation();

    Pose2d estimatedPose =
        robotPose
            .get()
            .exp(
                new Twist2d(
                    robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                    robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    Translation2d rotatedOffset =
        turret.turretOffset.getTranslation().rotateBy(estimatedPose.getRotation());

    // new Transform2d(
    //     turret.turretOffset.getX(), turret.turretOffset.getY(), estimatedPose.getRotation());

    // turretPose = estimatedPose.transformBy(rotatedOffset);

    turretPose = estimatedPose.transformBy(turret.turretOffset);

    // estimatedPose.transformBy(rotatedOffset);  //estimatedPose.transformBy(turret.turretOffset);
    // new Pose2d(
    //     estimatedPose.getX() + rotatedOffset.getX(),
    //     estimatedPose.getY() + rotatedOffset.getY(),
    //     adjustedTurretRotation(robotPose, desiredHub));

    // Translation2d turretFieldOffset =
    //     turretPose.getTranslation().minus(estimatedPose.getTranslation());

    double turretToPassDistance;

    if (isRed && robotPose.get().getY() > (FieldConstants.fieldWidth / 2)) {
      turretToPassDistance = FieldConstants.getDistanceToOutpost(turretPose);
    } else if (isRed) {
     turretToPassDistance = FieldConstants.getDistanceToDepot(turretPose);
    } else if (!isRed && robotPose.get().getY() > (FieldConstants.fieldWidth / 2)) {
      turretToPassDistance = FieldConstants.getDistanceToDepot(turretPose);
    } else {
      turretToPassDistance = FieldConstants.getDistanceToOutpost(turretPose);
    }

    ChassisSpeeds robotVelocity =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeVelocity, estimatedPose.getRotation());

    double robotAngle = estimatedPose.getRotation().getRadians();

    double
        turretVelocityX = // subtract x from the y to tke into the account the robot's rotation when
            // shooting and moving at the same time
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (turret.turretOffset.getY() * Math.cos(robotAngle)
                        - turret.turretOffset.getX() * Math.sin(robotAngle));

    double turretVelocityY =
        robotVelocity.vyMetersPerSecond
            + robotVelocity.omegaRadiansPerSecond
                * (turret.turretOffset.getX() * Math.cos(robotAngle)
                    - turret.turretOffset.getY() * Math.sin(robotAngle));

    double fuelTimeofFlight;
    lookAheadPose = turretPose;
    for (int i = 0; i < 20; i++) {

      fuelTimeofFlight =
          LauncherTable.flightTimeMap.getInterpolated(new InterpolatingDouble(turretToPassDistance))
              .value;

      turretDisplacementX = turretVelocityX * fuelTimeofFlight;
      turretDisplacementY = turretVelocityY * fuelTimeofFlight;

      lookAheadPose =
          new Pose2d(
              turretPose
                  .getTranslation()
                  .plus(new Translation2d(turretDisplacementX, turretDisplacementY)),
              turretPose.getRotation());

      // SmartDashboard.putNumber("Turret Pose X", turretPose.getX());
      // SmartDashboard.putNumber("Turret Pose Y", turretPose.getY());
      turretToPassDistance = target.getDistance(lookAheadPose.getTranslation());
    }
    // Calculate final turret angle to hub using atan2 for correct quadrant handling
    Rotation2d fieldAngleToHub = target.minus(lookAheadPose.getTranslation()).getAngle();
    // double turretVelocity =
    //     turretFilter.calculate(fieldAngleToHub.minus(fieldAngleToHub).getRadians() / 0.02);
    //                                       loopPeriodSecs ^^^^

    // Convert field angle to motor encoder coordinates using the same transformation as
    // autoAimTurret

    // Set turret position using the same logic as autoAimTurret
    setMovingTurretAngle =
        turret.adjustedTurretRotation(() -> lookAheadPose, desiredPass).getDegrees();

    if (setMovingTurretAngle < 0) {
      setMovingTurretAngle = setMovingTurretAngle + 360;
      SmartDashboard.putNumber("SOTM TurretAngle", setMovingTurretAngle);
    } else {
      SmartDashboard.putNumber("SOTM TurretAngle", setMovingTurretAngle);
    }

    SmartDashboard.putNumber("SOTM Moving Turret Angle", setMovingTurretAngle);
    Logger.recordOutput("SOTM Turret Pose", lookAheadPose);
    Logger.recordOutput("SOTM Target", new Pose2d(target, Rotation2d.kZero));

    return lookAheadPose;
  }

  public Command autoAimTurretHub() {
    return Commands.parallel(
        turret.setTurretPositionHub(drive::getPose),
        hood.setPositionHoodHub(drive::getPose),
        flywheel.setVelocityHub(drive::getPose));
  }

  public Command autoAimTurretPassing() {
    return Commands.parallel(
        turret.setTurretPositionPassing(drive::getPose),
        hood.setPositionHoodPassing(drive::getPose),
        flywheel.setVelocityPassing(drive::getPose));
  }

  public Command passOnTheMoveCommand() {
    return Commands.parallel(
        turret.setTurretPositionPassing(
            () -> passOnTheMove(drive::getPose, robotRelativeSpeed(), drive::getChassisSpeeds)),
        hood.setPositionHoodPassing(
            () -> passOnTheMove(drive::getPose, robotRelativeSpeed(), drive::getChassisSpeeds)),
        flywheel.setVelocityPassing(
            () -> passOnTheMove(drive::getPose, robotRelativeSpeed(), drive::getChassisSpeeds)));
  }

  public Command shootOnTheMoveCommand() {
    return Commands.parallel(
        turret.setTurretPositionHub(
            () -> shootOnTheMove(drive::getPose, robotRelativeSpeed(), drive::getChassisSpeeds)),
        hood.setPositionHoodHub(
            () -> shootOnTheMove(drive::getPose, robotRelativeSpeed(), drive::getChassisSpeeds)),
        flywheel.setVelocityHub(
            () -> shootOnTheMove(drive::getPose, robotRelativeSpeed(), drive::getChassisSpeeds)));
  }
}
