package frc.robot.subsystems.launcher.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.interpolation.InterpolatingDouble;
import frc.robot.interpolation.LauncherTable;

import java.lang.reflect.Field;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public final Transform2d turretOffset;

  private double phaseDelay = 0.03;
  public Rotation2d finalTurretAngle;
  public Pose2d turretPose;
  public double turretDisplacementX;
  public double turretDisplacementY;
  public double turretFinalAngularVelocity;

  public Turret(TurretIO turretIO) {
    io = turretIO;

    turretOffset =
        new Transform2d(
            Units.inchesToMeters(-4.75), Units.inchesToMeters(-4.125), new Rotation2d());

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        break;

      case SIM:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void setPositionTurret(double degrees) {
    degrees = MathUtil.clamp(degrees, 45, 315);
    io.setPositionTurret(Units.degreesToRotations(degrees));
  }

  public void setVoltageTurret(double volts) {
    io.setVoltageTurret(volts);
  }

  public void stopTurret() {
    io.stopTurret();
  }

  private Rotation2d adjustedTurretRotation(Supplier<Pose2d> robotPoseSupplier, Pose2d desiredHub) {
    Rotation2d rot = calculateTurretRotation(robotPoseSupplier.get(), desiredHub);
    Rotation2d adjustedRot = new Rotation2d(Units.degreesToRadians(adjustTurretAngle(rot)));
    Rotation2d robotRot = robotPoseSupplier.get().getRotation();
    Rotation2d turretRot = adjustedRot.plus(robotRot);
    if (turretRot.getDegrees() < 0) {
      setPositionTurret(turretRot.getDegrees() + 360);
    } else {
      setPositionTurret(turretRot.getDegrees());
    }
    return turretRot;
  }

  private void autoAimTurretHub(Supplier<Pose2d> robotPoseSupplier) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d desiredHub;
    if (isRed) {
      desiredHub = FieldConstants.Hub.redHubCenter;
    } else {
      desiredHub = FieldConstants.Hub.blueHubCenter;
    }

    Rotation2d rot = calculateTurretRotation(robotPoseSupplier.get(), desiredHub);
    Rotation2d adjustedRot = new Rotation2d(Units.degreesToRadians(adjustTurretAngle(rot)));
    Rotation2d robotRot = robotPoseSupplier.get().getRotation();
    Rotation2d turretRot = adjustedRot.plus(robotRot);

    if (turretRot.getDegrees() < 0) {
      setPositionTurret(turretRot.getDegrees() + 360);
      SmartDashboard.putNumber("autoAimTurretShooting", turretRot.getDegrees() + 360);
    } else {
      setPositionTurret(turretRot.getDegrees());
      SmartDashboard.putNumber("autoAimTurretShooting", turretRot.getDegrees());
    }
  }

  private void autoAimTurretPassing(Supplier<Pose2d> robotPoseSupplier) {
    boolean isRed =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    Pose2d desiredPassingLocation = FieldConstants.Outpost.redOutpostCenter;
    if (isRed) {
      desiredPassingLocation = FieldConstants.Outpost.redOutpostCenter;
    } else {
      desiredPassingLocation = FieldConstants.Outpost.blueOutpostCenter;
    }

    Rotation2d rot = calculateTurretRotation(robotPoseSupplier.get(), desiredPassingLocation);
    Rotation2d adjustedRot = new Rotation2d(Units.degreesToRadians(adjustTurretAngle(rot)));
    Rotation2d robotRot = robotPoseSupplier.get().getRotation();
    Rotation2d turretRot = adjustedRot.plus(robotRot);

    if (turretRot.getDegrees() < 0) {
      setPositionTurret(turretRot.getDegrees() + 360);
      SmartDashboard.putNumber("autoAimTurretPassing", turretRot.getDegrees() + 360);
    } else {
      setPositionTurret(turretRot.getDegrees());
      SmartDashboard.putNumber("autoAimTurretPassing", turretRot.getDegrees());
    }
  }

  @AutoLogOutput(key = "Odometry/TurretRotation")
  public Rotation2d calculateTurretRotation(Pose2d robotPose, Pose2d goalPose) {
    // Calculate difference
    double turretPoseX = robotPose.getX() + turretOffset.getX();
    double turretPoseY = robotPose.getY() + turretOffset.getY();

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
    SmartDashboard.putNumber("Turret Angle Degrees calc", Units.radiansToDegrees(angleRad));

    return Rotation2d.fromRadians(angleRad);
  }

  /** Returns the adjusted Turret Angle. */
  @AutoLogOutput(key = "Odometry/adjustedTurretRotation")
  public double adjustTurretAngle(Rotation2d calculatedTurretAngle) {

    // Turret Map
    //     180
    //  90     270
    //      0
    //
    // ROBOT INTAKE IS HERE

    // Formula: −calc + 90
    // Calculated		Encoder (actual)
    // 90	      0	    0
    // 135	     −45	−45
    // 179	     −89	−91
    // −179	   −91	−89
    // −135	   −135	−135
    // −90	     −180	−180

    Rotation2d adjustedTurretAngle = calculatedTurretAngle;
    double adjustedTurretAngleDegrees =
        adjustedTurretAngle
            .getDegrees(); // + drive.getRotation().getDegrees(); // get a degree value
    double encoderAngle =
        adjustedTurretAngleDegrees
            * -1; // We are getting values from -180 to 180 so we need to flip
    encoderAngle = encoderAngle + 360;

    encoderAngle = encoderAngle % 360; // Stay on the unit circle (but degrees)

    // return Rotation2d.fromDegrees(encoderAngle);
    return encoderAngle;
  }

  public Command setTurretPositionHub(Supplier<Pose2d> robotPoseSupplier) {
    return Commands.run(() -> autoAimTurretHub(robotPoseSupplier), this);
  }

  public Command setTurretPositionPassing(Supplier<Pose2d> robotPoseSupplier) {
    return Commands.run(() -> autoAimTurretPassing(robotPoseSupplier), this);
  }

  public void RRShootOnTheMove(Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotRelVelocity,
      Supplier<ChassisSpeeds> robotFieldVelocity) {

        Translation2d hub = FieldConstants.fieldCenter;

        Translation2d robotToGoal = hub.minus(robotPose.get().getTranslation());

        double dist = robotToGoal.getDistance(new Translation2d());
        double fuelTimeofFlight =
        LauncherTable.flightTimeMap.getInterpolated(new InterpolatingDouble(dist)).value;

        Translation2d movingGoalLocation = new Translation2d();

        for(int i = 0; i< 5; i++) {
          double virtualGoalX = hub.getX() - fuelTimeofFlight * robotRelVelocity.get().vxMetersPerSecond;
          double virtualGoalY = hub.getY() - fuelTimeofFlight * robotRelVelocity.get().vyMetersPerSecond;

          Translation2d testGoalLocation = new Translation2d(virtualGoalX, virtualGoalY);

          Translation2d toTestGoal = testGoalLocation.minus(robotPose.get().getTranslation());

          double newFlightTime = LauncherTable.flightTimeMap.getInterpolated(new InterpolatingDouble(toTestGoal.getDistance(new Translation2d()))).value;

          if (i == 4) {
            movingGoalLocation = testGoalLocation;
          } else {
            fuelTimeofFlight = newFlightTime;
          }
        }
        // Pose2d pose= new Pose2d(movingGoalLocation, new Rotation2d());
        autoAimTurretHub(robotPose);
  }

  public void shootOnTheMove(
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
    
    Pose2d estimatedPose = robotPose.get();
    ChassisSpeeds robotRelativeVelocity = robotRelVelocity.get();
    Translation2d target = desiredHub.getTranslation();

    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    turretPose =
        new Pose2d(
            estimatedPose.getX() + turretOffset.getX(),
            estimatedPose.getY() + turretOffset.getY(),
            adjustedTurretRotation(robotPose, desiredHub));

    double turretToHubDistance = FieldConstants.getDistanceToHubCenter(turretPose);

    ChassisSpeeds robotVelocity = robotFieldVelocity.get();
    double robotAngle = estimatedPose.getRotation().getDegrees();

    double
        turretVelocityX = // subtract x from the y to tke into the account the robot's rotation when
            // shooting and moving at the same time
            robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (turretPose.getY() * Math.cos(robotAngle)
                        - (turretPose.getX() * Math.sin(robotAngle)));

    double
        turretVelocityY = // subtract y from the x to tke into the account the robot's rotation when
            // shooting and moving at the same time
            robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                    * (turretPose.getX() * Math.sin(robotAngle)
                        - (turretPose.getY() * Math.cos(robotAngle)));

    SmartDashboard.putNumber("Turret Y Velocity", turretVelocityY);
    SmartDashboard.putNumber("Turret X Velocity", turretVelocityX);

    SmartDashboard.putNumber("Robot Y Velocity", robotVelocity.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot X Velocity", robotVelocity.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot Angular Velocity", robotVelocity.omegaRadiansPerSecond);

    double fuelTimeofFlight;

    // for (int i = 0; i < 20; i++) {

    fuelTimeofFlight =
        LauncherTable.flightTimeMap.getInterpolated(new InterpolatingDouble(turretToHubDistance))
            .value;

    turretDisplacementX = turretVelocityX * fuelTimeofFlight;
    turretDisplacementY = turretVelocityY * fuelTimeofFlight;

    turretToHubDistance = target.getDistance(turretPose.getTranslation());

    turretPose =
        new Pose2d(
            robotPose
                .get()
                .getTranslation()
                .plus(new Translation2d(turretDisplacementX, turretDisplacementY)),
            turretPose.getRotation());

    SmartDashboard.putNumber("Turret Pose X", turretPose.getX());
    SmartDashboard.putNumber("Turret Pose Y", turretPose.getY());
    // }

    // Calculate final turret angle to hub using atan2 for correct quadrant handling
    Rotation2d fieldAngleToHub = target.minus(turretPose.getTranslation()).getAngle();
    // Convert field angle to motor encoder coordinates using the same transformation as
    // autoAimTurret

    // Set turret position using the same logic as autoAimTurret
    double setMovingTurretAngle = adjustedTurretRotation(() -> turretPose, desiredHub).getDegrees();
    SmartDashboard.putNumber("SOTM Moving Turret Angle", setMovingTurretAngle);

    
    if (setMovingTurretAngle < 0) {
      setMovingTurretAngle = setMovingTurretAngle + 360;
      SmartDashboard.putNumber("SOTM TurretAngle", setMovingTurretAngle);
    } else {
      SmartDashboard.putNumber("SOTM TurretAngle", setMovingTurretAngle);
    }

    // setPositionTurret(setMovingTurretAngle);

    // Log diagnostic values for troubleshooting
    SmartDashboard.putNumber("SOTM Distance to Hub", turretToHubDistance);
    // SmartDashboard.putNumber("SOTM Turret Angle", turretAngleDegrees);
    SmartDashboard.putNumber("SOTM Turret X Displacement", turretDisplacementX);
    SmartDashboard.putNumber("SOTM Turret Y Displacement", turretDisplacementY);
    Logger.recordOutput("SOTM Turret Pose", turretPose);
    Logger.recordOutput("SOTM Target", new Pose2d(target, Rotation2d.kZero));
  }

  @AutoLogOutput(key = "Odometry/adjustedTurretRotation")
  public Pose2d turretMovingPose() {
    // turretPose =
    //     new Pose2d(
    //         turretPose
    //             .getTranslation()
    //             .plus(new Translation2d(turretDisplacementX, turretDisplacementY)),
    //         turretPose.getRotation());

    return turretPose;
  }

  public Command shootOnTheMoveCommand(
      Supplier<Pose2d> robotPose,
      Supplier<ChassisSpeeds> robotRelativeVelocity,
      Supplier<ChassisSpeeds> robotFieldVelocity) {
    return Commands.run(
        () -> shootOnTheMove(robotPose, robotRelativeVelocity, robotFieldVelocity), this);
  }
}
