// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedUntilEmptyCommand;
import frc.robot.commands.SuperstructureCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.interpolation.LauncherTable;
import frc.robot.subsystems.alignment.Alignment;
import frc.robot.subsystems.alignment.AlignmentConstants;
import frc.robot.subsystems.alignment.AlignmentIO;
import frc.robot.subsystems.alignment.AlignmentIOPhotonVision;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOSim;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.flywheel.FlywheelIO;
import frc.robot.subsystems.launcher.flywheel.FlywheelIOSim;
import frc.robot.subsystems.launcher.flywheel.FlywheelIOTalonFX;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.hood.HoodIO;
import frc.robot.subsystems.launcher.hood.HoodIOSim;
import frc.robot.subsystems.launcher.hood.HoodIOTalonFX;
import frc.robot.subsystems.launcher.turret.Turret;
import frc.robot.subsystems.launcher.turret.TurretIO;
import frc.robot.subsystems.launcher.turret.TurretIOSim;
import frc.robot.subsystems.launcher.turret.TurretIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Flywheel flywheel;
  // private final Climber climber;
  private final Vision vision;
  private final Alignment alignment;
  private final Hood hood;
  private final Turret turret;
  private final LauncherTable launcherTable;

  // operator
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final SuperstructureCommands superstructureCommands;
  private final FeedUntilEmptyCommand feedUntilEmptyCommand;

  // Match constants

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // instantiates a new drive joystick with the Xboxoperator class

    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        drive.setPose(
            Pose2d.kZero.rotateAround(
                FieldConstants.fieldCenter, Rotation2d.k180deg)); // rotateBy(Rotation2d.k180deg).

        // climber = new Climber(new ClimberIOTalonFX());

        // launcher = new Launcher(new LauncherIOTalonFX());

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVision(
        //             VisionConstants.camera1Name, VisionConstants.robotToOrangeCamera));
        vision =
            new Vision(
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToTurretCamera,
                    drive::addVisionMeasurement,
                    drive::getRotation3d),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToClimberCamera,
                    drive::addVisionMeasurement,
                    drive::getRotation3d),
                new VisionIOPhotonVision(
                    VisionConstants.camera2Name,
                    VisionConstants.robotToSwerveCamera,
                    drive::addVisionMeasurement,
                    drive::getRotation3d),
                new VisionIOPhotonVision(
                    VisionConstants.camera3Name,
                    VisionConstants.robotToHopperCamera,
                    drive::addVisionMeasurement,
                    drive::getRotation3d));
        // drive::addVisionMeasurement,
        // new VisionIOPhotonVision(
        //     VisionConstants.camera0Name, VisionConstants.robotToBlueCamera),
        // new VisionIOPhotonVision(
        //     VisionConstants.camera1Name, VisionConstants.robotToOrangeCamera));

        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        alignment =
            new Alignment(
                new AlignmentIOPhotonVision(
                    VisionConstants.camera1Name, AlignmentConstants.robotToOrangeCamera));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        // hood = new Hood(new HoodIO() {});
        turret = new Turret(new TurretIOTalonFX());
        // turret = new Turret(new TurretIO() {});
        launcherTable = new LauncherTable();

        // alignment =
        //     new Alignment(
        //         new AlignmentIOPhotonVision(
        //             AlignmentConstants.cameraName, AlignmentConstants.robotToCamera));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS operator connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));

        // launcher = new Launcher(new LauncherIOSim());
        // climber = new Climber(new ClimberIOSim());

        intake = new Intake(new IntakeIOSim());
        indexer = new Indexer(new IndexerIOSim());
        flywheel = new Flywheel(new FlywheelIOSim());
        hood = new Hood(new HoodIOSim());

        vision =
            new Vision(
                // new VisionIO() {}
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name,
                    VisionConstants.robotToTurretCamera,
                    drive::getPose,
                    drive::addVisionMeasurement,
                    drive::getRotation3d),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToClimberCamera,
                    drive::getPose,
                    drive::addVisionMeasurement,
                    drive::getRotation3d),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera2Name,
                    VisionConstants.robotToSwerveCamera,
                    drive::getPose,
                    drive::addVisionMeasurement,
                    drive::getRotation3d),
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera3Name,
                    VisionConstants.robotToHopperCamera,
                    drive::getPose,
                    drive::addVisionMeasurement,
                    drive::getRotation3d));
        alignment = new Alignment(new AlignmentIO() {});
        turret = new Turret(new TurretIOSim());
        launcherTable = new LauncherTable();

        // alignment =
        //     new Alignment(
        //         new AlignmentIOPhotonVisionSim(
        //             AlignmentConstants.cameraName,
        //             AlignmentConstants.robotToCamera,
        //             drive::getPose));

        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        // launcher = new Launcher(new LauncherIO() {});
        // climber = new Climber(new ClimberIO() {});

        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});

        vision = new Vision(new VisionIO() {}, new VisionIO() {});
        alignment = new Alignment(new AlignmentIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});
        turret = new Turret(new TurretIO() {});
        launcherTable = new LauncherTable();

        //      alignment = new Alignment(new AlignmentIO() {});

        break;
    }

    superstructureCommands = new SuperstructureCommands(drive, flywheel, hood, turret, indexer);
    feedUntilEmptyCommand = new FeedUntilEmptyCommand(indexer);

    // Set up auto routines

    /*Autonomous Commands*/
    NamedCommands.registerCommand("Run Indexer", indexer.setAllIndexingCommand(-12, 12)); // CHANGE
    NamedCommands.registerCommand("Stop Indexer", indexer.setAllIndexingCommand(0, 0));

    NamedCommands.registerCommand("Run Timed Indexer", feedUntilEmptyCommand);

    NamedCommands.registerCommand(
        "Run Feeder Wheel", new InstantCommand(() -> indexer.runFeed(-12), indexer));
    NamedCommands.registerCommand(
        "Stop Feeder Wheel", new InstantCommand(() -> indexer.runFeed(0), indexer));

    NamedCommands.registerCommand(
        "Run Indexer Wheel", new InstantCommand(() -> indexer.runSpin(12), indexer));
    NamedCommands.registerCommand(
        "Stop Indexer Wheel", new InstantCommand(() -> indexer.runSpin(0), indexer));

    NamedCommands.registerCommand(
        "Stop Flywheel", new InstantCommand(() -> flywheel.stopFlywheel()));
    NamedCommands.registerCommand(
        "Reset Hood", new InstantCommand(() -> hood.setPositionHood(-0.05)));
    NamedCommands.registerCommand(
        "Reset Turret", new InstantCommand(() -> turret.setPositionTurret(-90)));

    NamedCommands.registerCommand(
        "Rev Flywheel", new InstantCommand(() -> flywheel.setVelocityLeader(-35)));

    NamedCommands.registerCommand(
        "Interpolate Shot", superstructureCommands.interpolateShotCommand().withTimeout(0.25));
    NamedCommands.registerCommand(
        "Set Turret Position 300", new InstantCommand(() -> turret.setPositionTurret(290)));
    NamedCommands.registerCommand(
        "Set Manual Flywheel RPS", new InstantCommand(() -> flywheel.setVelocityLeader(-32.75)));
    NamedCommands.registerCommand(
        "Set Manual Hood Value", new InstantCommand(() -> hood.setPositionHood(-0.875)));

    NamedCommands.registerCommand("Aim to Score", superstructureCommands.shootOnTheMoveCommand());
    NamedCommands.registerCommand("Aim to Pass", superstructureCommands.autoAimTurretPassing());

    NamedCommands.registerCommand(
        "Deploy Intake", intake.setAllCollectCommand(4.9, 5.5).withTimeout(.5));

    NamedCommands.registerCommand(
        "Run Collector", new InstantCommand(() -> intake.setCollectVoltage(5.5)));
    NamedCommands.registerCommand(
        "Stop Collector", new InstantCommand(() -> intake.setCollectVoltage(0)));

    NamedCommands.registerCommand(
        "Reset Robot Pose Turret",
        new InstantCommand(() -> drive.setPose(vision.getPoseFromTurretCamera())));

    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    updateDesiredHub();

    // Configure the button bindings
    configureButtonBindings();

    RobotModeTriggers.autonomous()
        .or(RobotModeTriggers.teleop())
        .onTrue(Commands.run(() -> hood.homeHood()).until(hood::isHomed));

    RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> flywheel.setVelocityLeader(-10)));
    RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> hood.stopHood()));
    RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> turret.stopTurret()));
    RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> indexer.stopAll()));
    RobotModeTriggers.teleop().onTrue(new InstantCommand(() -> intake.setCollectVoltage(0)));
  }

  public void updateDesiredHub() {

    Pose2d hubPose = FieldConstants.Hub.redHubCenter;
    // DriverStation.getAlliance()
    //     .map(
    //         alliance ->
    //             alliance == Alliance.Red
    //                 ? FieldConstants.Hub.redHubCenter
    //                 : FieldConstants.Hub.blueHubCenter)
    //     .orElse(FieldConstants.Hub.redHubCenter); // or "" or "Unknown"

    drive.setDesiredHub(hubPose);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link Xboxoperator}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // driver.x().onTrue(drive.driveToTower());

    // Lock to 0° when A button is held
    // operator
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> operator.getLeftY(),
    //             () -> operator.getLeftX(),
    //             () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    //  operator.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // driver.y().onTrue(Commands.runOnce(() -> drive.resetGyro(0), drive));

    // Set robot rotation to 45 degrees when X button is pressed
    driver
        .rightBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> Rotation2d.fromDegrees(45)));

    driver
        .leftBumper()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> Rotation2d.fromDegrees(-45)));

    // Drive Forward Button for testing
    //  operator.povUp().whileTrue(drive.sysIdDynamic(Direction.kForward));
    // Reset gyro to 0° when B button is pressed

    /* Week 0 Bindings */

    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    operator.rightTrigger().onTrue(new InstantCommand(() -> indexer.runFeed(-12), indexer));
    operator.rightTrigger().onFalse(new InstantCommand(() -> indexer.stopFeeder(), indexer));

    operator.rightTrigger().onTrue(new InstantCommand(() -> indexer.runSpin(10), indexer));
    operator.rightTrigger().onFalse(new InstantCommand(() -> indexer.stopSpin(), indexer));

    // operator
    //     .rightTrigger()
    //     .onTrue(superstructureCommands.runIndexerSafe(flywheel::getFlywheelCurrentRPS));

    // operator.rightBumper().onTrue(new InstantCommand(() -> indexer.runSpin(-12), indexer));
    // operator.rightBumper().onFalse(new InstantCommand(() -> indexer.stopSpin(), indexer));

    driver.rightTrigger().onTrue(new InstantCommand(() -> indexer.runFeed(-12), indexer));
    driver.rightTrigger().onFalse(new InstantCommand(() -> indexer.stopFeeder(), indexer));

    driver.rightTrigger().onTrue(new InstantCommand(() -> indexer.runSpin(12), indexer));
    driver.rightTrigger().onFalse(new InstantCommand(() -> indexer.stopSpin(), indexer));

    // driver.leftTrigger().onTrue(new InstantCommand(() -> flywheel.setVelocityLeader(-30)));

    // operator
    //     .rightTrigger()
    //     .onTrue(
    //         superstructureCommands.runIndexerSafe(flywheel::getFlywheelCurrentRPS));

    operator.leftStick().onTrue(superstructureCommands.shootOnTheMoveCommand());
    operator.rightStick().onTrue(superstructureCommands.passOnTheMoveCommand());

    operator.povRight().onTrue(new InstantCommand(() -> turret.setPositionTurret(300), turret));
    operator.povRight().onTrue(new InstantCommand(() -> hood.setPositionHood(-0.87), hood));
    operator.povRight().onTrue(new InstantCommand(() -> flywheel.setVelocityLeader(-33.0)));

    operator.povUp().onTrue(new InstantCommand(() -> turret.setPositionTurret(180), turret));
    operator.povUp().onTrue(new InstantCommand(() -> hood.setPositionHood(-0.8), hood));
    operator.povUp().onTrue(new InstantCommand(() -> flywheel.setVelocityLeader(-32.5)));

    operator.povLeft().onTrue(new InstantCommand(() -> turret.setPositionTurret(60), turret));
    operator.povLeft().onTrue(new InstantCommand(() -> hood.setPositionHood(-0.87), hood));
    operator.povLeft().onTrue(new InstantCommand(() -> flywheel.setVelocityLeader(-33.0)));

    // operator.leftBumper().onTrue(new InstantCommand(() -> intake.setPositionIntake(0.25)));
    // operator.leftTrigger().onTrue(new InstantCommand(() -> intake.setPositionIntake(4.9)));
    operator.leftTrigger().onTrue(new InstantCommand(() -> intake.setCollectVoltage(5), intake));
    operator.leftTrigger().onFalse(new InstantCommand(() -> intake.setCollectVoltage(0), intake));

    operator.a().onTrue(new InstantCommand(() -> hood.homeHood(), hood));

    driver.leftTrigger().onTrue(new InstantCommand(() -> intake.setCollectVoltage(5), intake));
    driver.leftTrigger().onFalse(new InstantCommand(() -> intake.setCollectVoltage(0), intake));

    // driver
    //     .x()
    //     .onTrue(
    //         Commands.runOnce(
    //             () ->
    //                 drive.setPose(
    //                     new Pose2d(drive.getPose().getTranslation(),
    // Rotation2d.fromDegrees(45))),
    //             drive));

    // driver.rightTrigger().onTrue(superstructureCommands.shootOnTheMoveCommand());
    // operator.rightTrigger().onTrue(superstructureCommands.shootOnTheMoveCommand());

    // SmartDashboard.putNumber("Flywheel Manual RPS Input", -10);

    // operator
    //     .leftStick()
    //     .onTrue(
    //         new DeferredCommand(
    //             () -> {
    //               return new InstantCommand(
    //                   () ->
    //                       flywheel.setVelocityLeader(
    //                           SmartDashboard.getNumber("Flywheel Manual RPS Input", -10)));
    //             },
    //             Set.of(flywheel)));

    // // operator.leftStick().onTrue(hood.setPositionHoodHub(drive::getPose));
    // operator.leftStick().onTrue(turret.setTurretPositionHub(drive::getPose));

    // SmartDashboard.putNumber("Hood Manual Setpoint Input", -0.05);
    // operator
    //     .leftStick()
    //     .onTrue(
    //         new DeferredCommand(
    //             () -> {
    //               return new InstantCommand(
    //                   () ->
    //                       hood.setPositionHood(
    //                           SmartDashboard.getNumber("Hood Manual Setpoint Input", -0.05)));
    //             },
    //             Set.of(hood)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
