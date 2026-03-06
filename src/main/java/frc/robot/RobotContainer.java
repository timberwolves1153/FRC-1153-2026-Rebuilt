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

  //   private final CommandXboxoperator opoperator = new CommandXboxoperator(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final SuperstructureCommands superstructureCommands;

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
                    VisionConstants.robotToBlueCamera,
                    drive::addVisionMeasurement),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name,
                    VisionConstants.robotToOrangeCamera,
                    drive::addVisionMeasurement));

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

        vision = new Vision(new VisionIO() {});
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

    superstructureCommands = new SuperstructureCommands(drive, flywheel, hood, turret);

    // Set up SysId routines
    // autoChooser.addOption(
    //     "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Forward)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Quasistatic Reverse)",
    //     drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // autoChooser.addOption(
    //     "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // NamedCommands.registerCommand(
    //     "Run Feeder Wheel", new InstantCommand(() -> indexer.runFeed(-9)));
    // NamedCommands.registerCommand(
    //     "Stop Feeder Wheel", new InstantCommand(() -> indexer.runFeed(0)));

    // NamedCommands.registerCommand(
    //     "Run Indexer", new InstantCommand(() -> indexer.runSpin(10)));
    // NamedCommands.registerCommand(
    //     "Stop Indexer", new InstantCommand(() -> indexer.runSpin(0)));

    NamedCommands.registerCommand("Run Indexer", new FeedUntilEmptyCommand(indexer));

    NamedCommands.registerCommand("Stop Indexer", indexer.stopIndexer());

    NamedCommands.registerCommand("Shoot Fuel", superstructureCommands.autoAimTurretHub());

    NamedCommands.registerCommand(
        "Stop Flywheel", new InstantCommand(() -> flywheel.stopFlywheel()));
    NamedCommands.registerCommand(
        "Reset Hood", new InstantCommand(() -> hood.setPositionHood(-0.05)));
    // NamedCommands.registerCommand(
    //     "Reset Turret", new InstantCommand(() -> turret.setPositionTurret(-90)));

    updateDesiredHub();

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Configure the button bindings
    configureButtonBindings();

    RobotModeTriggers.autonomous()
        .or(RobotModeTriggers.teleop())
        .onTrue(Commands.run(() -> hood.homeHood()).until(hood::isHomed));
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

    //   driver.x().onTrue(drive.driveToTower());

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

    //   operator.y().onTrue(Commands.runOnce(() -> drive.resetGyro(0), drive));

    // Drive Forward Button for testing
    //  operator.povUp().whileTrue(drive.sysIdDynamic(Direction.kForward));
    // Reset gyro to 0° when B button is pressed

    /* Week 0 Bindings */

    //   driver.x().onTrue(drive.driveToTower());

    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    driver.leftBumper().onTrue(new InstantCommand(() -> intake.setCollectVoltage(-11), intake));
    driver.leftBumper().onFalse(new InstantCommand(() -> intake.setCollectVoltage(0), intake));

    driver.rightBumper().onTrue(new InstantCommand(() -> indexer.runFeed(-9), indexer));
    driver.rightBumper().onFalse(new InstantCommand(() -> indexer.stopFeeder(), indexer));

    driver.rightBumper().onTrue(new InstantCommand(() -> indexer.runSpin(10), indexer));
    driver.rightBumper().onFalse(new InstantCommand(() -> indexer.stopSpin(), indexer));

    // driver.rightTrigger().onTrue(superstructureCommands.shootOnTheMoveCommand());
    // driver.rightTrigger().onFalse(new InstantCommand(() -> turret.stopTurret()));

    driver.rightTrigger().onTrue(superstructureCommands.autoAimTurretHub());
    driver.rightTrigger().onFalse(new InstantCommand(() -> flywheel.stopFlywheel(), flywheel));
    driver.rightTrigger().onFalse(new InstantCommand(() -> hood.setPositionHood(0), hood));

    driver.leftTrigger().onTrue(new FeedUntilEmptyCommand(indexer));
    // driver.povUp().onTrue(new InstantCommand(() -> turret.setPositionTurret(180)));
    // driver.povUp().onTrue(superstructureCommands.interpolateShot());
    // driver.povUp().onFalse(new InstantCommand(() -> flywheel.stopFlywheel(), flywheel));
    // driver.povUp().onFalse(new InstantCommand(() -> hood.setPositionHood(0), hood));

    // driver.povLeft().onTrue(new InstantCommand(() -> turret.setPositionTurret(90)));
    // driver.povLeft().onTrue(superstructureCommands.interpolateShot());
    // driver.povLeft().onFalse(new InstantCommand(() -> flywheel.stopFlywheel(), flywheel));
    // driver.povLeft().onFalse(new InstantCommand(() -> hood.setPositionHood(0), hood));

    // driver.povRight().onTrue(new InstantCommand(() -> turret.setPositionTurret(270)));
    // driver.povRight().onTrue(superstructureCommands.interpolateShot());
    // driver.povRight().onFalse(new InstantCommand(() -> flywheel.stopFlywheel(), flywheel));
    // driver.povRight().onFalse(new InstantCommand(() -> hood.setPositionHood(0), hood));

    // driver.rightTrigger().onTrue(new InstantCommand(() -> turret.setVoltageTurret(2), turret));
    // driver.rightTrigger().onFalse(new InstantCommand(() -> turret.stopTurret(), turret));

    // driver.leftTrigger().onTrue(new InstantCommand(() -> turret.setVoltageTurret(-2), turret));
    // driver.leftTrigger().onFalse(new InstantCommand(() -> turret.stopTurret(), turret));

    // driver.rightBumper().onTrue(superstructureCommands.autoAimTurretPassing());
    // driver.rightBumper().onFalse(new InstantCommand(() -> flywheel.stopFlywheel(), flywheel));
    // driver.rightBumper().onFalse(new InstantCommand(() -> hood.setPositionHood(0), hood));
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
