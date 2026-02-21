// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.interpolation.LauncherTable;
import frc.robot.subsystems.Superstructure;
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
  private final Superstructure superstructure;

  // operator
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  //   private final CommandXboxoperator opoperator = new CommandXboxoperator(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

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
                drive::addVisionMeasurement,
                new VisionIOPhotonVision(
                    VisionConstants.camera0Name, VisionConstants.robotToBlueCamera),
                new VisionIOPhotonVision(
                    VisionConstants.camera1Name, VisionConstants.robotToOrangeCamera));

        intake = new Intake(new IntakeIOTalonFX());
        indexer = new Indexer(new IndexerIOTalonFX());
        alignment =
            new Alignment(
                new AlignmentIOPhotonVision(
                    VisionConstants.camera1Name, AlignmentConstants.robotToOrangeCamera));
        flywheel = new Flywheel(new FlywheelIOTalonFX());
        // hood = new Hood(new HoodIOTalonFX());
        hood = new Hood(new HoodIO() {});
        turret = new Turret(new TurretIOTalonFX());
        launcherTable = new LauncherTable();

        superstructure = new Superstructure(drive, flywheel, hood, turret, launcherTable);

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
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera0", VisionConstants.robotToBlueCamera, drive::getPose),
                new VisionIOPhotonVisionSim(
                    "camera1", VisionConstants.robotToOrangeCamera, drive::getPose)
                // new VisionIOPhotonVisionSim(
                //     "camera2", VisionConstants.robotToCamera2, drive::getPose)
                // ,
                // new VisionIOPhotonVisionSim(
                //     "camera3", VisionConstants.robotToCamera3, drive::getPose)
                );
        alignment = new Alignment(new AlignmentIO() {});
        turret = new Turret(new TurretIOSim());
        launcherTable = new LauncherTable();
        superstructure = new Superstructure(drive, flywheel, hood, turret, launcherTable);

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

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        alignment = new Alignment(new AlignmentIO() {});
        flywheel = new Flywheel(new FlywheelIO() {});
        hood = new Hood(new HoodIO() {});
        turret = new Turret(new TurretIO() {});
        launcherTable = new LauncherTable();
        superstructure = new Superstructure(drive, flywheel, hood, turret, launcherTable);

        //      alignment = new Alignment(new AlignmentIO() {});

        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    updateDesiredHub();
    // Configure the button bindings
    configureButtonBindings();
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
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()));

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
    operator.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    //   operator.y().onTrue(Commands.runOnce(() -> drive.resetGyro(0), drive));

    // Drive Forward Button for testing
    //  operator.povUp().whileTrue(drive.sysIdDynamic(Direction.kForward));
    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    SmartDashboard.putNumber("Flywheel Manual RPS Input", -10);
    SmartDashboard.putNumber("Hood Manual Position Input", 0);

    // operator
    //     .rightBumper()
    //     .onTrue(
    //         new DeferredCommand(
    //             () -> {
    //               return new InstantCommand(
    //                   () ->
    //                       flywheel.setVelocityLeader(
    //                           SmartDashboard.getNumber("Flywheel Manual RPS Input", -10)));
    //             },
    //             Set.of(flywheel)));

    // operator.rightBumper().onFalse(new InstantCommand(() -> flywheel.stopFlywheel()));

    // operator
    //     .leftBumper()
    //     .onTrue(
    //         new DeferredCommand(
    //             () -> {
    //               return new InstantCommand(
    //                   () ->
    //                       hood.setPositionHood(
    //                           SmartDashboard.getNumber("Hood Manual Position Input", -0.2)));
    //             },
    //             Set.of(hood)));
    // operator.leftBumper().onFalse(new InstantCommand(() -> hood.stopHood()));

    // operator.rightBumper().whileTrue(new InstantCommand(() ->
    // superstructure.interpolateShot()));
    // operator.rightBumper().whileFalse(new InstantCommand(() -> flywheel.stopFlywheel()));
    // operator.rightBumper().whileFalse(new InstantCommand((\) -> hood.stopHood()));

    // operator
    //     .rightTrigger()
    //     .onTrue(new InstantCommand(() -> flywheel.setVoltageLeader(-4), flywheel));
    // operator.rightTrigger().onFalse(new InstantCommand(() -> flywheel.stopFlywheel(),
    // flywheel));

    // operator.leftBumper().onTrue(new InstantCommand(() -> indexer.runFeed(-3), indexer));
    // operator.leftBumper().onFalse(new InstantCommand(() -> indexer.stopFeeder(), indexer));

    // operator.leftBumper().onTrue(new InstantCommand(() -> indexer.runSpin(8), indexer));
    // operator.leftBumper().onFalse(new InstantCommand(() -> indexer.stopSpin(), indexer));

    // operator.leftTrigger().onTrue(new InstantCommand(() -> intake.setCollectVoltage(-12),
    // intake));
    // operator.leftTrigger().onFalse(new InstantCommand(() -> intake.setCollectVoltage(0),
    // intake));

    // operator.leftBumper().onTrue(new InstantCommand(() -> turret.setPositionTurret(-0.9445)));
    // operator.leftBumper().onFalse(new InstantCommand(() -> turret.stopTurret()));

    // operator.y().onTrue(new InstantCommand(() -> turret.setPositionTurret(-0.47)));
    // operator.a().onTrue(new InstantCommand(() -> turret.setPositionTurret(-0.165)));

    // operator.leftBumper().whileTrue(new InstantCommand(() -> superstructure.autoAimTurret()));

    // operator.y().onTrue(new InstantCommand(() -> hood.setVoltageHood(-2)));
    // operator.a().onTrue(new InstantCommand(() -> hood.setVoltageHood(2)));

    // operator.y().onFalse(new InstantCommand(() -> hood.setVoltageHood(0)));
    // operator.a().onFalse(new InstantCommand(() -> hood.setVoltageHood(0)));

    //  operator.rightBumper().onTrue(drive.driveToTower());

    //  operator.y().onTrue(DriveCommands.driveToPose(drive.getTestPose(), drive));

    // operator.y().onTrue(new InstantCommand(() -> launcher.setVoltageLeader(5), launcher));
    // operator.y().onFalse(new InstantCommand(() -> launcher.setVoltageLeader(0), launcher));

    // operator.y().onTrue(new InstantCommand(() -> launcher.setVoltageFollower(-5), launcher));
    // operator.y().onFalse(new InstantCommand(() -> launcher.setVoltageFollower(0), launcher));

    // // operator.y().onTrue(new InstantCommand(() -> launcher.runVoltsFollower(5), launcher));
    // operator.povDown().onTrue(new InstantCommand(() -> climber.setVoltage(-5)));
    // operator.povUp().onTrue(new InstantCommand(() -> climber.setVoltage(5)));
    // operator.povUp().onFalse(new InstantCommand(() -> climber.setVoltage(0)));
    // operator.povDown().onFalse(new InstantCommand(() -> climber.setVoltage(0)));

    // operator.rightBumper().onTrue(new InstantCommand(() -> intake.setCollectVoltage(-10)));
    // operator.rightBumper().onFalse(new InstantCommand(() -> intake.setCollectVoltage(0)));

    // operator.rightTrigger().onTrue(new InstantCommand(() -> indexer.runSpin(-12)));
    // operator.rightTrigger().onFalse(new InstantCommand(() -> indexer.runSpin(0)));

    // SmartDashboard.putNumber("cameraX", -10.625);
    // SmartDashboard.putNumber("cameraY", -3.75);
    // SmartDashboard.putNumber("cameraZ", 17.75);
    // SmartDashboard.putNumber("cameraRoll", 0);
    // SmartDashboard.putNumber("cameraPitch", -20);
    // SmartDashboard.putNumber("cameraYaw", 210);

    // operator
    //     .leftBumper()
    //     .onTrue(
    //         new DeferredCommand(
    //             () -> {
    //               return new InstantCommand(() -> alignment.updateCameraPosition());
    //             },
    //             Set.of()));

    operator.povLeft().onTrue(new InstantCommand(() -> turret.setPositionTurret(-90), turret));
    operator.povRight().onTrue(new InstantCommand(() -> turret.setPositionTurret(0), turret));

    /* Week 0 Bindings */

    driver.x().onTrue(drive.driveToTower());

    operator.leftBumper().onTrue(new InstantCommand(() -> intake.setCollectVoltage(-11), intake));
    operator.leftBumper().onFalse(new InstantCommand(() -> intake.setCollectVoltage(0), intake));

    operator.leftTrigger().onTrue(new InstantCommand(() -> indexer.runFeed(-3), indexer));
    operator.leftTrigger().onFalse(new InstantCommand(() -> indexer.stopFeeder(), indexer));

    operator.leftTrigger().onTrue(new InstantCommand(() -> indexer.runSpin(8), indexer));
    operator.leftTrigger().onFalse(new InstantCommand(() -> indexer.stopSpin(), indexer));

    operator.rightTrigger().onTrue(new InstantCommand(() -> superstructure.interpolateShot()));
    operator.rightTrigger().onFalse(new InstantCommand(() -> flywheel.stopFlywheel(), flywheel));

    // operator.rightTrigger().onTrue(new InstantCommand(() -> superstructure.autoAimTurret()));

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
