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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
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
  // private final Launcher launcher;
  // private final Climber climber;
  private final Vision vision;
  // private final Alignment alignment;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  //   private final CommandXboxController opController = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Match constants

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // instantiates a new drive joystick with the XboxController class

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
        drive.setPose(Pose2d.kZero.rotateBy(Rotation2d.k180deg));
        // climber = new Climber(new ClimberIOTalonFX());

        // launcher = new Launcher(new LauncherIOTalonFX());

        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

        // alignment =
        //     new Alignment(
        //         new AlignmentIOPhotonVision(
        //             AlignmentConstants.cameraName, AlignmentConstants.robotToCamera));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
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

        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    "camera0", VisionConstants.robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(
                    "camera1", VisionConstants.robotToCamera1, drive::getPose),
                new VisionIOPhotonVisionSim(
                    "camera2", VisionConstants.robotToCamera2, drive::getPose)
                // ,
                // new VisionIOPhotonVisionSim(
                //     "camera3", VisionConstants.robotToCamera3, drive::getPose)
                );

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
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});

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
    Pose2d hubPose =
        DriverStation.getAlliance()
            .map(
                alliance ->
                    alliance == Alliance.Red
                        ? FieldConstants.Hub.redHubCenter
                        : FieldConstants.Hub.blueHubCenter)
            .orElse(FieldConstants.Hub.redHubCenter); // or "" or "Unknown"

    drive.setDesiredHub(hubPose);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> controller.getLeftY(),
                () -> controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    controller.y().onTrue(Commands.runOnce(() -> drive.resetGyro(0), drive));

    // Drive Forward Button for testing
    controller.povUp().whileTrue(drive.sysIdDynamic(Direction.kForward));
    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    controller.y().onTrue(DriveCommands.driveToPose(drive.getTestPose(), drive));

    // controller.y().onTrue(new InstantCommand(() -> launcher.setVoltageLeader(5), launcher));
    // controller.y().onFalse(new InstantCommand(() -> launcher.setVoltageLeader(0), launcher));

    // controller.y().onTrue(new InstantCommand(() -> launcher.setVoltageFollower(-5), launcher));
    // controller.y().onFalse(new InstantCommand(() -> launcher.setVoltageFollower(0), launcher));

    // // controller.y().onTrue(new InstantCommand(() -> launcher.runVoltsFollower(5), launcher));
    // controller.povDown().onTrue(new InstantCommand(() -> climber.setVoltage(-5)));
    // controller.povUp().onTrue(new InstantCommand(() -> climber.setVoltage(5)));
    // controller.povUp().onFalse(new InstantCommand(() -> climber.setVoltage(0)));
    // controller.povDown().onFalse(new InstantCommand(() -> climber.setVoltage(0)));

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
