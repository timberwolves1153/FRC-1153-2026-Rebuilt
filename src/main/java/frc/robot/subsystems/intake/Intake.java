// Intake

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Intake extends SubsystemBase {

  public IntakeIO intakeIO;
  public IntakeInputsAutoLogged intakeInputs;

  public TrapezoidProfile.Constraints constraints;
  public ProfiledPIDController profiledPIDController;
  public SimpleMotorFeedforward intakeFF;

  private LoggedMechanism2d intakeMech2d;
  private LoggedMechanismRoot2d intakeRoot2d;
  private LoggedMechanismLigament2d intakeLig2d;

  private final Rotation2d elev_angle = Rotation2d.fromDegrees(90);
  public final double gearRatio = 7.1429;
  public final double pitchDiameter = 1.751;

  public Intake(IntakeIO intakeIO) {
    intakeInputs = new IntakeInputsAutoLogged();
    this.intakeIO = intakeIO;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        constraints = new TrapezoidProfile.Constraints(5, 10);
        profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);
        intakeFF = new SimpleMotorFeedforward(0, 0, 0);
        break;

      case SIM:
        constraints = new TrapezoidProfile.Constraints(5.0, 10.0);

        profiledPIDController = new ProfiledPIDController(40, 0, 0.1, constraints);

        intakeFF =
            new SimpleMotorFeedforward(
                0, 0.06, (DCMotor.getFalcon500(1).KvRadPerSecPerVolt * 1.7567) / 12);

        break;

      default:
        profiledPIDController = new ProfiledPIDController(0, 0, 0, new Constraints(5, 10));
        intakeFF = new SimpleMotorFeedforward(0, 0, 0);
        break;
    }

    intakeMech2d = new LoggedMechanism2d(3, Units.feetToMeters(6));
    intakeRoot2d =
        intakeMech2d.getRoot(
            "Elevator", (3.0 / 2.0) + Units.inchesToMeters(9.053), Units.inchesToMeters(12.689));
    intakeLig2d =
        new LoggedMechanismLigament2d(
            "Elevator Lig", Units.inchesToMeters(80), elev_angle.getDegrees());

    intakeRoot2d.append(intakeLig2d);
  }

  public void setDeployVoltage(double volts) {
    intakeIO.setDeployVoltage(volts);
  }

  public void stopDeploy() {
    intakeIO.stopDeploy();
  }

  public void resetDeployEncoder() {
    intakeIO.resetDeployEncoder();
  }

  public void setCollectVoltage(double volts) {
    intakeIO.setCollectVoltage(volts);
  }

  public void stopCollect() {
    intakeIO.stopCollect();
  }

  public void resetCollectEncoder() {
    intakeIO.resetCollectEncoder();
  }

  //   public double inchesToRotations(double inches) {
  //     return (7.1429 * inches) / (Math.PI * pitchDiameter);
  //   }

  //   public double rotationsToInches(double rotations) {
  //     return (rotations / gearRatio) * (Math.PI * 2 * pitchDiameter);
  //   }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
  }
}
