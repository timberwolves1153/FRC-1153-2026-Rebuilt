// Intake

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public IntakeIO intakeIO;
  public IntakeInputsAutoLogged intakeInputs;

  // public TrapezoidProfile.Constraints constraints;
  // public ProfiledPIDController profiledPIDController;
  // public SimpleMotorFeedforward intakeFF;

  public final double gearRatio = 7.1429;
  public final double pitchDiameter = 1.751;

  public Intake(IntakeIO intakeIO) {
    intakeInputs = new IntakeInputsAutoLogged();
    this.intakeIO = intakeIO;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        // constraints = new TrapezoidProfile.Constraints(5, 10);
        // profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);
        // intakeFF = new SimpleMotorFeedforward(0, 0, 0);
        break;

      case SIM:
        // constraints = new TrapezoidProfile.Constraints(5.0, 10.0);

        // profiledPIDController = new ProfiledPIDController(40, 0, 0.1, constraints);

        // intakeFF =
        //     new SimpleMotorFeedforward(
        //         0, 0.06, (DCMotor.getFalcon500(1).KvRadPerSecPerVolt * 1.7567) / 12);

        break;

        // default:
        //   profiledPIDController = new ProfiledPIDController(0, 0, 0, new Constraints(5, 10));
        //   intakeFF = new SimpleMotorFeedforward(0, 0, 0);
        //   break;
    }
  }

  public void setDeployVoltage(double volts) {
    intakeIO.setDeployVoltage(volts);
  }

  public void resetDeployEncoder() {
    intakeIO.resetDeployEncoder();
  }

  public void setCollectVoltage(double volts) {
    intakeIO.setCollectVoltage(volts);
  }

  public void setPositionIntake(double rotations) {
    intakeIO.setPositionIntake(rotations);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
  }
}
