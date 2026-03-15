// Intake

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  public IntakeIO intakeIO;
  public IntakeInputsAutoLogged intakeInputs;

  public final double gearRatio = 7.1429;
  public final double pitchDiameter = 1.751;

  public Intake(IntakeIO intakeIO) {
    intakeInputs = new IntakeInputsAutoLogged();
    this.intakeIO = intakeIO;

    switch (Constants.currentMode) {
      case REAL:
        break;
      case REPLAY:
        break;
      case SIM:
        break;
      default:
        break;
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

  public void setAllCollect(double rotations, double volts) {
    setPositionIntake(rotations);
    setCollectVoltage(volts);
  }

  public Command setAllCollectCommand(double rotations, double volts) {
    return Commands.run(() -> setAllCollect(rotations, volts), this);
  }

  @Override
  public void periodic() {
    intakeIO.updateInputs(intakeInputs);
    Logger.processInputs("Intake", intakeInputs);
  }
}
