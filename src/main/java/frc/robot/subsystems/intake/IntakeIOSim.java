// IntakeIOSim

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private DCMotorSim deployMotorSim;
  private DCMotorSim collectMotorSim;

  private Voltage volts;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5.0, 10.0);

  ProfiledPIDController profiledPIDController = new ProfiledPIDController(40, 0, 0.1, constraints);

  ElevatorFeedforward elevatorFF =
      new ElevatorFeedforward(0, 0.06, (DCMotor.getFalcon500(1).KvRadPerSecPerVolt * 1.7567) / 12);

  public IntakeIOSim() {
    // deployMotorSim =
    //     new DCMotorSim(DCMotor.getKrakenX44(1),
    //     12,
    //     Units.lbsToKilograms(17.966),
    //     Units.inchesToMeters(1.751),
    //     0,
    //     Units.inchesToMeters(23),
    //     true,
    //     0);

    // collectMotorSim =
    // new DCMotorSim(DCMotor.getKrakenX60(1),
    // 12,
    // Units.lbsToKilograms(17.966),
    // Units.inchesToMeters(1.751),
    // 0,
    // Units.inchesToMeters(23),
    // true,
    // 0);
    deployMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 1, 1),
            DCMotor.getKrakenX44(1));

    collectMotorSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX44(1));

    volts = Voltage.ofBaseUnits(0.0, Volts);
  }
  // there will be two objects, fix methods accordingly.
  @Override
  public void updateInputs(IntakeInputs intakeInputs) {
    deployMotorSim.update(0.02);
    collectMotorSim.update(0.02);

    intakeInputs.deployCurrentAmps = deployMotorSim.getCurrentDrawAmps();
    intakeInputs.deployAppliedVolts = volts.baseUnitMagnitude();

    intakeInputs.collectCurrentAmps = collectMotorSim.getCurrentDrawAmps();
    intakeInputs.collectAppliedVolts = volts.baseUnitMagnitude();
  }

  @Override
  public void setDeployVoltage(double volts) {
    deployMotorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void stopDeploy() {
    deployMotorSim.setInputVoltage(0);
  }

  @Override
  public void setCollectVoltage(double volts) {
    collectMotorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void stopCollect() {
    collectMotorSim.setInputVoltage(0);
  }
}
