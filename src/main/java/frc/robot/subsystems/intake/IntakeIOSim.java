// IntakeIOSim

package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {

  private DCMotorSim deployMotorSim;
  private DCMotorSim collectMotorSim;

  private Voltage volts;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(5.0, 10.0);

  double kp = 5; /* proportional coefficent */
  double ki = 5; /* integral coefficent */
  double kd = 5; /* derivitve coefficent */

  ProfiledPIDController profiledPIDController = new ProfiledPIDController(40, 0, 0.1, constraints);
  double ks = 5; /* static gain in volts */
  double kg = 5; /* gravity gain in volts */
  double kv = 5; /* velocity gain in V/(m/s)*/

  SimpleMotorFeedforward intakeFF =
      new SimpleMotorFeedforward(
          0, 0.06, (DCMotor.getFalcon500(1).KvRadPerSecPerVolt * 1.7567) / 12);

  public IntakeIOSim() {

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
    intakeInputs.deployAppliedVolts = deployMotorSim.getInputVoltage();

    intakeInputs.collectCurrentAmps = collectMotorSim.getCurrentDrawAmps();
    intakeInputs.collectAppliedVolts = collectMotorSim.getInputVoltage();
  }

  public double getDeployPosition() {
    return Units.degreesToRotations(deployMotorSim.getAngularPositionRotations());
  }

  @Override
  public void setDeployVoltage(double volts) {
    deployMotorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void setCollectVoltage(double volts) {
    collectMotorSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }
}
