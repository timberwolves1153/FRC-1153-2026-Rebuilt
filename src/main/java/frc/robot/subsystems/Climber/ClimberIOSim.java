package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimberIOSim implements ClimberIO {
  private ElevatorSim climberSim;
  private Voltage volts;

  double maxVel = 2;
  double maxAcc = 2;
  TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(maxVel, maxAcc);

  double kp = 40; /* proportional coefficent */
  double ki = 0; /* integral coefficent */
  double kd = 0.1; /* derivitve coefficent */
  ProfiledPIDController profiledPIDController = new ProfiledPIDController(kp, ki, kd, constraints);

  double ks = 0; /* static gain in volts */
  double kg = 0.6; /* gravity gain in volts */
  double kv =
      (DCMotor.getFalcon500(1).KvRadPerSecPerVolt * 1.7567) / 12; /* velocity gain in V/(m/s)*/
  ElevatorFeedforward climberFF = new ElevatorFeedforward(ks, kg, kv);

  public ClimberIOSim() {
    climberSim =
        new ElevatorSim(
            DCMotor.getFalcon500(1),
            12, /*UPDATE*/
            Units.lbsToKilograms(3), /*^^^^^^*/
            Units.inchesToMeters(1.751), /*^^^^^^*/
            0, /*^^^^^^*/
            Units.inchesToMeters(60), /*^^^^^^*/
            true,
            0);

    volts = Voltage.ofBaseUnits(0.0, Volts);
  }

  @Override
  public void updateInputs(ClimberInputs climberInputs) {
    climberSim.update(0.02);

    climberInputs.heightInches = Units.metersToInches(climberSim.getPositionMeters());
    climberInputs.climberCurrentAmps = climberSim.getCurrentDrawAmps();
    climberInputs.tempCelsius = 20;
    climberInputs.getAppliedVolts = volts.baseUnitMagnitude();
    climberInputs.leaderRotations = 0;
  }

  public double getClimberHeight() {
    return Units.metersToInches(climberSim.getPositionMeters());
  }

  @Override
  public void setVoltage(final Voltage voltage) {
    volts = voltage;
    double simVolts = voltage.baseUnitMagnitude();
    climberSim.setInputVoltage(MathUtil.clamp(simVolts, -12, 12));
  }

  @Override
  public void stop() {
    climberSim.setInputVoltage(0);
  }

  @Override
  public void setTargetHeight(double inches) {
    double calculatedVolts =
        profiledPIDController.calculate(
                climberSim.getPositionMeters(), Units.inchesToMeters(inches))
            + climberFF.calculate(profiledPIDController.getSetpoint().velocity);
    setVoltage(Voltage.ofBaseUnits(calculatedVolts, Volts));
  }
}
