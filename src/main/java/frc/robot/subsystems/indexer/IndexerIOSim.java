package frc.robot.subsystems.indexer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private final DCMotorSim spinSim;
  private final DCMotorSim feederSim;

  private double appliedVoltsSpin = 0.0;
  private double appliedVoltsFeed = 0.0;

  public IndexerIOSim() {
    spinSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0, 0),
            DCMotor.getKrakenX60(1));

    feederSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0, 0),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    inputs.spinAppliedVolts = spinSim.getInputVoltage();
    inputs.spinCurrentAmps = spinSim.getCurrentDrawAmps();

    inputs.feedAppliedVolts = feederSim.getInputVoltage();
    inputs.feedCurrentAmps = feederSim.getCurrentDrawAmps();
  }

  /** Runs the Spin motor to serialize game pieces. */
  @Override
  public void runSpin(double volts) {
    spinSim.setInputVoltage(volts);
  }

  /** Runs the Feed motor to push piece into shooter. */
  @Override
  public void runFeeder(double volts) {
    feederSim.setInputVoltage(volts);
  }

  /** Stops both motors. */
  @Override
  public void stopAll() {
    stopSpin();
    stopFeeder();
  }

  @Override
  public void stopSpin() {
    spinSim.setInputVoltage(0);
  }

  @Override
  public void stopFeeder() {
    feederSim.setInputVoltage(0);
  }
}
