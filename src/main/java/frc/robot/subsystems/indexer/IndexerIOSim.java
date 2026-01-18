package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IndexerIOSim implements IndexerIO {
  private DCMotorSim indexerSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 0, 0),
          DCMotor.getKrakenX60(1));

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(IndexerIOInputs inputs) {
    indexerSim.setInputVoltage(appliedVolts);
    indexerSim.update(0.02);

    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = indexerSim.getCurrentDrawAmps();
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, 12, -12);
  }
}
