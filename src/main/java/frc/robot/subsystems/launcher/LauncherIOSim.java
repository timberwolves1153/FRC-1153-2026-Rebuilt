package frc.robot.subsystems.launcher;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class LauncherIOSim implements LauncherIO {
  private final DCMotorSim simLeader;
  private final DCMotorSim simFollower;

  private double appliedVoltsLeader = 0.0;

  public LauncherIOSim() {
    simLeader =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));

    simFollower =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    inputs.leadAppliedVoltage = simLeader.getInputVoltage();
    inputs.leadCurrent = simLeader.getCurrentDrawAmps();

    inputs.followerAppliedVoltage = simFollower.getInputVoltage();
    inputs.followerCurrent = simFollower.getCurrentDrawAmps();
  }

  @Override
  public void runVolts(double volts) {
    appliedVoltsLeader = Math.clamp(volts, -12.0, 12.0);
    simLeader.setInputVoltage(appliedVoltsLeader);
  }
}
