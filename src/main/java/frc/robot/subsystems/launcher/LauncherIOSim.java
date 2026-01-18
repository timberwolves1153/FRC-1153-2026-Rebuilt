package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class LauncherIOSim implements LauncherIO {
  private final FlywheelSim simLeader;
  private final FlywheelSim simFollower;

  private double appliedVoltsLeader = 0.0;
  private double appliedVoltsFollower = 0.0;

  public LauncherIOSim() {
    simLeader =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));

    simFollower =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {

    if (DriverStation.isDisabled()) {
      simLeader.setInputVoltage(0);
      simFollower.setInputVoltage(0);
    }

    inputs.leadAppliedVoltage = simLeader.getInputVoltage();
    inputs.leadCurrent = simLeader.getCurrentDrawAmps();

    inputs.followerAppliedVoltage = simFollower.getInputVoltage();
    inputs.followerCurrent = simFollower.getCurrentDrawAmps();
  }

  @Override
  public void runVoltsLeader(double volts) {
    appliedVoltsLeader = MathUtil.clamp(volts, -12.0, 12.0);
    simLeader.setInputVoltage(appliedVoltsLeader);
  }

  @Override
  public void runVoltsFollower(double volts) {
    appliedVoltsFollower = MathUtil.clamp(volts, -12.0, 12.0);
    simFollower.setInputVoltage(appliedVoltsFollower);
  }

  @Override
  public void stopLeader() {
    simLeader.setInputVoltage(0);
  }

  @Override
  public void stopFollower() {
    simFollower.setInputVoltage(0);
  }
}
