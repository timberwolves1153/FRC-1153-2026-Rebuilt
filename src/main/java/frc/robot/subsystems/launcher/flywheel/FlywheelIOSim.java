package frc.robot.subsystems.launcher.flywheel;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FlywheelIOSim implements FlywheelIO {
  private final FlywheelSim leaderSim;
  private final FlywheelSim followerSim;

  private double appliedVoltsLeader = 0.0;
  private double appliedVoltsFollower = 0.0;
  private double appliedVelocityLeader = 0.0;
  private double appliedVelocityFollower = 0.0;

  public FlywheelIOSim() {
    leaderSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));

    followerSim =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    if (DriverStation.isDisabled()) {
      leaderSim.setInputVoltage(0);
      followerSim.setInputVoltage(0);
    }

    inputs.leadAppliedVoltage = leaderSim.getInputVoltage();
    inputs.leadCurrent = leaderSim.getCurrentDrawAmps();
    inputs.leadVelocity = leaderSim.getAngularVelocityRPM() / 60;

    inputs.followerAppliedVoltage = followerSim.getInputVoltage();
    inputs.followerCurrent = followerSim.getCurrentDrawAmps();
    inputs.leadVelocity = followerSim.getAngularVelocityRPM() / 60;
  }

  @Override
  public void setVoltageLeader(double volts) {
    appliedVoltsLeader = MathUtil.clamp(volts, -12.0, 12.0);
    SmartDashboard.putNumber("Flyhweel volts", volts);
    leaderSim.setInputVoltage(appliedVoltsLeader);
  }

  @Override
  public void setVoltageFollower(double volts) {
    appliedVoltsFollower = MathUtil.clamp(volts, -12.0, 12.0);
    leaderSim.setInputVoltage(appliedVoltsFollower);
  }

  @Override
  public void setVelocityLeader(double velocity) {
    leaderSim.setAngularVelocity(30);
  }

  @Override
  public void stopFlywheel() {
    leaderSim.setInputVoltage(0);
    leaderSim.setInputVoltage(0);
  }
}
