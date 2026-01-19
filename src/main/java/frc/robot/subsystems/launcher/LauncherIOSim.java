package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LauncherIOSim implements LauncherIO {

  /* Flywheel */

  private final FlywheelSim simLeader;
  private final FlywheelSim simFollower;

  private double appliedVoltsLeader = 0.0;
  private double appliedVoltsFollower = 0.0;

  /* Hood */

  private final DCMotorSim simHood;

  private double appliedVoltsHood = 0.0;
  private double positionDegreesHood = 0.0;

  /* Turret */

  private final DCMotorSim simTurret;

  private double appliedVoltsTurret = 0.0;

  public LauncherIOSim() {

    /* Flywheel */

    simLeader =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));

    simFollower =
        new FlywheelSim(
            LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
            DCMotor.getKrakenX60(1));

    /* Hood */

    simHood =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 1, 1),
            DCMotor.getKrakenX44(1));

    /* Turret */

    simTurret =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 1, 1),
            DCMotor.getKrakenX44(1));
  }

  @Override
  public void updateInputs(LauncherIOInputs inputs) {
    /*Flywheel */

    if (DriverStation.isDisabled()) {
      simLeader.setInputVoltage(0);
      simFollower.setInputVoltage(0);
    }

    inputs.leadAppliedVoltage = simLeader.getInputVoltage();
    inputs.leadCurrent = simLeader.getCurrentDrawAmps();

    inputs.followerAppliedVoltage = simFollower.getInputVoltage();
    inputs.followerCurrent = simFollower.getCurrentDrawAmps();

    /* Hood */

    // inputs.hoodAppliedVoltage = simHood.getInputVoltage();
    // inputs.hoodCurrent = simHood.getCurrentDrawAmps();
    // inputs.hoodPositionDegrees =
  }

  /* Flywheel */

  @Override
  public void setVoltageLeader(double volts) {
    appliedVoltsLeader = MathUtil.clamp(volts, -12.0, 12.0);
    SmartDashboard.putNumber("Flyhweel volts", volts);
    simLeader.setInputVoltage(appliedVoltsLeader);
  }

  @Override
  public void setVoltageFollower(double volts) {
    appliedVoltsFollower = MathUtil.clamp(volts, -12.0, 12.0);
    simFollower.setInputVoltage(appliedVoltsFollower);
  }

  @Override
  public void stopLauncher() {
    simLeader.setInputVoltage(0);
    simFollower.setInputVoltage(0);
  }

  /* Hood */

  // @Override
  // public void setPositionHood(double hoodDegrees) {
  //   simHood.setPositionHood(hoodDegrees);
  // }

}
