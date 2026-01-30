package frc.robot.subsystems.launcher.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class TurretIOSim implements TurretIO {
  private final DCMotorSim turretSim;

  public TurretIOSim() {
    turretSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 1, 1),
            DCMotor.getKrakenX44(1));
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {

    if (DriverStation.isDisabled()) {
      turretSim.setInputVoltage(0);
    }

    inputs.turretAppliedVoltage = turretSim.getInputVoltage();
    inputs.turretCurrent = turretSim.getCurrentDrawAmps();
    inputs.turretPositionRad = turretSim.getAngularPositionRad();
  }

  // TODO: add methods
}
