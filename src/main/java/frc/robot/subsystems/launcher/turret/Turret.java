package frc.robot.subsystems.launcher.turret;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private PIDController turretPID;

  public Turret(TurretIO turretIO) {
    io = turretIO;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        break;

      case SIM:
        break;
    }

    turretPID = new PIDController(0, 0, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Launcher", inputs);
  }

  public void setTurretPosition(double rotations) {
    io.setPositionTurret(rotations);
  }

  public void setVoltageTurret(double volts) {
    io.setVoltageTurret(volts);
  }

  public void stopTurret() {
    io.stopTurret();
  }
}
