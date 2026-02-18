package frc.robot.subsystems.launcher.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  public final Transform2d turretOffset;

  public Turret(TurretIO turretIO) {
    io = turretIO;

    turretOffset =
        new Transform2d(
            Units.inchesToMeters(-4.75), Units.inchesToMeters(-4.125), new Rotation2d());

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        break;

      case SIM:
        break;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
  }

  public void setPositionTurret(double rotations) {
    io.setPositionTurret(rotations);
  }

  public void setVoltageTurret(double volts) {
    io.setVoltageTurret(volts);
  }

  public void stopTurret() {
    io.stopTurret();
  }
}
