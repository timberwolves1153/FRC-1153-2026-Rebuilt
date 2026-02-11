package frc.robot.subsystems.launcher.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO hoodIO) {
    io = hoodIO;

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
    Logger.processInputs("Hood", inputs);
  }

  public void setVoltageHood(double volts) {
    io.setVoltageHood(volts);
  }

  public void setPositionHood(double position) {
    io.setPositionHood(position);
  }

  public void stopHood() {
    io.stopHood();
  }
}
