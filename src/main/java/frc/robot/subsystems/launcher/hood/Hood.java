package frc.robot.subsystems.launcher.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public enum Position {
    HOMED(0),
    MIN(-.01),
    MAX(-1.9);

    private double rotations;

    private Position(double rotations) {
      this.rotations = rotations;
    }

    public double rotations() {
      return rotations;
    }
  }

  public Hood(HoodIO hoodIO) {
    this.hoodIO = hoodIO;
  }

  @Override
  public void periodic() {
    hoodIO.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }

  public void homeHood() {
    hoodIO.homeHood();
  }

  public void setVoltageHood(double volts) {
    hoodIO.setVoltageHood(volts);
  }

  public void setPositionHood(double position) {
    hoodIO.setPositionHood(position);
  }

  public void stopHood() {
    hoodIO.stopHood();
  }

  public boolean isHomed() {
    return inputs.isHomed;
  }
}
