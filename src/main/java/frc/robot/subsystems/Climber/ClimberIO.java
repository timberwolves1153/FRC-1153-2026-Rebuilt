package frc.robot.subsystems.Climber;

import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  public static class ClimberInputs {
    public double climberCurrentAmps = 0.0;
    public double heightInches = 0.0;
    public double leaderRotations = 0;
    public double getAppliedVolts = 0.0;
    public double tempCelsius = 0.0;
    public boolean isSwitchTriggered = false;
    public double goal = 0;
  }

  public default void updateInputs(ClimberInputs climberInputs) {}

  public default void setVoltage(Voltage volts) {}

  public default void resetClimberEncoder() {}

  public default void setTargetHeight(double rotations) {}

  public default void stop() {}
}
