package frc.robot.subsystems.launcher.hood;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.interpolation.InterpolatingDouble;
import frc.robot.interpolation.LauncherTable;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO hoodIO;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private static final LauncherTable launcherTable = new LauncherTable();

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

  public Command setPositionHoodCommand(Supplier<Pose2d> robotPose) {
    return Commands.run(
        () ->
            setPositionHood(
                LauncherTable.hoodMap.getInterpolated(
                        new InterpolatingDouble(
                            FieldConstants.getDistanceToHubCenter(robotPose.get())))
                    .value),
        this);
  }
}
