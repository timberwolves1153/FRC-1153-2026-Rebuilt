package frc.robot.subsystems.launcher;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  // private PIDController launcherPID;
  // private SimpleMotorFeedforward launcherFF;

  public Launcher(LauncherIO launcherIO) {
    io = launcherIO;

    //  launcherPID = new PIDController(0.1, 0, 0);
    //  launcherFF = new SimpleMotorFeedforward(0, 0.0075); //TODO: Tune
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
    Logger.processInputs("Launcher", inputs);
  }

  /* Flywheel */

  public void setVoltageLeader(double volts) {
    io.setVoltageLeader(volts);
  }

  public void setVoltageFollower(double volts) {
    io.setVoltageFollower(volts);
  }

  public void stopLauncher() {
    io.stopLauncher();
  }

  /* Hood */

  // public void setPositionHood(double hoodDegrees) {
  //   io.setPositionHood(hoodDegrees);
  // }

  /* Turret */

  public void setTurretPositionRad(double radians) {
    io.setTurretPositionRad(radians);
  }

  public void setTurretPositionRotaions(double rotations) {
    io.setTurretPositionRotaions(rotations);
  }
}

/*
 *  IDS [delete soon]
 *
 * Flywheel: 64 (lead), 65 (follower), 66 (encoder)
 *
 * Hood: 62 (motor), 63? (encoder)
 *
 * Turret: 60 (motor), 61 (encoder)
 *
 */
