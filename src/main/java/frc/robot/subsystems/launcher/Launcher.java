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
    io = new LauncherIOTalonFX();

    //  launcherPID = new PIDController(0.1, 0, 0);
    //  launcherFF = new SimpleMotorFeedforward(0, 0.0075);
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

  public void runVoltsLeader(double volts) {
    io.runVoltsLeader(volts);
  }

  public void runVoltsFollower(double volts) {
    io.runVoltsFollower(volts);
  }

  public void stop() {
    io.stopLeader();
    io.stopFollower();
  }
}
