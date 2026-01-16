package frc.robot.subsystems.launcher;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
  private final LauncherIO io;
  private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

  private PIDController launcherPID;
  private SimpleMotorFeedforward launcherFF;

  public Launcher() {
    io = new LauncherIOTalonFX();

    launcherPID = new PIDController(0.1, 0, 0);
    launcherFF = new SimpleMotorFeedforward(0, 0.0075);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
  }

  public void runDutyCycle(double output) {
    io.runDutyCycle(output);
  }
}
