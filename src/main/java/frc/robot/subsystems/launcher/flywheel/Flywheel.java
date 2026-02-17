package frc.robot.subsystems.launcher.flywheel;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  // private PIDController flywheelPID;
  // private SimpleMotorFeedforward flywheelFF;

  public Flywheel(FlywheelIO flywheelIO) {
    io = flywheelIO;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        break;

      case SIM:
        break;
    }

    // flywheelPID = new PIDController(0, 0, 0);
    // flywheelFF = new SimpleMotorFeedforward(0, 0); // TODO: Set
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);
  }

  public void setVoltageLeader(double volts) {
    io.setVoltageLeader(volts);
  }

  public void setVoltageFollower(double volts) {
    io.setVoltageFollower(volts);
  }

  public void setVelocityLeader(double velocity) {
    io.setVelocityLeader(velocity);
  }

  public void setVelocityFollower(double velocity) {
    io.setVelocityFollower(velocity);
  }

  public void setVelocityManual() {
    double manualVelocityInput = SmartDashboard.getNumber("Flywheel RPS Manual Input", 0);

    io.setVelocityLeader(manualVelocityInput);
  }

  public void stopFlywheel() {
    io.stopFlywheel();
  }
}
