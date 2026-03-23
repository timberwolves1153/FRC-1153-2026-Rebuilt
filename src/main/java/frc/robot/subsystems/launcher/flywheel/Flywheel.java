package frc.robot.subsystems.launcher.flywheel;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.interpolation.InterpolatingDouble;
import frc.robot.interpolation.LauncherTable;
import java.util.function.Supplier;
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

  public void setVelocityLeader(double velocity) {
    io.setVelocityLeader(velocity);
  }

  public void setVelocityManual() {
    double manualVelocityInput = SmartDashboard.getNumber("Flywheel RPS Manual Input", 0);

    io.setVelocityLeader(manualVelocityInput);
  }

  public void stopFlywheel() {
    io.stopFlywheel();
  }

  public double getFlywheelCurrentRPS() {
    return io.getCurrentRPS();
  }

  public Command setVelocityHub(Supplier<Pose2d> robotPose) {
    return Commands.run(
        () ->
            setVelocityLeader(
                LauncherTable.flywheelShootingMap.getInterpolated(
                        new InterpolatingDouble(
                            FieldConstants.getDistanceToHubCenter(robotPose.get())))
                    .value),
        this);
  }

  public Command setVelocityPassing(Supplier<Pose2d> robotPose) {
    return Commands.run(
        () ->
            setVelocityLeader(
                LauncherTable.flywheelPassingMap.getInterpolated(
                        new InterpolatingDouble(
                            FieldConstants.getDistanceToHubCenter(robotPose.get())))
                    .value),
        this);
  }
}
