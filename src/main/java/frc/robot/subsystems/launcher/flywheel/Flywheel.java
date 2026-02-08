// package frc.robot.subsystems.launcher.flywheel;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import org.littletonrobotics.junction.Logger;

// public class Flywheel extends SubsystemBase {
//   private final FlywheelIO io;
//   private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

//   public Flywheel(FlywheelIO flywheelIO) {
//     io = flywheelIO;

//     switch (Constants.currentMode) {
//       case REAL:
//       case REPLAY:
//         break;

//       case SIM:
//         break;
//     }
//   }

//   @Override
//   public void periodic() {
//     io.updateInputs(inputs);
//     Logger.processInputs("Flywheel", inputs);
//   }

//   public void setVoltageLeader(double volts) {
//     io.setVoltageLeader(volts);
//   }

//   public void setVoltageFollower(double volts) {
//     io.setVoltageFollower(volts);
//   }

//   public void setVelocityLeader(double velocity) {
//     io.setVoltageLeader(velocity);
//   }

//   public void setVelocityFollower(double velocity) {
//     io.setVoltageFollower(velocity);
//   }

//   public void stopLauncher() {
//     io.stopFlywheel();
//   }
// }
