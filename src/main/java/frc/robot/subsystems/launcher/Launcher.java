// package frc.robot.subsystems.launcher;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;
// import org.littletonrobotics.junction.Logger;

// public class Launcher extends SubsystemBase {
//   private final LauncherIO io;
//   private final LauncherIOInputsAutoLogged inputs = new LauncherIOInputsAutoLogged();

//   private PIDController launcherPID;
//   private SimpleMotorFeedforward launcherFF;

//   private PIDController turretPID;

//   public Launcher(LauncherIO launcherIO) {
//     io = launcherIO;

//     switch (Constants.currentMode) {
//       case REAL:
//       case REPLAY:
//         break;

//       case SIM:
//         break;
//     }

//     launcherPID = new PIDController(0.1, 0, 0);
//     launcherFF = new SimpleMotorFeedforward(0, 0.0075); // TODO: Tune

//     turretPID = new PIDController(0, 0, 0);
//   }

//   @Override
//   public void periodic() {
//     io.updateInputs(inputs);
//     Logger.processInputs("Launcher", inputs);
//   }

//   /* Flywheel */

//   public void setVoltageLeader(double volts) {
//     io.setVoltageLeader(volts);
//   }

//   public void setVoltageFollower(double volts) {
//     io.setVoltageFollower(volts);
//   }

//   public void stopLauncher() {
//     io.stopLauncher();
//   }

//   /* Hood */

//   // public void setPositionHood(double hoodDegrees) {
//   //   io.setPositionHood(hoodDegrees);
//   // }

//   /* Turret */

//   public void setPositionTurretRad(double turretSetpointRad) {
//     io.setPositionTurretRad(turretSetpointRad);
//   }

//   public void setTurretPosition(double rotations) {
//     io.setTurretPosition(rotations);
//   }

//   public void setVoltageTurret(double volts) {
//     io.setVoltageTurret(volts);
//   }

//   public void stopTurret() {
//     io.stopTurret();
//   }
// }

// /*
//  *  IDS [delete soon]
//  *
//  * Flywheel: 64 (lead), 65 (follower), 66 (encoder)
//  *
//  * Hood: 62 (motor), 63? (encoder)
//  *
//  * Turret: 60 (motor), 61 (encoder)
//  *
//  */
