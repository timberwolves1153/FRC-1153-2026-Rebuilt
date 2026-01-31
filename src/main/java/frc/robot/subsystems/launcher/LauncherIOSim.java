// package frc.robot.subsystems.launcher;

// import com.ctre.phoenix6.hardware.CANcoder;
// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class LauncherIOSim implements LauncherIO {

//   /* Flywheel */

//   private final FlywheelSim simLeader;
//   private final FlywheelSim simFollower;

//   private double appliedVoltsLeader = 0.0;
//   private double appliedVoltsFollower = 0.0;

//   /* Hood */

//   private final DCMotorSim simHood;
//   private CANcoder turretEncoder;

//   private double appliedVoltsHood = 0.0;

//   /* Turret */

//   private final DCMotorSim simTurret;

//   // private final TalonFX turretKraken;

//   // private final CANcoder turretAbsoluteEncoder;

//   public LauncherIOSim() {

//     /* Flywheel */

//     simLeader =
//         new FlywheelSim(
//             LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
//             DCMotor.getKrakenX60(1));

//     simFollower =
//         new FlywheelSim(
//             LinearSystemId.createFlywheelSystem(DCMotor.getKrakenX60(1), 1, 1),
//             DCMotor.getKrakenX60(1));

//     /* Hood */

//     simHood =
//         new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 1, 1),
//             DCMotor.getKrakenX44(1));

//     /* Turret */

//     simTurret =
//         new DCMotorSim(
//             LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX44(1), 1, 1),
//             DCMotor.getKrakenX44(1));

//     turretEncoder = new CANcoder(61);
//   }

//   @Override
//   public void updateInputs(LauncherIOInputs inputs) {
//     /*Flywheel */

//     if (DriverStation.isDisabled()) {
//       simLeader.setInputVoltage(0);
//       simFollower.setInputVoltage(0);
//     }

//     inputs.leadAppliedVoltage = simLeader.getInputVoltage();
//     inputs.leadCurrent = simLeader.getCurrentDrawAmps();

//     inputs.followerAppliedVoltage = simFollower.getInputVoltage();
//     inputs.followerCurrent = simFollower.getCurrentDrawAmps();

//     /* Hood */

//     inputs.hoodAppliedVoltage = simHood.getInputVoltage();
//     inputs.hoodCurrent = simHood.getCurrentDrawAmps();

//     /* Turret */

//     inputs.turretAppliedVoltage = simTurret.getInputVoltage();
//     inputs.turretCurrent = simTurret.getCurrentDrawAmps();
//     inputs.turretPositionRad = simTurret.getAngularPositionRad();

//     getAbsolutePosition();

//     // TalonFXSimState turretFXSim = turretKraken.getSimState();
//     // turretFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());
//     // Voltage turretVoltage = turretFXSim.getMotorVoltageMeasure();

//   }

//   /* Flywheel */

//   @Override
//   public void setVoltageLeader(double volts) {
//     appliedVoltsLeader = MathUtil.clamp(volts, -12.0, 12.0);
//     SmartDashboard.putNumber("Flyhweel volts", volts);
//     simLeader.setInputVoltage(appliedVoltsLeader);
//   }

//   @Override
//   public void setVoltageFollower(double volts) {
//     appliedVoltsFollower = MathUtil.clamp(volts, -12.0, 12.0);
//     simFollower.setInputVoltage(appliedVoltsFollower);
//   }

//   @Override
//   public void stopLauncher() {
//     simLeader.setInputVoltage(0);
//     simFollower.setInputVoltage(0);
//   }

//   /* Hood */

//   /* Turret */

//   @Override
//   public void setPositionTurretRad(double turretSetpointRad) {
//     simTurret.setAngle(turretSetpointRad);
//   }

//   public double getAbsolutePosition() {
//     return turretEncoder.getAbsolutePosition().getValueAsDouble();
//   }
// }
