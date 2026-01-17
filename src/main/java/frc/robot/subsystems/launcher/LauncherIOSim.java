// package frc.robot.subsystems.launcher;

// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// public class LauncherIOSim implements LauncherIO {
//     private final DCMotorSim simLeader;
//     private final DCMotorSim simFollower;

//     private double appliedVoltsLeader = 0.0;

//     public LauncherIOSim() {
//         simLeader =
//             new DCMotorSim(
//                 LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
// DCMotor.getKrakenX60(1));

//         simFollower =
//             new DCMotorSim(
//                 LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(1), 1, 1),
// DCMotor.getKrakenX60(1));
//     }

//     @Override
//     public void updateInputs(LauncherIOInputs inputs) {
//         inputs.appliedVoltsLeader = simLeader.getInputVoltage()
//     }

// }
