// package frc.robot.subsystems.indexer;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.system.plant.DCMotor;
// import edu.wpi.first.math.system.plant.LinearSystemId;
// import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// public class IndexerIOSim implements IndexerIO {
//     private DCMotorSim sim = new DCMotorSim(
//         LinearSystemId.createDCMotorSystem(
//             DCMotor.getNEO(1),
//             1,
//             1
//         ), DCMotor.getNEO(1)
//     );

//     private double appliedVolts = 0.0;

//     @Override
//     public void updateInputs(IndexerIOInputs inputs) {
//         sim.setInputVoltage(appliedVolts);
//         sim.update(0.02);

//         inputs.appliedVolts = appliedVolts;
//         inputs.currentAmps = sim.getCurrentDrawAmps();
//     }

//     @Override
//     public void setVoltage(double volts) {
//         appliedVolts = MathUtil.clamp(volts, 12, -12);
//     }
// }