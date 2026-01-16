// package frc.robot.subsystems.indexer;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// public class IndexerIOTalonFX implements IndexerIO {
//     private SparkMax indexerMotor;
//     private SparkMaxConfig indexerConfig;

//     public IndexerIOTalonFX() {
//         //indexerMotor = new TalonFX(), MotorType.kBrushless); //TODO: ID
//         new TalonFX()
//        // indexerConfig = new SparkMaxConfig();
//     }

//     public void configMotors() {
//         indexerConfig
//             .smartCurrentLimit(40)
//             .idleMode(IdleMode.kBrake);

//         indexerMotor.clearFaults();
//         indexerMotor.configure(indexerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//     }

//     @Override
//     public void updateInputs(IndexerIOInputs inputs) {
//         inputs.appliedVolts = indexerMotor.getAppliedOutput() * indexerMotor.getBusVoltage();
//         inputs.currentAmps = indexerMotor.getOutputCurrent();
//     }

//     @Override
//     public void setVoltage(double volts) {
//         indexerMotor.setVoltage(volts);
//     }

//     @Override
//     public void stop() {
//         indexerMotor.setVoltage(0);
//     }

// }