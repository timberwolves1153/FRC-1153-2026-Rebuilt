package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class IndexerIOTalonFX implements IndexerIO {

  //     private SparkMax indexerMotor;
  //     private SparkMaxConfig indexerConfig;
  // --- Constants ---
  // Update these if IDs are swapped
  private static final int SPIN_MOTOR_ID = 51;
  private static final int FEED_MOTOR_ID = 62;

  // Speeds (0.0 to 1.0)
  private static final double INDEX_SPEED = 0.5;
  private static final double FEED_SPEED = 0.8;

  // --- Hardware ---
  // Krakens controlled by TalonFX class in Phoenix 6
  private final TalonFX spinMotor = new TalonFX(SPIN_MOTOR_ID);
  private final TalonFX feedMotor = new TalonFX(FEED_MOTOR_ID);

  // --- Inputs to log ---
  private final StatusSignal<Voltage> spinAppliedVoltage = spinMotor.getMotorVoltage();
  private final StatusSignal<Current> spinCurrent = spinMotor.getSupplyCurrent();

  private final StatusSignal<Voltage> feedAppliedVoltage = feedMotor.getMotorVoltage();

  private final StatusSignal<Current> feedCurrent = feedMotor.getSupplyCurrent();

  public IndexerIOTalonFX() {
    configMotors();
  }

  private void configMotors() {
    TalonFXConfiguration spinConfig = new TalonFXConfiguration();
    TalonFXConfiguration feedConfig = new TalonFXConfiguration();

    spinMotor.getConfigurator().apply(spinConfig);
    feedMotor.getConfigurator().apply(feedConfig);

    spinMotor.optimizeBusUtilization();
    feedMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IndexerIOInputs indexerInputs) {
    BaseStatusSignal.refreshAll(spinAppliedVoltage, spinCurrent, feedAppliedVoltage, feedCurrent);

    indexerInputs.spinAppliedVolts = spinAppliedVoltage.getValueAsDouble();
    indexerInputs.spinCurrentAmps = spinCurrent.getValueAsDouble();
    indexerInputs.feedAppliedVolts = feedAppliedVoltage.getValueAsDouble();
    indexerInputs.feedCurrentAmps = feedCurrent.getValueAsDouble();
  }

  /** Runs the Spin motor to serialize game pieces. */
  @Override
  public void runSpin(double volts) {
    spinMotor.setVoltage(volts);
  }

  /** Runs the Feed motor to push piece into shooter. */
  @Override
  public void runFeeder(double volts) {
    feedMotor.setVoltage(volts);
  }

  /** Stops both motors. */
  @Override
  public void stopAll() {
    stopSpin();
    stopFeeder();
  }

  @Override
  public void stopSpin() {
    spinMotor.setVoltage(0);
  }

  @Override
  public void stopFeeder() {
    feedMotor.setVoltage(0);
  }
}
