// IntakeIOTalonFX

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX deployMotor = new TalonFX(41, "rio");
  private TalonFX collectMotor = new TalonFX(42, "rio");
  private CANcoder deployEncoder = new CANcoder(43, "rio");

  private VoltageOut voltageRequest;
  private MotionMagicVoltage positionRequest;

  private TalonFXConfiguration motorConfig;
  private CANcoderConfiguration encoderConfig;

  // ========

  // all of these private final vars are in order to get real-time data from the motors(safety)

  private final StatusSignal<Current> leaderCurrentValue = deployMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> leaderAppliedVolts = deployMotor.getMotorVoltage();
  private final StatusSignal<Angle> leaderPosition = deployEncoder.getPosition();
  private final StatusSignal<Temperature> leaderTemp = deployMotor.getDeviceTemp();

  private final StatusSignal<Current> followerCurrentValue = collectMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> followerAppliedVolts = collectMotor.getMotorVoltage();
  private final StatusSignal<Temperature> followerTemp = collectMotor.getDeviceTemp();

  public IntakeIOTalonFX() {
    // voltageRequest = new VoltageOut(0);
    // positionRequest = new MotionMagicVoltage(0).withSlot(0);
    // deployMotor.setPosition(0);

    motorConfig = new TalonFXConfiguration();
    encoderConfig = new CANcoderConfiguration();

    config();
  }

  public void config() {

    motorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    deployMotor.getConfigurator().apply(motorConfig);
    collectMotor.getConfigurator().apply(motorConfig);
    deployEncoder.getConfigurator().apply(encoderConfig);

    var slot0Configs = motorConfig.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 115;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.375; // TODO: Set

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    motorConfig.MotionMagic.MotionMagicAcceleration = 5;

    motorConfig.Feedback.FeedbackRemoteSensorID = deployEncoder.getDeviceID();
    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    motorConfig.Feedback.SensorToMechanismRatio = 1;
    motorConfig.Feedback.RotorToSensorRatio = 30;

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leaderCurrentValue,
        leaderAppliedVolts,
        leaderPosition,
        leaderTemp,
        followerAppliedVolts,
        followerCurrentValue,
        followerTemp);

    deployMotor.optimizeBusUtilization();
    collectMotor.optimizeBusUtilization();
  }

  @Override
  public void setDeployVoltage(double volts) {
    deployMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setCollectVoltage(double volts) {
    collectMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void resetDeployEncoder() {
    deployMotor.setPosition(0);
  }

  @Override
  public void setPositionIntake(double rotations) {
    deployMotor.setControl(positionRequest.withPosition(rotations));
  }
}
