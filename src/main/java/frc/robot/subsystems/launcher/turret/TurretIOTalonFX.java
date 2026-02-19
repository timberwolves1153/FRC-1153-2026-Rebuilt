package frc.robot.subsystems.launcher.turret;

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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretIOTalonFX implements TurretIO {
  private TalonFX turretMotor = new TalonFX(55, "rio");
  private CANcoder encoder = new CANcoder(56, "rio");

  private VoltageOut voltageRequest = new VoltageOut(0);
  private MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

  private TalonFXConfiguration turretConfig;
  private CANcoderConfiguration encoderConfig;

  private final StatusSignal<Voltage> turretAppliedVoltage = turretMotor.getMotorVoltage();
  private final StatusSignal<Current> turretCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<Angle> turretPosition = encoder.getPosition();
  private final StatusSignal<Temperature> turretTemp = turretMotor.getDeviceTemp();

  public TurretIOTalonFX() {
    turretConfig = new TalonFXConfiguration();
    encoderConfig = new CANcoderConfiguration();

    configMotors();
  }

  private void configMotors() {
    turretConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Configs = turretConfig.Slot0;
    slot0Configs.kS = 0.3;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 115;
    slot0Configs.kI = 0;
    slot0Configs.kD = 1;

    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = -0.9445;

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 1;
    turretConfig.MotionMagic.MotionMagicAcceleration = 1;

    turretConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.SensorToMechanismRatio = 1.25;
    turretConfig.Feedback.RotorToSensorRatio = 30;

    turretMotor.getConfigurator().apply(turretConfig);
    encoder.getConfigurator().apply(encoderConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, turretAppliedVoltage, turretCurrent, turretPosition, turretTemp);
  }

  @Override
  public void updateInputs(TurretIOInputs turretInputs) {
    BaseStatusSignal.refreshAll(turretAppliedVoltage, turretCurrent, turretPosition, turretTemp);

    turretInputs.turretAppliedVoltage = turretAppliedVoltage.getValueAsDouble();
    turretInputs.turretCurrent = turretCurrent.getValueAsDouble();
    turretInputs.turretPosition = turretPosition.getValueAsDouble();
    turretInputs.turretTemp = turretTemp.getValueAsDouble();

    SmartDashboard.putNumber("Turret Encoder Position", encoder.getPosition().getValueAsDouble());
  }

  @Override
  public void setPositionTurret(double rotations) {
    rotations = MathUtil.clamp(rotations, -0.7553, 0.104);
    turretMotor.setControl(positionRequest.withPosition(rotations));
  }

  @Override
  public void setVoltageTurret(double volts) {
    turretMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stopTurret() {
    turretMotor.setControl(voltageRequest.withOutput(0));
  }
}
