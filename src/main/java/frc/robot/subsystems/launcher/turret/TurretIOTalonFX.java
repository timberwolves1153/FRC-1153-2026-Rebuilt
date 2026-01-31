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
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class TurretIOTalonFX implements TurretIO {
  private TalonFX turretMotor = new TalonFX(60, "rio");
  private CANcoder encoder = new CANcoder(61, "rio");

  private VoltageOut voltageRequest = new VoltageOut(0);
  private MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);

  private TalonFXConfiguration turretConfig;
  private CANcoderConfiguration encoderConfig;

  private double TURRET_RADIUS = 4.743; // IN
  private double TURRET_GEAR_RATIO = 30;

  private final StatusSignal<Voltage> turretAppliedVoltage = turretMotor.getMotorVoltage();
  private final StatusSignal<Current> turretCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<Angle> turretPosition = turretMotor.getPosition();
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

    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0; // TODO: Set

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    turretConfig.MotionMagic.MotionMagicAcceleration = 5; // TODO: Set all

    turretConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.SensorToMechanismRatio = 1;
    turretConfig.Feedback.RotorToSensorRatio = 30;

    turretMotor.getConfigurator().apply(turretConfig);
    encoder.getConfigurator().apply(encoderConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        turretAppliedVoltage,
        turretCurrent,
        turretPosition,
        turretTemp
      );
  }

  @Override
  public void updateInputs(TurretIOInputs turretInputs) {
    BaseStatusSignal.refreshAll(
      turretAppliedVoltage,
      turretCurrent,
      turretPosition,
      turretTemp
    );

    turretInputs.turretAppliedVoltage = turretAppliedVoltage.getValueAsDouble();
    turretInputs.turretCurrent = turretCurrent.getValueAsDouble();
    turretInputs.turretPosition = turretPosition.getValueAsDouble();
    turretInputs.turretTemp = turretTemp.getValueAsDouble();
  }

  @Override
  public void setPositionTurret(double rotations) {
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
