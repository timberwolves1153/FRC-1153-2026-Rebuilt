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
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TurretIOTalonFX implements TurretIO {
  private TalonFX turretMotor = new TalonFX(55, "superstructure");
  private CANcoder encoder = new CANcoder(56, "superstructure");

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
    turretConfig.CurrentLimits.SupplyCurrentLimit = 25.0;
    turretConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoderConfig.MagnetSensor.MagnetOffset =
        Units.degreesToRotations(294.4 - 25 - 0.65); // -294.2); // subject to change

    var slot0Configs = turretConfig.Slot0;
    slot0Configs.kS = 0.3;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 130;
    slot0Configs.kI = 0;
    slot0Configs.kD = 1;

    encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.99;

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 7.5;
    turretConfig.MotionMagic.MotionMagicAcceleration = 7.5;

    turretConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.SensorToMechanismRatio = 0.933;
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

    SmartDashboard.putNumber(
        "Turret Encoder Position",
        Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble()) * 1.0718);

    SmartDashboard.putNumber(
        "Turret Encoder Position Raw",
        Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble()));

    SmartDashboard.putNumber(
        "Error",
        (Units.rotationsToDegrees(encoder.getPosition().getValueAsDouble()) * 1.0718)
            - SmartDashboard.getNumber("SOTM TurretAngle", 0));
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
