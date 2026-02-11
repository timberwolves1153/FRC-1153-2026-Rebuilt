package frc.robot.subsystems.launcher.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HoodIOTalonFX implements HoodIO {
  private TalonFX hoodMotor = new TalonFX(57, "rio");

  private VoltageOut voltageRequest = new VoltageOut(0);
  private MotionMagicVoltage positionRequest = new MotionMagicVoltage(0).withSlot(0);

  private TalonFXConfiguration hoodConfig;

  private final StatusSignal<Voltage> hoodAppliedVoltage = hoodMotor.getMotorVoltage();
  private final StatusSignal<Current> hoodCurrent = hoodMotor.getSupplyCurrent();
  private final StatusSignal<Angle> hoodPosition = hoodMotor.getPosition();
  private final StatusSignal<Temperature> hoodTemp = hoodMotor.getDeviceTemp();

  public HoodIOTalonFX() {
    hoodConfig = new TalonFXConfiguration();

    configMotors();
  }

  public void configMotors() {
    hoodConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    hoodConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    var slot0Configs = hoodConfig.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 50;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    hoodConfig.MotionMagic.MotionMagicCruiseVelocity = 5;
    hoodConfig.MotionMagic.MotionMagicAcceleration = 5;

    hoodMotor.getConfigurator().apply(hoodConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, hoodAppliedVoltage, hoodCurrent, hoodPosition, hoodTemp);
  }

  @Override
  public void updateInputs(HoodIOInputs hoodInputs) {
    BaseStatusSignal.refreshAll(hoodAppliedVoltage, hoodCurrent, hoodPosition, hoodTemp);

    hoodInputs.hoodAppliedVoltage = hoodAppliedVoltage.getValueAsDouble();
    hoodInputs.hoodCurrent = hoodCurrent.getValueAsDouble();
    hoodInputs.hoodPosition = hoodPosition.getValueAsDouble();
    hoodInputs.hoodTemp = hoodTemp.getValueAsDouble();

    SmartDashboard.putNumber("Hood encoder position", hoodMotor.getPosition().getValueAsDouble());
  }

  public void setVoltageHood(double volts) {
    hoodMotor.setControl(voltageRequest.withOutput(volts));
  }

  public void setPositionHood(double position) {
    hoodMotor.setControl(positionRequest.withPosition(position));
  }

  public void stopHood() {
    hoodMotor.setControl(voltageRequest.withOutput(0));
  }
}
