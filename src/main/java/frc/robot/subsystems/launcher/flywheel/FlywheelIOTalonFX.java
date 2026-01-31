package frc.robot.subsystems.launcher.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final TalonFX leadMotor = new TalonFX(64, "rio");
  private final TalonFX followerMotor = new TalonFX(65, "rio");

  private VoltageOut voltageRequest = new VoltageOut(0);
  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private TalonFXConfiguration launcherConfig;

  private final StatusSignal<Voltage> leadAppliedVoltage = leadMotor.getMotorVoltage();
  private final StatusSignal<Voltage> followerAppliedVoltage = followerMotor.getMotorVoltage();

  private final StatusSignal<AngularVelocity> leadVelocity = leadMotor.getVelocity();
  private final StatusSignal<AngularVelocity> FollowerVelocity = followerMotor.getVelocity();

  private final StatusSignal<Current> leadCurrent = leadMotor.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = followerMotor.getSupplyCurrent();

  private final StatusSignal<Temperature> leadTemp = leadMotor.getDeviceTemp();
  private final StatusSignal<Temperature> followerTemp = followerMotor.getDeviceTemp();

  public FlywheelIOTalonFX() {
    launcherConfig = new TalonFXConfiguration();

    configMotors();
  }

  public void configMotors() {
    launcherConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    launcherConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    launcherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Configs = launcherConfig.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    leadMotor.getConfigurator().apply(launcherConfig);
    followerMotor.getConfigurator().apply(launcherConfig);

    followerMotor.setControl(new Follower(63, MotorAlignmentValue.Opposed));

    leadMotor.optimizeBusUtilization();
    followerMotor.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leadAppliedVoltage,
        leadVelocity,
        leadCurrent,
        leadTemp,
        followerAppliedVoltage,
        FollowerVelocity,
        followerCurrent,
        followerTemp);
  }

  @Override
  public void updateInputs(FlywheelIOInputs flywheelInputs) {
    BaseStatusSignal.refreshAll(
        leadAppliedVoltage,
        leadVelocity,
        leadCurrent,
        leadTemp,
        followerAppliedVoltage,
        FollowerVelocity,
        followerCurrent,
        followerTemp);

    flywheelInputs.leadAppliedVoltage = leadAppliedVoltage.getValueAsDouble();
    flywheelInputs.leadCurrent = leadCurrent.getValueAsDouble();
    flywheelInputs.leadVelocity = leadVelocity.getValueAsDouble();
    flywheelInputs.followerCurrent = followerCurrent.getValueAsDouble();

    flywheelInputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    flywheelInputs.followerVelocity = FollowerVelocity.getValueAsDouble();
    flywheelInputs.leadTemp = leadTemp.getValueAsDouble();
    flywheelInputs.followerTemp = followerTemp.getValueAsDouble();
  }

  @Override
  public void setVoltageLeader(double volts) {
    leadMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVoltageFollower(double volts) {
    followerMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setVelocityLeader(double velocity) {
    leadMotor.setControl(velocityVoltage.withVelocity(velocity));
  }

  @Override
  public void setVelocityFollower(double velocity) {
    followerMotor.setControl(velocityVoltage.withVelocity(velocity));
  }

  @Override
  public void stopFlywheel() {
    leadMotor.setControl(voltageRequest.withOutput(0));
    followerMotor.setControl(voltageRequest.withOutput(0));
  }
}
