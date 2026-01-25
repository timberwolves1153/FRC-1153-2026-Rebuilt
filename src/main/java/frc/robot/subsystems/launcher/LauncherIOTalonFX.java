package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class LauncherIOTalonFX implements LauncherIO {
  /* Flywheel */

  private final TalonFX leadLauncherMotor = new TalonFX(64, "rio");
  private final TalonFX followerLauncherMotor = new TalonFX(65, "rio");

  private TalonFXConfiguration launcherConfig;
  private CANcoderConfiguration flywheelEncoder;

  private final StatusSignal<Voltage> leadAppliedVoltage = leadLauncherMotor.getMotorVoltage();
  private final StatusSignal<Voltage> followerAppliedVoltage =
      followerLauncherMotor.getMotorVoltage();

  private final StatusSignal<Current> leadCurrent = leadLauncherMotor.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = followerLauncherMotor.getSupplyCurrent();

  private final StatusSignal<Temperature> leadTemp = leadLauncherMotor.getDeviceTemp();
  private final StatusSignal<Temperature> followerTemp = followerLauncherMotor.getDeviceTemp();

  /* Hood */

  private TalonFX hoodMotor = new TalonFX(61, "rio");

  private TalonFXConfiguration hoodConfig;

  /* Turret */

  private TalonFX turretMotor = new TalonFX(60, "rio");
  private CANcoder turretEncoder = new CANcoder(61, "rio");

  private VoltageOut turretVoltageRequest = new VoltageOut(0);
  private MotionMagicVoltage turretPositionRequest = new MotionMagicVoltage(0);

  private TalonFXConfiguration turretConfig;
  private CANcoderConfiguration turretEncoderConfig;

  private final StatusSignal<Voltage> turretAppliedVoltage = turretMotor.getMotorVoltage();
  private final StatusSignal<Current> turretCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<Angle> turretPosition = turretMotor.getPosition();
  private final StatusSignal<Temperature> turretTemp = turretMotor.getDeviceTemp();

  public LauncherIOTalonFX() {
    launcherConfig = new TalonFXConfiguration();
    flywheelEncoder = new CANcoderConfiguration();

    turretConfig = new TalonFXConfiguration();
    turretEncoderConfig = new CANcoderConfiguration();

    configMotors();
  }

  public void configMotors() {
    /* Flywheel */

    launcherConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    launcherConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    launcherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leadLauncherMotor.getConfigurator().apply(launcherConfig);
    followerLauncherMotor.getConfigurator().apply(launcherConfig);

    followerLauncherMotor.setControl(new Follower(63, MotorAlignmentValue.Opposed));

    leadLauncherMotor.optimizeBusUtilization();
    followerLauncherMotor.optimizeBusUtilization();

    /* Hood */

    hoodConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    hoodConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    hoodMotor.getConfigurator().apply(hoodConfig);

    /* Turret */

    turretConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    turretConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    turretConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turretConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    turretEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0; // TODO: Set

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 25;
    turretConfig.MotionMagic.MotionMagicAcceleration = 10; // TODO: Set all

    turretConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.SensorToMechanismRatio = 1;
    turretConfig.Feedback.RotorToSensorRatio = 6;

    turretMotor.getConfigurator().apply(turretConfig);
    turretEncoder.getConfigurator().apply(turretEncoderConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leadCurrent,
        followerCurrent,
        leadAppliedVoltage,
        followerAppliedVoltage,
        leadTemp,
        followerTemp,
        turretAppliedVoltage,
        turretCurrent,
        turretPosition,
        turretTemp);
  }

  @Override
  public void updateInputs(LauncherIOInputs launcherInputs) {

    BaseStatusSignal.refreshAll(
        leadCurrent,
        followerCurrent,
        leadAppliedVoltage,
        followerAppliedVoltage,
        leadTemp,
        followerTemp,
        turretAppliedVoltage,
        turretCurrent,
        turretPosition,
        turretTemp);

    /* Flywheel */

    launcherInputs.leadCurrent = leadCurrent.getValueAsDouble();
    launcherInputs.followerCurrent = followerCurrent.getValueAsDouble();
    launcherInputs.leadAppliedVoltage = leadAppliedVoltage.getValueAsDouble();
    launcherInputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    launcherInputs.leadTemp = leadTemp.getValueAsDouble();
    launcherInputs.followerTemp = followerTemp.getValueAsDouble();

    /* Hood */

    /* Turret */

    launcherInputs.turretAppliedVoltage = turretAppliedVoltage.getValueAsDouble();
    launcherInputs.turretCurrent = turretCurrent.getValueAsDouble();
    launcherInputs.turretPositionRad = turretCurrent.getValueAsDouble();
    launcherInputs.turretTemp = turretTemp.getValueAsDouble();
  }

  /* Flywheel */

  @Override
  public void setVoltageLeader(double volts) {
    leadLauncherMotor.setVoltage(volts);
  }

  @Override
  public void setVoltageFollower(double volts) {
    followerLauncherMotor.setVoltage(volts);
  }

  @Override
  public void stopLauncher() {
    leadLauncherMotor.setVoltage(0);
    followerLauncherMotor.setVoltage(0);
  }

  /* Hood */

  /* Turret */

  @Override
  public void setTurretPosition(double rotations) {
    turretMotor.setControl(turretPositionRequest.withPosition(rotations));
  }

  @Override
  public void setVoltageTurret(double volts) {
    turretMotor.setControl(turretVoltageRequest.withOutput(volts));
  }

  @Override
  public void stopTurret() {
    turretMotor.setControl(turretVoltageRequest.withOutput(0));
  }
}
