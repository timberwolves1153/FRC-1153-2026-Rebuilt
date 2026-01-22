package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
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

  private final TalonFX hoodMotor = new TalonFX(61, "rio");

  private TalonFXConfiguration hoodConfig;

  /* Turret */

  private final TalonFX turretMotor = new TalonFX(60, "rio");
  private final CANcoder turretEncoder = new CANcoder(61, "rio");

  private DutyCycleOut turretDutyCycle = new DutyCycleOut(0.0);
  private MotionMagicDutyCycle turretMotionMagic = new MotionMagicDutyCycle(0.0);

  private double turretSetpointRadians = 0.0;

  private TalonFXConfiguration turretConfig;
  private CANcoderConfiguration turretEncoderConfig;

  private final StatusSignal<Voltage> turretAppliedVoltage = turretMotor.getMotorVoltage();
  private final StatusSignal<Current> turretCurrent = turretMotor.getSupplyCurrent();
  private final StatusSignal<Angle> turretPositionRadians = turretMotor.getPosition();
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

    turretMotor.getConfigurator().apply(turretConfig);

    turretEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.0; // TODO: Set

    turretEncoder.getConfigurator().apply(turretEncoderConfig);

    turretConfig.MotionMagic.MotionMagicCruiseVelocity = 0;
    turretConfig.MotionMagic.MotionMagicAcceleration = 0; // TODO: Set all

    turretConfig.Feedback.FeedbackRemoteSensorID = turretEncoder.getDeviceID();
    turretConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    turretConfig.Feedback.SensorToMechanismRatio = 1;

    var slot0Configs = turretConfig.Slot0;
    slot0Configs.kS = 0.0;
    slot0Configs.kV = 0.0;
    slot0Configs.kA = 0.0;
    slot0Configs.kP = 0.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0; // TODO: Set all

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
        turretPositionRadians,
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
        turretPositionRadians,
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

  // @Override
  // public void setPositionTurret(double turretSetpoint) {
  //   turretSetpoint = Rotation2d.fromRadians(MathUtil.clamp(turretSetpoint.getRadians(), 0, 0));

  //   turretMotor.setControl(turretMotionMagic.withPosition(turretSetpoint.getRotations()));
  // }
}
