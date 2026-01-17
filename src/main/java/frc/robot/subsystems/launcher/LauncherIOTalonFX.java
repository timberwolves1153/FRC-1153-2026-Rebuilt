package frc.robot.subsystems.launcher;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class LauncherIOTalonFX implements LauncherIO {
  private final TalonFX leadLauncherMotor = new TalonFX(63, "rio");
  private final TalonFX followerLauncherMotor = new TalonFX(64, "rio");
  private final CANcoder encoder;
  private VoltageOut voltageRequest;

  private TalonFXConfiguration launcherConfig;
  private CANcoderConfiguration encoderConfig;

  //   private DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  //   private MotionMagicDutyCycle motionMagic = new MotionMagicDutyCycle(0.0).withSlot(0);

  private final StatusSignal<Voltage> leadAppliedVoltage = leadLauncherMotor.getMotorVoltage();
  private final StatusSignal<Voltage> followerAppliedVoltage =
      followerLauncherMotor.getMotorVoltage();

  private final StatusSignal<Current> leadCurrent = leadLauncherMotor.getSupplyCurrent();
  private final StatusSignal<Current> followerCurrent = followerLauncherMotor.getSupplyCurrent();

  private final StatusSignal<Temperature> leadTemp = leadLauncherMotor.getDeviceTemp();
  private final StatusSignal<Temperature> followerTemp = followerLauncherMotor.getDeviceTemp();

  public LauncherIOTalonFX() {
    encoder = new CANcoder(0, "rio"); //TODO: Set ID, bus
    voltageRequest = new VoltageOut(0);

    encoderConfig = new CANcoderConfiguration();
    // var magnetSensorConfigs = new MagnetSensorConfigs();
    // encoderConfig.withMagnetSensor(magnetSensorConfigs);
    // magnetSensorConfigs.AbsoluteSensorDiscontinuityPoint = 0.05;
    encoder.getConfigurator().apply(encoderConfig);

    configMotors();
  }

  public void configMotors() {
    launcherConfig = new TalonFXConfiguration();
    encoderConfig = new CANcoderConfiguration();

    var slot0Configs = launcherConfig.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 0;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    launcherConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    launcherConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    launcherConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    launcherConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // var motionMagicConfigs = launcherConfig.MotionMagic;
    // motionMagicConfigs.MotionMagicCruiseVelocity = 640;
    // motionMagicConfigs.MotionMagicAcceleration = 320;
    // motionMagicConfigs.MotionMagicJerk = 1600;

    launcherConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
    launcherConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    launcherConfig.Feedback.SensorToMechanismRatio = 0; //TODO: SET

    leadLauncherMotor.getConfigurator().apply(launcherConfig);
    followerLauncherMotor.getConfigurator().apply(launcherConfig);

    followerLauncherMotor.setControl(new Follower(63, MotorAlignmentValue.Opposed));

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leadCurrent,
        followerCurrent,
        leadAppliedVoltage,
        followerAppliedVoltage,
        leadTemp,
        followerTemp);

    leadLauncherMotor.optimizeBusUtilization();
    followerLauncherMotor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(LauncherIOInputs launcherInputs) {

    BaseStatusSignal.refreshAll(
        leadCurrent,
        followerCurrent,
        leadAppliedVoltage,
        followerAppliedVoltage,
        leadTemp,
        followerTemp);

    launcherInputs.leadCurrent = leadCurrent.getValueAsDouble();
    launcherInputs.followerCurrent = followerCurrent.getValueAsDouble();
    launcherInputs.leadAppliedVoltage = leadAppliedVoltage.getValueAsDouble();
    launcherInputs.followerAppliedVoltage = followerAppliedVoltage.getValueAsDouble();
    launcherInputs.leadTemp = leadTemp.getValueAsDouble();
    launcherInputs.followerTemp = followerTemp.getValueAsDouble();
  }

  @Override
  public void runVolts(double volts) {
    leadLauncherMotor.setControl(voltageRequest.withOutput(volts));
  }
}
