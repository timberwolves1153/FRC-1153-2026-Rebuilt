// IntakeIOTalonFX

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX deployMotor = new TalonFX(41, "superstructure");
  private TalonFX collectMotor = new TalonFX(42, "superstructure");

  private VoltageOut voltageRequest;
  private MotionMagicVoltage positionRequest;

  private TalonFXConfiguration deployMotorConfig;
  private TalonFXConfiguration collectorMotorConfig;

  private final StatusSignal<Current> collectorMotorCurrent = collectMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> collectorMotorAppliedVolts = collectMotor.getMotorVoltage();
  private final StatusSignal<Temperature> collectorMotorTemp = collectMotor.getDeviceTemp();

  private final StatusSignal<Current> deployMotorCurrent = deployMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> deployMotorAppliedVolts = deployMotor.getMotorVoltage();
  private final StatusSignal<Temperature> deployMotorTemp = deployMotor.getDeviceTemp();

  public double error;
  public double currentPosition = collectMotor.getPosition().getValueAsDouble();

  public IntakeIOTalonFX() {
    voltageRequest = new VoltageOut(0);
    positionRequest = new MotionMagicVoltage(0).withSlot(0);
    // deployMotor.setPosition(0);

    deployMotorConfig = new TalonFXConfiguration();
    collectorMotorConfig = new TalonFXConfiguration();

    // deployMotor.setPosition(0);

    config();
  }

  public void config() {

    deployMotorConfig.CurrentLimits.SupplyCurrentLimit = 15.0;
    deployMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    deployMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    collectorMotorConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    collectorMotorConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    collectorMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    collectorMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    var slot0Configs = deployMotorConfig.Slot0;
    slot0Configs.kS = 0;
    slot0Configs.kV = 0;
    slot0Configs.kA = 0;
    slot0Configs.kP = 25;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.12;

    deployMotorConfig.MotionMagic.MotionMagicCruiseVelocity = 24;
    deployMotorConfig.MotionMagic.MotionMagicAcceleration = 48;

    deployMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 1;

    deployMotor.getConfigurator().apply(deployMotorConfig);
    collectMotor.getConfigurator().apply(collectorMotorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        collectorMotorCurrent,
        collectorMotorAppliedVolts,
        collectorMotorTemp,
        deployMotorCurrent,
        deployMotorAppliedVolts,
        deployMotorTemp);

    deployMotor.optimizeBusUtilization();
    collectMotor.optimizeBusUtilization();

    SmartDashboard.putNumber(
        "Intake encoder position", deployMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    SmartDashboard.putNumber(
        "Intake encoder position", deployMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setDeployVoltage(double volts) {
    deployMotor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setCollectVoltage(double volts) {
    collectMotor.setControl(voltageRequest.withOutput(volts));
  }

  // @Override
  // public void resetDeployEncoder() {
  //   deployMotor.setPosition(0);
  // }

  @Override
  public void setPositionIntake(double rotations) {
    // error = rotations - currentPosition;

    deployMotor.setControl(positionRequest.withPosition(rotations));

    // if (Math.abs(error) < 5) {
    //   setDeployVoltage(0);
    // }
  }

  // public void intakeFuel(double rotations, double volts) {
  //   setPositionIntake(rotations);
  //   setCollectVoltage(volts);
  // }
}
