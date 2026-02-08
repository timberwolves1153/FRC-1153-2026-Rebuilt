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
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {

  private TalonFX deployMotor = new TalonFX(41, "rio");
  private TalonFX collectMotor = new TalonFX(42, "rio");
  private VoltageOut voltageRequest;
  private MotionMagicVoltage positionRequest;
  private final TrapezoidProfile trapezoidProfile;

  // ========

  // all of these private final vars are in order to get real-time data from the motors(safety)

  private final StatusSignal<Current> leaderCurrentValue = deployMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> leaderAppliedVolts = deployMotor.getMotorVoltage();
  private final StatusSignal<Angle> leaderPosition = deployMotor.getPosition();
  private final StatusSignal<Temperature> leaderTemp = deployMotor.getDeviceTemp();

  private final StatusSignal<Current> followerCurrentValue = collectMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> followerAppliedVolts = collectMotor.getMotorVoltage();
  private final StatusSignal<Angle> followerPosition = collectMotor.getPosition();
  private final StatusSignal<Temperature> followerTemp = collectMotor.getDeviceTemp();

  //   public final double gearRatio = 7.1429;
  //   public final double pitchDiameter = 1.751;

  public IntakeIOTalonFX() {
    voltageRequest = new VoltageOut(0);
    positionRequest = new MotionMagicVoltage(0).withSlot(0);
    deployMotor.setPosition(0);

    trapezoidProfile =
        new TrapezoidProfile( // Units are rotations per second & rotations per seocond^2
            new TrapezoidProfile.Constraints(80, 160));

    config();
  }

  // motor config ||

  public void config() {
    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    deployMotor.getConfigurator().apply(config);
    collectMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50,
        leaderCurrentValue,
        leaderAppliedVolts,
        leaderPosition,
        leaderTemp,
        followerAppliedVolts,
        followerCurrentValue,
        followerPosition,
        followerTemp);

    deployMotor.optimizeBusUtilization();
    collectMotor.optimizeBusUtilization();
  }

  @Override
  public void setDeployVoltage(double volts) {
    deployMotor.setVoltage(volts);
  }

  @Override
  public void setCollectVoltage(double volts) {
    collectMotor.setVoltage(volts);
  }

  // @Override
  // public void stopDeploy() {
  //   deployMotor.setControl(voltageRequest.withOutput(0));
  // }

  // @Override
  // public void stopCollect() {
  //   collectMotor.setControl(voltageRequest.withOutput(0));
  // }

  @Override
  public void resetDeployEncoder() {
    deployMotor.setPosition(0);
  }

  @Override
  public void resetCollectEncoder() {
    collectMotor.setPosition(0);
  }
}
