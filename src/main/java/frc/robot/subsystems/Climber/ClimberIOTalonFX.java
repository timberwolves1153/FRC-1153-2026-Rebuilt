package frc.robot.subsystems.climber;

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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimberIOTalonFX implements ClimberIO {

  private TalonFX winchMotor = new TalonFX(65, "rio"); /* CAN ID TBD */
  private VoltageOut voltageRequest;
  private MotionMagicVoltage positionRequest;
  private final TrapezoidProfile trapezoidProfile;

  public DigitalInput magnetSwitch; /* Anticipating a Magnet Switch */

  private final StatusSignal<Current> motorCurrentValue = winchMotor.getSupplyCurrent();
  private final StatusSignal<Voltage> motorAppliedVolts = winchMotor.getMotorVoltage();
  private final StatusSignal<Angle> motorPosition = winchMotor.getPosition();
  private final StatusSignal<Temperature> motorTemp = winchMotor.getDeviceTemp();

  public final double gearRatio = 0; /*UPDATE IN FUTURE*/
  public final double pitchDiameter = 0; /*UPDATE IN FUTURE*/

  public ClimberIOTalonFX() {
    magnetSwitch = new DigitalInput(9);
    voltageRequest = new VoltageOut(0);
    positionRequest = new MotionMagicVoltage(0).withSlot(0);
    winchMotor.setPosition(0);

    trapezoidProfile =
        new TrapezoidProfile( // Units are rotations per second & rotations per seocond^2
            new TrapezoidProfile.Constraints(80, 160));

    config();
  }

  public void config() {
    var config = new TalonFXConfiguration();

    config.CurrentLimits.SupplyCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // in init function, set slot 0 gains
    // the values below are NOT correct and need to be adjusted to work with the windmill
    var slot0Configs = config.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 14; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0; // A velocity error of 1 rps results in 0.1 V output

    // set Motion Magic settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 640; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        320; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    winchMotor.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50, motorCurrentValue, motorAppliedVolts, motorPosition, motorTemp);

    winchMotor.optimizeBusUtilization();
  }

  private boolean isSwitchTriggered() {
    return !magnetSwitch.get();
  }

  @Override
  public void setVoltage(Voltage volts) {
    if (isSwitchTriggered() && volts.baseUnitMagnitude() < 0) {
      winchMotor.setControl(voltageRequest.withOutput(0.25));
    } else {
      winchMotor.setControl(voltageRequest.withOutput(volts));
    }
  }

  @Override
  public void stop() {
    winchMotor.setControl(voltageRequest.withOutput(null));
  }

  @Override
  public void setTargetHeight(double rotations) {
    winchMotor.setControl(positionRequest.withPosition(rotations));
  }

  @Override
  public void resetClimberEncoder() {
    winchMotor.setPosition(0);
  }

  @Override
  public void updateInputs(ClimberInputs climberInputs) {
    BaseStatusSignal.refreshAll(motorCurrentValue, motorAppliedVolts, motorPosition, motorTemp);

    climberInputs.climberCurrentAmps = motorCurrentValue.getValueAsDouble();

    climberInputs.getAppliedVolts = motorAppliedVolts.getValueAsDouble();

    climberInputs.tempCelsius = motorTemp.getValueAsDouble();

    climberInputs.isSwitchTriggered = isSwitchTriggered();
    climberInputs.leaderRotations = motorPosition.getValueAsDouble();
    climberInputs.heightInches = rotationsToInches(motorPosition.getValueAsDouble());

    SmartDashboard.putBoolean("Magnet Switch", isSwitchTriggered());
  }

  public double rotationsToInches(double rotations) {
    return (rotations / gearRatio) * (Math.PI * 2 * pitchDiameter);
  }
}
