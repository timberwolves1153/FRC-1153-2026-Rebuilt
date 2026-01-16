package frc.robot.subsystems.Climber;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

public class Climber extends SubsystemBase {

  public ClimberIO climberIO;
  public ClimberInputsAutoLogged climberInputs;

  public TrapezoidProfile.Constraints constraints;
  public ProfiledPIDController profiledPIDController;
  public ElevatorFeedforward climberFF;

  private LoggedMechanism2d climberMech2d;
  private LoggedMechanismRoot2d climberRoot2d;
  private LoggedMechanismLigament2d climberLig2d;

  private final Rotation2d elev_angle = Rotation2d.fromDegrees(90);
  public final SysIdRoutine sysIdRoutine;
  private final double gearRatio = 0; /* figure this out later */
  private final double pitchDiameter = 0; /* ^^^^^^^^^^^^^^^^^^^^^ */

  public enum ClimberGoal {
  // STOW(),
  // L1(),
  // L2(),
  // L3();
  ;
    /*check back for more */

    private double heightinInches;

    private ClimberGoal(double heightinInches) {
      this.heightinInches = heightinInches;
    }

    public double getHeightInInches() {
      return this.heightinInches;
    }
  }

  public Climber(ClimberIO climberIO) {
    climberInputs = new ClimberInputsAutoLogged();
    this.climberIO = climberIO;

    double ks = 0; /* static gain in volts */
    double kg = 0; /* gravity gain in volts */
    double kv = 0; /* velocity gain in V/(m/s)*/

    double kp = 0; /* proportional coefficent */
    double ki = 0; /* integral coefficent */
    double kd = 0; /* derivitve coefficent */

    sysIdRoutine =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null, // Use default ramp rate (1 V/s)
                Volts.of(4), // Reduce dynamic step voltage to 4 to prevent brownout
                null, // Use default timeout (10 s)
                // Log state with Phoenix SignalLogger class
                (state) -> SignalLogger.writeString("state", state.toString())),
            new SysIdRoutine.Mechanism((volts) -> climberIO.setVoltage(volts), null, this));

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        constraints = new TrapezoidProfile.Constraints(5, 10);
        profiledPIDController = new ProfiledPIDController(0, 0, 0, constraints);

        /* PLACEHOLDERS UNTIL REAL VALUES */

        ks = 0; /* static gain in volts */
        kg = 0; /* gravity gain in volts */
        kv = 0; /* velocity gain in V/(m/s)*/

        kp = 0; /* proportional coefficent */
        ki = 0; /* integral coefficent */
        kd = 0; /* derivitve coefficent */

        climberFF = new ElevatorFeedforward(ks, kg, kv);
        break; /*NEED TO CHECK THESE VALUES*/

      case SIM:
        ks = 0; /* static gain in volts */
        kg = 0; /* gravity gain in volts */
        kv = 0; /* velocity gain in V/(m/s)*/

        kp = 0; /* proportional coefficent */
        ki = 0; /* integral coefficent */
        kd = 0; /* derivitve coefficent */

        constraints = new TrapezoidProfile.Constraints(5.0, 10.0);

        profiledPIDController = new ProfiledPIDController(kp, ki, kd, constraints);

        climberFF = new ElevatorFeedforward(ks, kg, kv);

        break; /*NEED TO CHECK THESE VALUES*/

      default:
        profiledPIDController = new ProfiledPIDController(0, 0, 0, new Constraints(5, 10));
        climberFF = new ElevatorFeedforward(0, 0, 0);
        break; /*NEED TO CHECK THESE VALUES*/
    }

    climberMech2d =
        new LoggedMechanism2d(0, 0); /* change these values based on real time updates */

    double rootX = 0; /* Delete when you have the values */
    double rootY = 0; /* ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^ */

    climberRoot2d = climberMech2d.getRoot("Climber", rootX, rootY);

    double length = 0;
    double angle = 0;
    climberLig2d = new LoggedMechanismLigament2d("Climber Lig", length, angle);
    climberRoot2d.append(climberLig2d);
  }

  public void setVoltage(double volts) {
    climberIO.setVoltage(Voltage.ofBaseUnits(volts, Volts));
  }

  public void stop() {
    climberIO.stop();
  }

  public void updateMech2d() {
    climberLig2d.setLength(climberInputs.heightInches);
  }

  public void setTargetHeight(ClimberGoal heightGoal) {
    setTargetHeightInches(heightGoal.getHeightInInches());
  }

  public void setTargetHeightInches(double inches) {
    double rots = inchesToRotations(inches);
    climberIO.setTargetHeight(rots);
  }

  public void holdTargetHeight() {
    double calculatedVolts =
        profiledPIDController.calculate(climberInputs.heightInches, climberInputs.heightInches)
            + climberFF.calculate(profiledPIDController.getSetpoint().velocity);
    climberIO.setVoltage(Voltage.ofBaseUnits(calculatedVolts, Volts));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }

  public double inchesToRotations(double inches) {
    return (7.1429 * inches) / (Math.PI * pitchDiameter);
  }

  public double rotationsToInches(double rotations) {
    return (rotations / gearRatio) * (Math.PI * 2 * pitchDiameter);
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);
    Logger.recordOutput("Elevator/Mechanism2D", climberMech2d);
    Logger.recordOutput("Elevator Height", climberInputs.leaderRotations);
    climberLig2d.setLength(Units.inchesToMeters(climberInputs.heightInches));
    SmartDashboard.putNumber("elevator height", climberInputs.heightInches);
    SmartDashboard.putNumber("elevator rotations", climberInputs.leaderRotations);
  }
}
