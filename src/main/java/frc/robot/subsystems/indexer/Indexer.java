package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Indexer extends SubsystemBase {
    // --- Constants ---
    // Update these if IDs are swapped
    private static final int SPIN_MOTOR_ID = 51;
    private static final int FEED_MOTOR_ID = 62;

    // Speeds (0.0 to 1.0)
    private static final double INDEX_SPEED = 0.5;
    private static final double FEED_SPEED = 0.8;

    // --- Hardware ---
    // Krakens controlled by TalonFX class in Phoenix 6
    private final TalonFX spinMotor;
    private final TalonFX feedMotor;

    public Indexer() {
        // Initialize motors
        spinMotor = new TalonFX(SPIN_MOTOR_ID);
        feedMotor = new TalonFX(FEED_MOTOR_ID);

        // Configure Factory Defaults
        spinMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());
        feedMotor.getConfigurator().apply(new com.ctre.phoenix6.configs.TalonFXConfiguration());

        // Set Neutral Mode to Brake or Coast
        spinMotor.setNeutralMode(NeutralModeValue.Brake);
        feedMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    /**
     * Runs the Spin motor (ID 51) to intake game pieces.
     */
    public void runSpin(double speed) {
        spinMotor.set(speed);
    }

    /**
     * Runs the Feed motor (ID 62) to push piece into shooter.
     */
    public void runFeed(double speed) {
        feedMotor.set(speed);
    }

    /**
     * Stops both motors.
     */
    public void stop() {
        spinMotor.set(0);
        feedMotor.set(0);
    }

    // --- Command Factory Methods ---
    // These create Commands compatible with the RobotContainer bindings

    /**
     * Command to run the indexer (spin only).
     */
    public Command intakeCommand() {
        return this.runEnd(
            () -> runSpin(INDEX_SPEED),
            () -> stop()
        );
    }

    /**
     * Command to run both motors to shoot/feed.
     */
    public Command feedToShooterCommand() {
        return this.runEnd(
            () -> {
                runSpin(INDEX_SPEED);
                runFeed(FEED_SPEED);
            },
            () -> stop()
        );
    }

    /**
     * Command to reverse everything (if jam).
     */
    public Command reverseCommand() {
        return this.runEnd(
            () -> {
                runSpin(-INDEX_SPEED);
                runFeed(-FEED_SPEED);
            },
            () -> stop()
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Indexer", inputs);
    }

    public void run() {
        io.setVoltage(3);
    }

    public void stop() {
        io.stop();
    }
}