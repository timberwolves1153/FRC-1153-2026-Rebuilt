package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {

  public IndexerIO indexerIO;
  public IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  public Indexer(IndexerIO indexerIO) {

    this.indexerIO = indexerIO;

    switch (Constants.currentMode) {
      case REAL:
      case REPLAY:
        break;
      case SIM:
        break;
    }
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerInputs);
    Logger.processInputs("Indexer", indexerInputs);
  }

  /** Runs the Spin motor to serialize game pieces. */
  public void runSpin(double volts) {
    indexerIO.runSpin(volts);
  }

  /** Runs the Feed motor to push piece into shooter. */
  public void runFeed(double volts) {
    indexerIO.runFeeder(volts);
  }

  /** Stops both motors. */
  public void stopAll() {
    stopSpin();
    stopFeeder();
  }

  public void stopSpin() {
    indexerIO.stopSpin();
  }

  public void stopFeeder() {
    indexerIO.stopFeeder();
  }
}
