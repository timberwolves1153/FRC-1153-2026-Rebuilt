package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;

public class FeedUntilShotCommand extends Command {

  private final Indexer indexer;
  // private MedianFilter currentFilter = new MedianFilter(20);
  private static final double SHOT_FEED_AMPS = 15;

  public FeedUntilShotCommand(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  public boolean isShot() {
    // double averageAmps = currentFilter.calculate(indexer.getFeederCurrentAmps());
    double amps = indexer.getFeederCurrentAmps();
    // Logger.recordOutput("Feed2CommandAmpsAverage", averageAmps);
    if (amps > SHOT_FEED_AMPS) {
      return true;
    } else {
      return false;
    }
  }

  @Override
  public void initialize() {
    // currentFilter.reset();
    indexer.runSpin(5.5);
    indexer.runFeed(-12);
  }

  @Override
  public void execute() {
    // do nothing right now
  }

  @Override
  public boolean isFinished() {
    return isShot();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopAll();
    // currentFilter.reset();
  }
}
