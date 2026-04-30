package frc.robot.commands;

import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.indexer.Indexer;
import org.littletonrobotics.junction.Logger;

public class FeedUntilEmptyCommand extends Command {

  private final Indexer indexer;
  private MedianFilter currentFilter = new MedianFilter(40);
  private static final double EMPTY_FEED_AMPS = 6;

  public FeedUntilEmptyCommand(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  public boolean isEmpty() {
    double averageAmps = currentFilter.calculate(indexer.getFeederCurrentAmps());
    Logger.recordOutput("FeedCommandAmpsAverage", averageAmps);
    return averageAmps > EMPTY_FEED_AMPS;
  }

  @Override
  public void initialize() {
    currentFilter.reset();
    indexer.runSpin(12);
    indexer.runFeed(-12);
  }

  @Override
  public void execute() {
    // do nothing right now
  }

  @Override
  public boolean isFinished() {
    return isEmpty();
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stopAll();
    currentFilter.reset();
  }
}
