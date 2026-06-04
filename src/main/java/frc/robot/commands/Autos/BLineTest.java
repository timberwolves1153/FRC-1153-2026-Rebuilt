package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.lib.BLine.Path;

public class BLineTest {
  public static Command getCommand() {
    return Commands.sequence(RobotContainer.pathBuilder.build(new Path("Center_to_Right_Trench")));
  }
}
