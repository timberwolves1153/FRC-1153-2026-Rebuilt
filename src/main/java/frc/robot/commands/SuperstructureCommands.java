package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.turret.Turret;

public class SuperstructureCommands {
  private Drive drive;
  private Flywheel flywheel;
  private Hood hood;
  private Turret turret;
  private Superstructure superstructure;

  public SuperstructureCommands(
      Drive drive, Flywheel flywheel, Hood hood, Turret turret, Superstructure superstructure) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    this.superstructure = superstructure;
  }

  public Command autoAimTurretHub() {
    return Commands.run(
        () ->
            superstructure
                .interpolateHubCommand()
                .alongWith(superstructure.autoAimTurretHubCommand()),
        superstructure);
  }
}
