package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.turret.Turret;

public class SuperstructureCommands {
  private Drive drive;
  private Turret turret;
  private Flywheel flywheel;
  private Hood hood;

  public SuperstructureCommands(Drive drive, Flywheel flywheel, Hood hood, Turret turret) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
  }

  public Command autoAimTurretHub() {
    return Commands.parallel(
        turret.setTurretPositionHub(drive::getPose),
        hood.setPositionHoodHub(drive::getPose),
        flywheel.setVelocityHub(drive::getPose));
  }

  public Command autoAimTurretPassing() {
    return Commands.parallel(
        turret.setTurretPositionPassing(drive::getPose),
        hood.setPositionHoodPassing(drive::getPose),
        flywheel.setVelocityPassing(drive::getPose));
  }
}
