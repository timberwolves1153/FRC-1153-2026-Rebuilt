package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.FieldConstants;
import frc.robot.interpolation.HoodTable;
import frc.robot.interpolation.InterpolatingDouble;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.launcher.flywheel.Flywheel;
import frc.robot.subsystems.launcher.hood.Hood;
import frc.robot.subsystems.launcher.turret.Turret;

public class Superstructure extends SubsystemBase {
  private Drive drive;
  private Flywheel flywheel;
  private Hood hood;
  private Turret turret;
  private HoodTable hoodtable;

  private Timer timer = new Timer();

  public Superstructure(
      Drive drive, Flywheel flywheel, Hood hood, Turret turret, HoodTable hoodtable) {
    this.drive = drive;
    this.flywheel = flywheel;
    this.hood = hood;
    this.turret = turret;
    hoodtable = new HoodTable();
  }

  public void interpolateShot() {
    hood.setPositionHood(
        hoodtable.hoodMap.getInterpolated(
                new InterpolatingDouble(FieldConstants.getDistanceToHubCenter(drive.getPose())))
            .value);
  }
}
