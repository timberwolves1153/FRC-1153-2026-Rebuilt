package frc.robot.subsystems.alignment;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Alignment extends SubsystemBase {
  private final AlignmentIO io;
  private final AlignmentIOInputsAutoLogged alignmentInputs;

  public Alignment(AlignmentIO io) {
    this.io = io;

    // Initialize alignmentInputs
    this.alignmentInputs = new AlignmentIOInputsAutoLogged();
  }

  public Transform3d getBestCameraToTarget() {
    if (AlignmentConstants.TOWER_TAGS.contains(this.alignmentInputs.bestTargetTagId)) {
      return this.alignmentInputs.cameraToTarget;
    }
    return new Transform3d();
  }

  public int getTargetId() {
    return this.alignmentInputs.bestTargetTagId;
  }

  public Pose2d getRobotPose() {
    return alignmentInputs.photonpose.toPose2d();
  }

  public void updateCameraPosition() {

    Transform3d newRobotToCameraTransform =
        new Transform3d(
            Units.inchesToMeters(SmartDashboard.getNumber("cameraX", 0)),
            Units.inchesToMeters(SmartDashboard.getNumber("cameraY", 0)),
            Units.inchesToMeters(SmartDashboard.getNumber("cameraZ", 0)),
            new Rotation3d(
                Units.degreesToRadians(SmartDashboard.getNumber("cameraRoll", 0)),
                Units.degreesToRadians(SmartDashboard.getNumber("cameraPitch", 0)),
                Units.degreesToRadians(SmartDashboard.getNumber("cameraYaw", 0))));
    io.updateRobotToCamera(newRobotToCameraTransform);
  }

  @Override
  public void periodic() {
    io.updateInputs(alignmentInputs);
    Logger.processInputs("Alignment/Camera", alignmentInputs);
  }
}
