package frc.robot.subsystems.alignment;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class AlignmentIOPhotonVision implements AlignmentIO {

  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  protected final PhotonPoseEstimator alignmentPoseEstimator;

  /**
   * Creates a new AlignmentIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param rotationSupplier The 3D position of the camera relative to the robot.
   */
  public AlignmentIOPhotonVision(String name, Transform3d robotToCamera) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;

    AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    this.alignmentPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
  }

  @Override
  public void updateInputs(AlignmentIOInputs inputs) {
    inputs.connected = camera.isConnected();

    // Read new camera observations
    Set<Short> tagIDs = new HashSet<>();
    for (var result : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> overallPhotonResult = Optional.empty();
      if (!result.targets.isEmpty()) { // single tag result
        inputs.hasTarget = true;
        overallPhotonResult = alignmentPoseEstimator.update(result);
        var target = result.getBestTarget();
        // Add tag ID
        if (AlignmentConstants.TOWER_TAGS.contains((Integer) target.fiducialId)) {
          tagIDs.add((short) target.fiducialId);
          inputs.bestTargetTagId = target.getFiducialId();
          inputs.cameraToTarget = target.getBestCameraToTarget();
          if (overallPhotonResult.isPresent()) {
            inputs.photonpose = overallPhotonResult.get().estimatedPose;
          }
        }
      } else { // no targets
        inputs.hasTarget = false;
      }
    }
  }
}
