// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
  private static VisionSystemSim visionSim;

  // private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;
  private final VisionConsumer estimateConsumer;
  private Supplier<Pose2d> poseSupplier;
  private Pose2d bestPose = Pose2d.kZero;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionIOPhotonVisionSim(
      String name,
      Transform3d robotToCamera,
      Supplier<Pose2d> poseSupplier,
      VisionConsumer estimateConsumer,
      Supplier<Rotation3d> rotation3dSupplier) {
    super(name, robotToCamera, estimateConsumer, rotation3dSupplier);
    this.estimateConsumer = estimateConsumer;
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(aprilTagLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public Pose2d getBestPose() {
    return this.bestPose;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    visionSim.update(poseSupplier.get());

    super.updateInputs(inputs);
  }
}
