// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import static frc.robot.subsystems.vision.VisionConstants.*;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.vision.Vision.VisionConsumer;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

/** IO implementation for real PhotonVision hardware. */
public class VisionIOPhotonVision implements VisionIO {
  protected final PhotonCamera camera;
  protected final Transform3d robotToCamera;
  private final PhotonPoseEstimator visionPoseEstimator;
  private Matrix<N3, N1> curStdDev;
  private final VisionConsumer estimateConsumer;
  private Pose2d bestPose = Pose2d.kZero;
  private Supplier<Rotation3d> rotation3dSupplier;

  /**
   * Creates a new VisionIOPhotonVision.
   *
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  public VisionIOPhotonVision(
      String name,
      Transform3d robotToCamera,
      VisionConsumer estimateConsumer,
      Supplier<Rotation3d> rotation3dSupplier) {
    camera = new PhotonCamera(name);
    this.robotToCamera = robotToCamera;
    this.visionPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    this.curStdDev = kMultiTagStdDevs;
    this.estimateConsumer = estimateConsumer;
    this.rotation3dSupplier = rotation3dSupplier;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    Set<Short> tagIds = new HashSet<>();

    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var result : camera.getAllUnreadResults()) {
      visionPoseEstimator.addHeadingData(Timer.getFPGATimestamp(), rotation3dSupplier.get());
      visionEst = visionPoseEstimator.estimateCoprocMultiTagPose(result);
      if (visionEst.isEmpty()) {
        visionEst = visionPoseEstimator.estimateLowestAmbiguityPose(result);
      }

      updateEstimationStdDevs(visionEst, result.getTargets());
      visionEst.ifPresent(
          est -> {
            var estStdDevs = getEstimatedStdDevs();

            bestPose = est.estimatedPose.toPose2d();
            estimateConsumer.accept(bestPose, est.timestampSeconds, estStdDevs);
            Logger.recordOutput("Vision " + camera.getName() + " Pose", bestPose);
          });
    }

    // Save tag IDs to inputs objects
    inputs.tagIds = new int[tagIds.size()];
    int i = 0;
    for (int id : tagIds) {
      inputs.tagIds[i++] = id;
    }
  }

  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> trackedTargets) {

    if (estimatedPose.isEmpty()) {
      curStdDev = kSingleTagStdDevs;
    } else {
      var estStdDevs = kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      for (var tgt : trackedTargets) {
        var tagPose = visionPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());

        if (tagPose.isEmpty()) continue;
        numTags++;
        avgDist +=
            tagPose
                .get()
                .toPose2d()
                .getTranslation()
                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        curStdDev = kSingleTagStdDevs;
      } else {
        avgDist /= numTags;

        if (numTags > 1) {
          estStdDevs = kMultiTagStdDevs;
        }

        if (numTags == 1 && avgDist > 4) {
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        } else {
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
          curStdDev = estStdDevs;
        }
      }
    }
  }

  public Matrix<N3, N1> getEstimatedStdDevs() {
    return curStdDev;
  }

  @Override
  public Pose2d getBestPose() {
    return this.bestPose;
  }

  public String getName() {
    return camera.getName();
  }
}
