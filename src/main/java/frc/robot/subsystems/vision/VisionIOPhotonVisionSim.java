// // Copyright (c) 2021-2026 Littleton Robotics
// // http://github.com/Mechanical-Advantage
// //
// // Use of this source code is governed by a BSD
// // license that can be found in the LICENSE file
// // at the root directory of this project.

// package frc.robot.subsystems.vision;

// import static frc.robot.subsystems.vision.VisionConstants.aprilTagLayout;
// import static frc.robot.subsystems.vision.VisionConstants.kMultiTagStdDevs;

// import edu.wpi.first.math.Matrix;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Transform3d;
// import edu.wpi.first.math.numbers.N1;
// import edu.wpi.first.math.numbers.N3;

// import java.util.function.Supplier;

// import org.photonvision.PhotonPoseEstimator;
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.SimCameraProperties;
// import org.photonvision.simulation.VisionSystemSim;

// /** IO implementation for physics sim using PhotonVision simulator. */
// public class VisionIOPhotonVisionSim extends VisionIOPhotonVision {
//   private static VisionSystemSim visionSim;

//   private final Supplier<Pose2d> poseSupplier;
//   private final PhotonCameraSim cameraSim;

//   private final PhotonPoseEstimator visionPoseEstimator;
//   private final Matrix<N3, N1> curStdDev;
//   private final EstimateConsumer estimateConsumer;

//   /**
//    * Creates a new VisionIOPhotonVisionSim.
//    *
//    * @param name The name of the camera.
//    * @param poseSupplier Supplier for the robot pose to use in simulation.
//    */
//   public VisionIOPhotonVisionSim(
//       String name, Transform3d robotToCamera, EstimateConsumer estimateConsumer) {
//     super(name, robotToCamera,
//     new PhotonPoseEstimator(aprilTagLayout, robotToCamera),
//     kMultiTagStdDevs,
//     estimateConsumer);

//     // Initialize vision sim
//     if (visionSim == null) {
//       visionSim = new VisionSystemSim("main");
//       visionSim.addAprilTags(aprilTagLayout);
//     }

//     // Add sim camera
//     var cameraProperties = new SimCameraProperties();
//     cameraSim = new PhotonCameraSim(camera, cameraProperties, aprilTagLayout);
//     visionSim.addCamera(cameraSim, robotToCamera);
//   }

//   @Override
//   public void updateInputs(VisionIOInputs inputs) {
//     visionSim.update(poseSupplier.get());
//     super.updateInputs(inputs);
//   }
// }
