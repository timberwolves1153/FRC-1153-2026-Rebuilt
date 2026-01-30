// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "camera_0";
  public static String camera1Name = "camera_1";
  public static String camera2Name = "camera_2";
  // public static String camera3Name = "camera_3";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToCamera0 =
      new Transform3d(
          Units.inchesToMeters(4),
          Units.inchesToMeters(13),
          Units.inchesToMeters(18.46),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(120)));

  public static Transform3d robotToCamera1 =
      new Transform3d(
          Units.inchesToMeters(-13),
          Units.inchesToMeters(8),
          Units.inchesToMeters(18.46),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(75)));

  public static Transform3d robotToCamera2 =
      new Transform3d(
          Units.inchesToMeters(-13),
          Units.inchesToMeters(0),
          Units.inchesToMeters(15),
          new Rotation3d(0.0, Units.degreesToRadians(-5), Units.degreesToRadians(180)));

  // public static Transform3d robotToCamera3 =
  //     new Transform3d(
  //         Units.inchesToMeters(9),
  //         Units.inchesToMeters(12),
  //         Units.inchesToMeters(18.5),
  //         new Rotation3d(0.0, Units.degreesToRadians(-15), Units.degreesToRadians(45)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.3;
  public static double maxZError = 0.75;

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.02; // Meters
  public static double angularStdDevBaseline = 0.06; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0 // Camera 1
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
