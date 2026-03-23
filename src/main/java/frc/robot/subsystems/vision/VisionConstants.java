// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  // AprilTag layout
  public static AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "Turret";
  public static String camera1Name = "Climber";
  public static String camera2Name = "Swerve";
  public static String camera3Name = "Hopper";

  // Robot to camera transforms
  // (Not used by Limelight, configure in web UI instead)
  public static Transform3d robotToTurretCamera =
      new Transform3d(
          Units.inchesToMeters(-10.783),
          Units.inchesToMeters(-12.645),
          Units.inchesToMeters(15.743),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(180 - 20)));

  public static Transform3d robotToClimberCamera =
      new Transform3d(
          Units.inchesToMeters(-12.598),
          Units.inchesToMeters(-10.083),
          Units.inchesToMeters(15.743),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(270 + 25)));

  public static Transform3d robotToSwerveCamera =
      new Transform3d(
          Units.inchesToMeters(-11.403),
          Units.inchesToMeters(10.623),
          Units.inchesToMeters(8.224),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(180 - 88)));

  public static Transform3d robotToHopperCamera =
      new Transform3d(
          Units.inchesToMeters(-8),
          Units.inchesToMeters(0), // -12.645
          Units.inchesToMeters(15.743),
          new Rotation3d(0.0, Units.degreesToRadians(-20), Units.degreesToRadians(180 + 20)));

  public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.2;
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
        1.0, // Camera 1
        1.0, // Camera 2
        1.0 // Camera 3
      };

  // Multipliers to apply for MegaTag 2 observations
  public static double linearStdDevMegatag2Factor = 0.5; // More stable than full 3D solve
  public static double angularStdDevMegatag2Factor =
      Double.POSITIVE_INFINITY; // No rotation data available
}
