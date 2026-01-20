package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

  public static final double fieldLength = Units.inchesToMeters(651.22);
  public static final double fieldWidth = Units.inchesToMeters(317.69);
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final int aprilTagCount = 32;
  public static final Translation2d fieldCenter =
      new Translation2d(fieldWidth / 2, fieldLength / 2);
  public static final double gamePieceDiameter = Units.inchesToMeters(5.91);
  public static final double startingLineX =
      Units.inchesToMeters(158.61); // distance from driver station to starting line
  public static final double centerLine = fieldLength / 2;

  public static final double trenchHeight = Units.inchesToMeters(40.25);
  public static final double trenchDepth = Units.inchesToMeters(47.0);
  public static final double trenchWidth = Units.inchesToMeters(65.65);
  public static final double trenchOpeningWidth = Units.inchesToMeters(50.34);
  public static final double trenchOpeningHeight = Units.inchesToMeters(22.25);

  public static class VerticalLines {
    public static final double startingLineX = centerLine - 166.74;
    public static final double allianceZone = startingLineX;
    public static final double nearNeutralZone = fieldWidth / 2 - Units.inchesToMeters(120);
    public static final double farNeutralZone = fieldWidth / 2 + Units.inchesToMeters(120);
  }

  public static class Hub {
    public static double hubWidth = Units.inchesToMeters(47);
    public static double hubHeight = Units.inchesToMeters(72);

    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    public static final Translation3d hubCenter =
        new Translation3d(Units.inchesToMeters(181.56), 158.32, hubHeight);

    public static final Pose2d[] hubFaces = new Pose2d[4]; // Start with side facing driver station

    static {
      hubFaces[0] =
          new Pose2d( // closest to driver station
              Units.inchesToMeters(158.84),
              Units.inchesToMeters(158.84),
              Rotation2d.fromDegrees(180));

      hubFaces[1] =
          new Pose2d( // left of driver station
              Units.inchesToMeters(182.11),
              Units.inchesToMeters(158.84 + (hubWidth / 2)),
              Rotation2d.fromDegrees(180));

      hubFaces[2] =
          new Pose2d( // away from driver station
              Units.inchesToMeters(158.84),
              Units.inchesToMeters(158.84 + hubWidth),
              Rotation2d.fromDegrees(180));

      hubFaces[3] =
          new Pose2d( // right of driver station
              Units.inchesToMeters(182.11),
              Units.inchesToMeters(158.84 - (hubWidth / 2)),
              Rotation2d.fromDegrees(180));

      hubFaces[4] = hubFaces[0].rotateAround(fieldCenter, Rotation2d.k180deg);
      hubFaces[5] = hubFaces[1].rotateAround(fieldCenter, Rotation2d.k180deg);
      hubFaces[6] = hubFaces[2].rotateAround(fieldCenter, Rotation2d.k180deg);
      hubFaces[7] = hubFaces[3].rotateAround(fieldCenter, Rotation2d.k180deg);
    }
  }

  public static class Tower {
    public static final double towerHeight = Units.inchesToMeters(78.25);
    public static final double towerDepth = Units.inchesToMeters(45);
    public static final double uprightHeight = Units.inchesToMeters(72.1);

    public static final double towerBaseWidth = Units.inchesToMeters(39);
    public static final double towerBaseLength = Units.inchesToMeters(45.18);

    public static final Translation2d towerCenter =
        new Translation2d(Units.inchesToMeters(43.51), 39.125);

    public static final Translation2d leftUpright =
        new Translation2d(Units.inchesToMeters(43.51), 158.84);
    public static final Translation2d rightUpright =
        new Translation2d(Units.inchesToMeters(43.51), 158.84 - 2 * 11.46);

    public static final double L1RungHeight = Units.inchesToMeters(27);
    public static final double L2RungHeight = Units.inchesToMeters(45);
    public static final double L3RungHeight = Units.inchesToMeters(63);

    public static final Translation2d oppTowerCenter =
        towerCenter.rotateAround(fieldCenter, Rotation2d.k180deg);

    public static final Translation2d oppLeftUpright =
        leftUpright.rotateAround(fieldCenter, Rotation2d.k180deg);

    public static final Translation2d oppRightUpright =
        rightUpright.rotateAround(fieldCenter, Rotation2d.k180deg);
  }

  public static class Depot {
    public static double depotWidth = Units.inchesToMeters(42);
    public static double depotLength = Units.inchesToMeters(27);
    public static double depotHeight = Units.inchesToMeters(1.125);

    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    public static Translation3d depotCenter =
        new Translation3d(depotLength, fieldWidth / 2 + distanceFromCenterY, depotHeight);

    public static final Translation3d leftCorner =
        new Translation3d(
            depotLength, (fieldWidth / 2) + distanceFromCenterY + (depotWidth / 2), depotHeight);

    public static final Translation3d rightCorner =
        new Translation3d(
            depotLength, (fieldWidth / 2) + distanceFromCenterY - (depotWidth / 2), depotHeight);
  }

  public static class Outpost {
    public static final double outpostWidth = Units.inchesToMeters(31.8);
    public static final double tallOpeningDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double baseOpeningHeight = Units.inchesToMeters(7.0);

    // Relevant reference points on alliance side
    public static final Translation2d centerPoint = new Translation2d(0, 49.32 / 2);
  }

  public static class RightTrench {
    public static final Translation3d openingTopLeft =
        new Translation3d(181.56, trenchOpeningWidth, trenchOpeningHeight);

    public static final Translation3d openingTopRight =
        new Translation3d(181.56, 0, trenchOpeningHeight);

    public static final Translation3d oppOpeningTopLeft =
        openingTopLeft.rotateAround(openingTopLeft, new Rotation3d(Rotation2d.k180deg));

    public static final Translation3d oppOpeningTopRight =
        openingTopRight.rotateAround(openingTopRight, new Rotation3d(Rotation2d.k180deg));
  }

  public static class LeftTrench {
    public static final Translation3d openingTopLeft =
        new Translation3d(181.56, fieldWidth, trenchOpeningHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(181.56, fieldWidth - trenchOpeningWidth, trenchOpeningHeight);

    public static final Translation3d oppOpeningTopLeft =
        openingTopLeft.rotateAround(openingTopLeft, new Rotation3d(Rotation2d.k180deg));

    public static final Translation3d oppOpeningTopRight =
        openingTopRight.rotateAround(openingTopRight, new Rotation3d(Rotation2d.k180deg));
  }

  public static final double bumpWidth = Units.inchesToMeters(73.0);
  public static final double bumpHeight = Units.inchesToMeters(6.513);
  public static final double bumpLength = Units.inchesToMeters(44.4);

  public static class RightBump {

    public static final Translation2d nearLeftCorner =
        new Translation2d(181.56 - bumpWidth / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = new Translation2d(181.56 - 47 / 2, 62.37);
    public static final Translation2d farLeftCorner =
        new Translation2d(181.56 + bumpWidth / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = new Translation2d(181.56 + 47 / 2, 62.37);
  }

  public static class LeftBump {

    public static final Translation2d nearLeftCorner =
        new Translation2d(181.56 - bumpWidth / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner =
        new Translation2d(181.56 - 47 / 2, 255.063781);
    public static final Translation2d farLeftCorner =
        new Translation2d(181.56 + bumpWidth / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner =
        new Translation2d(181.56 + 47 / 2, 255.063781);
  }
}
