// package frc.robot.util;

// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.FieldConstants;
// import frc.robot.Robot;
// import frc.robot.subsystems.drive.Drive;

// public class Zones {

//     private static final Drive drive = Drive.getPose();

//     public static final Trigger blueFieldSide = drive.inXzone(0, FieldConstants.LinesVertical.center);
//     public static final Trigger opponentFieldSide =
//             new Trigger(() -> blueFieldSide.getAsBoolean() != Field.isBlue());

// }
