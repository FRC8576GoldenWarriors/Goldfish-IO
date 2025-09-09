package frc.robot.Subsystems.Vision.PhotonVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class PhotonVisionConstants {

  public static class NameConstants {
    public static final String LEFT_CAMERA = "Arducam_OV9782_USB_Camera (1)";
    public static final String RIGHT_CAMERA = "Arducam_OV9782_USB_Camera";
  }

  public static class PositionalConstants {
    public static final double mountingAngle = Units.degreesToRadians(-10);

    public static final Pose3d LEFT_CAMERA_LOCATION =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(13.128),
                -Units.inchesToMeters(8.320),
                Units.inchesToMeters(7.633)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(0)));

    public static final Pose3d RIGHT_CAMERA_LOCATION =
        new Pose3d(
            new Translation3d(
                Units.inchesToMeters(13.128),
                Units.inchesToMeters(8.320),
                Units.inchesToMeters(7.633)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(-10), Units.degreesToRadians(0)));
  }

  public static class PhysicalConstants {

    public static final int ALGAE_ID = 0;
    public static final int CORAL_ID = 1;

    public static final double ALGAE_DIAMETER = Units.inchesToMeters(16.25);

    public static final double CORAL_HEIGHT = Units.inchesToMeters(4.5);
    public static final double CORAL_WIDTH = Units.inchesToMeters(11.875);

    public static final double CAMERA_FOCAL_X = 550.82;
    public static final double CAMERA_FOCAL_Y = 550.72;
  }
}
