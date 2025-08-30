package frc.robot.Subsystems.Vision.PhotonVision;

import edu.wpi.first.math.util.Units;

public class PhotonVisionConstants {

  public static class NameConstants {
    public static final String LEFT_CAMERA = "Arducam_OV9782_USB_Camera (1)";
    public static final String RIGHT_CAMERA = "Arducam_OV9782_USB_Camera";
  }

  public static class PositionalConstants {
    public static final double mountingAngle = Units.degreesToRadians(-10);
  }

  public static class PhysicalConstants {
    public static final double CORAL_HEIGHT = Units.inchesToMeters(4.5);
    public static final double CORAL_WIDTH = Units.inchesToMeters(11.875);

    public static final double CAMERA_FOCAL_X = 0.3;
    public static final double CAMERA_FOCAL_Y = 0.3;
  }
}
