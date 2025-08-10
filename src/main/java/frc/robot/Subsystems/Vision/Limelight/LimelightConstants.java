package frc.robot.Subsystems.Vision.Limelight;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.HashMap;
import java.util.Map;

public class LimelightConstants {

  public static class NameConstants {
    public static final String BARGE_NETWORKTABLE_KEY = "limelight-barge";
    public static final String REEF_NETWORKTABLE_KEY = "limelight-reef";
  }

  public static class PositionalConstants {
    public static final Pose3d BARGE_LIMELIGHT_LOCATION =
        new Pose3d(
            new Translation3d(
                -Units.inchesToMeters(6.018),
                -Units.inchesToMeters(0.208),
                Units.inchesToMeters(29.798)),
            new Rotation3d(
                Units.degreesToRadians(0), Units.degreesToRadians(30), Units.degreesToRadians(0)));

    public static final Pose3d REEF_LIMELIGHT_LOCATION =
        new Pose3d(
            new Translation3d(
                -Units.inchesToMeters(8.490),
                Units.inchesToMeters(3.025),
                Units.inchesToMeters(8.052)),
            new Rotation3d(
                Units.degreesToRadians(0),
                Units.degreesToRadians(20),
                Units.degreesToRadians(180)));
  }

  public static class PhysicalConstants {
    public static final double FOCAL_LENGTH = 4.1;
    public static final double REAL_WIDTH = 165.0;
    public static final double PIXEL_WIDTH = 320.0;
    public static final double DESIRED_APRIL_TAG_DISTANCE_BARGE = 2.75;
    public static final double DESIRED_APRIL_TAG_DISTANCE_REEF = 0.125;
    public static final double LEFT_STICK_OFFSET = -0.3;
    public static final double RIGHT_STICK_OFFSET = 0.3;

    public static final HashMap<Integer, Double> tagMap =
        new HashMap<>(
            Map.ofEntries(
                Map.entry(18, 180.0),
                Map.entry(7, 180.0),
                Map.entry(17, -120.0),
                Map.entry(6, 120.0),
                Map.entry(22, -60.0),
                Map.entry(11, 60.0),
                Map.entry(21, 0.0),
                Map.entry(10, 0.0),
                Map.entry(20, 60.0),
                Map.entry(9, -60.0),
                Map.entry(19, 120.0),
                Map.entry(8, -120.0)));
  }

  public static class PIDConstants {
    public static final double rotationkP = 0.08;
    public static final double rotationkI = 0.00;
    public static final double rotationkD = 0.001;
    public static final double ALLOWED_ANGLE_ERROR = 2.0;

    public static final double forwardkP = 2.3;
    public static final double forwardkI = 0.04;
    public static final double forwardkD = 0.001;
    public static final double ALLOWED_DISTANCE_ERROR = 0.5;

    public static final double strafekP = 2.3; // 0.1;
    public static final double strafekI = 0.0;
    public static final double strafekD = 0.001;
    public static final double ALLOWED_STRAFE_ERROR = 0.016; // 1.0;
  }

  public static enum reefOffsets {
    LEFT,
    CENTER,
    RIGHT
  }
}
