package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

import java.util.Arrays;
import java.util.List;

public class LimelightConstants {

  public static class NameConstants {
    public static final String BARGE_NETWORKTABLE_KEY = "limelight-barge";
    public static final String REEF_NETWORKTABLE_KEY = "limelight-reef";
  }

  public static class PositionalConstants {
    public static final Pose3d BARGE_LIMELIGHT_LOCATION =
        new Pose3d(new Translation3d(
            -Units.inchesToMeters(6.018), 
            -Units.inchesToMeters(0.208), 
            Units.inchesToMeters(29.798)), 
          new Rotation3d(
            0, 
            30, 
            0));
            
    public static final Pose3d REEF_LIMELIGHT_LOCATION =
        new Pose3d(
          new Translation3d(
            -Units.inchesToMeters(8.490), 
            Units.inchesToMeters(3.025), 
            Units.inchesToMeters(8.052)), 
          new Rotation3d(
            0, 
            20, 
            0));
  }

  public static class PhysicalConstants {
    public static final double FOCAL_LENGTH = 4.1;
    public static final double REAL_WIDTH = 165.0;
    public static final double PIXEL_WIDTH = 320.0;
  }
}

class AprilTagConstants {
  public static class GameObjectIDConstants {
    public static final List<Integer> REEF_TAG_IDS =
        Arrays.asList(6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);
    public static final List<Integer> BARGE_TAG_IDS = Arrays.asList(4, 5, 14, 15);
    public static final List<Integer> PROCESSOR_TAG_IDS = Arrays.asList(3, 16);
    public static final List<Integer> CORAL_STATION_TAG_IDS = Arrays.asList(1, 2, 12, 13);
  }

  public static final List<Integer> RED_TAG_IDS =
      Arrays.asList(1, 2, 5, 6, 7, 8, 9, 10, 11, 15, 16);

  public static final List<Integer> BLUE_TAG_IDS =
      Arrays.asList(3, 4, 12, 13, 14, 17, 18, 19, 20, 21, 22);
}
