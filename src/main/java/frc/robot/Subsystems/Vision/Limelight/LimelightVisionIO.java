package frc.robot.Subsystems.Vision.Limelight;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface LimelightVisionIO {

  default void updateInputs(VisionIOInputs inputs) {}

  default void setLeds(boolean on) {}

  default void setLimelightLocation(Pose3d limelightPose) {}

  default Pose3d getLimelightLocation() {
    return new Pose3d();
  }

  default String getLimelightName() {
    return "";
  }

  default void setThrottleValue(int throttleValue) {}

  @AutoLog
  class VisionIOInputs {
    public boolean hasTargets = false;
    public int tagId = -1;
    public boolean connected = false;
    public double yaw = 0;
    public double pitch = 0;
    public double area = 0;

    public double megaTag1distanceToTagMeters = 0;
    public double megaTag2distanceToTagMeters = 0;

    public double megaTag1ambiguity = 0;
    public double megaTag2ambiguity = 0;

    public int megaTag1AmountOfTagsInView = 0;
    public int megaTag2AmountOfTagsInView = 0;

    public boolean megaTag1UpdateAccepted = false;
    public boolean megaTag2UpdateAccepted = false;

    public double metaTag2XCord = 0.0;

    public Pose2d megaTag1Estimate = null; // new Pose2d();
    public Pose2d megaTag2Estimate = null; // new Pose2d();

    public boolean isAligned = false;
    public double[] generatedStddevs = null;
    public double velocity = 0.0;
  }
}
