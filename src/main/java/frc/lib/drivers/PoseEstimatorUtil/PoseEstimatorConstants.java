package frc.lib.drivers.PoseEstimatorUtil;

public class PoseEstimatorConstants {

  static class ValidityEstimateConstants {

    public static final double MAXIMUM_TAG_AMBIGUITY = 0.4;

    public static final double MIN_TAG_DISTANCE = 0.1;
    public static final double MAX_TAG_DISTANCE = 4;

    public static final double MAX_ROTATIONAL_SPEED = 3 * Math.PI;
    public static final double MAX_TRANSLATION_SPEED = 4;
  }
}
