// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision.Limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.Limelight.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Stack;

public class LimelightIO implements LimelightVisionIO {
  private String networkTableName;
  // private StructPublisher<Pose2d> limelightRobotPose;
  private static Drivetrain drivetrainInstance = Drivetrain.getInstance();
  public static boolean AlignedVar = false;
  private Stack<Double> tagSeenTimestampStack = new Stack<>();

  public LimelightIO(String networkTableName) {
    this.networkTableName = networkTableName;
    // this.limelightRobotPose =
    //     NetworkTableInstance.getDefault()
    //         .getTable("Goldfish")
    //         .getStructTopic("Limelight Vision Pose/" + networkTableName, Pose2d.struct)
    //         .publish();
  }

  public LimelightIO(String networkTableName, Pose3d limelightPose) {
    this(networkTableName);
    this.setLimelightLocation(limelightPose);
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.connected = NetworkTableInstance.getDefault().getTable(networkTableName) != null;
    inputs.hasTargets = LimelightHelpers.getTV(networkTableName);

    if (inputs.hasTargets) {
      inputs.tagId = (int) LimelightHelpers.getFiducialID(networkTableName);
      tagSeenTimestampStack.add(Timer.getFPGATimestamp());
      // Vertical Angle to Tag
      inputs.pitch = LimelightHelpers.getTY(networkTableName);
      // Horizontal Angle to Tag
      inputs.yaw = LimelightHelpers.getTX(networkTableName);
      inputs.area = LimelightHelpers.getTA(networkTableName);

      Pair<PoseEstimate, Boolean> megaTag1EstimateAndStatus = this.getMegaTag1RobotPoseEstimate();
      Pair<PoseEstimate, Boolean> megaTag2EstimateAndStatus = this.getMegaTag2RobotPoseEstimate();

      this.integratePose();

      inputs.megaTag1UpdateAccepted = megaTag1EstimateAndStatus.getSecond();
      inputs.megaTag2UpdateAccepted = megaTag2EstimateAndStatus.getSecond();

      var megaTag1PoseEstimate = megaTag1EstimateAndStatus.getFirst();
      var megaTag2PoseEstimate = megaTag2EstimateAndStatus.getFirst();

      inputs.megaTag1Estimate = megaTag1PoseEstimate.pose;
      inputs.megaTag2Estimate = megaTag2PoseEstimate.pose;

      inputs.megaTag1AmountOfTagsInView = megaTag1PoseEstimate.tagCount;
      inputs.megaTag2AmountOfTagsInView = megaTag2PoseEstimate.tagCount;

      // inputs.megaTag1ambiguity = megaTag1PoseEstimate.rawFiducials[0].ambiguity;
      // inputs.megaTag2ambiguity = megaTag2PoseEstimate.rawFiducials[0].ambiguity;

      if (!megaTag2PoseEstimate.pose.equals(null))
        inputs.metaTag2XCord = megaTag2PoseEstimate.pose.getX();

      inputs.megaTag1distanceToTagMeters = megaTag1PoseEstimate.avgTagDist;
      inputs.megaTag2distanceToTagMeters = megaTag2PoseEstimate.avgTagDist;
    }

    this.setDynamicCrop();
  }

  @Override
  public void setLimelightLocation(Pose3d limelightPose) {
    LimelightHelpers.setCameraPose_RobotSpace(
        networkTableName,
        limelightPose.getX(),
        limelightPose.getY(),
        limelightPose.getZ(),
        Units.radiansToDegrees(limelightPose.getRotation().getX()),
        Units.radiansToDegrees(limelightPose.getRotation().getY()),
        Units.radiansToDegrees(limelightPose.getRotation().getZ()));
  }

  @Override
  public void setLeds(boolean on) {
    if (on) LimelightHelpers.setLEDMode_ForceOn(networkTableName);
    else LimelightHelpers.setLEDMode_ForceOff(networkTableName);
  }

  @Override
  public void setThrottleValue(int throttleValue) {
    LimelightHelpers.SetThrottle(networkTableName, throttleValue);
  }

  @Override
  public Pose3d getLimelightLocation() {
    return LimelightHelpers.getCameraPose3d_RobotSpace(networkTableName);
  }

  @Override
  public String getLimelightName() {
    return networkTableName;
  }

  private void integratePose() {
    var megaTag2PoseEstimate = this.getMegaTag2RobotPoseEstimate();

    if (megaTag2PoseEstimate.getSecond() && megaTag2PoseEstimate.getFirst() != null) {
      // drivetrainInstance.setVisionMeasurementStdDevs(.7, .7, 9999999);
      drivetrainInstance.setVisionMeasurementStdDevs(
          .3 * LimelightHelpers.getTY(networkTableName),
          .3 * LimelightHelpers.getTX(networkTableName),
          9999999);

      drivetrainInstance.addVisionMeasurement(
          megaTag2PoseEstimate.getFirst().pose, megaTag2PoseEstimate.getFirst().timestampSeconds);
    }
  }

  private void setCrop(double leftCrop, double rightCrop, double bottomCrop, double topCrop) {
    LimelightHelpers.setCropWindow(networkTableName, leftCrop, rightCrop, bottomCrop, topCrop);
  }

  public void setFullCrop() {
    setCrop(-1, 1, -1, 1);
  }

  public void setDynamicCrop() {
    ArrayList<Double> cornerXList = new ArrayList<>();
    ArrayList<Double> cornerYList = new ArrayList<>();

    var corners =
        LimelightHelpers.getLimelightNTTable(networkTableName)
            .getEntry("tcornxy")
            .getDoubleArray(new double[0]);

    int tagsWithSuitbaleCorners = corners.length / 8;

    for (int i = 0; i < (8 * tagsWithSuitbaleCorners - 1); i++) {
      if (i % 2 == 0) cornerXList.add(corners[i]);
      else cornerYList.add(corners[i]);
    }
    if (!(cornerXList.size() >= 4) || !(cornerYList.size() >= 4)) {
      setFullCrop();
      return;
    }

    setCrop(
        Collections.min(cornerXList).doubleValue(),
        Collections.max(cornerXList).doubleValue(),
        Collections.min(cornerYList).doubleValue(),
        Collections.max(cornerYList).doubleValue());
  }

  private Pair<PoseEstimate, Boolean> getMegaTag1RobotPoseEstimate() {
    boolean acceptUpdate = true;
    LimelightHelpers.PoseEstimate megaTagEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue(networkTableName);

    if (megaTagEstimate.tagCount == 0) acceptUpdate = false;

    if (acceptUpdate) {

      if (megaTagEstimate.tagCount == 1
          && megaTagEstimate.rawFiducials.length == 1
          && (megaTagEstimate.rawFiducials[0].ambiguity > .1
              || megaTagEstimate.rawFiducials[0].distToCamera > 3)) acceptUpdate = false;
    }

    // if (acceptUpdate) {

    //   drivetrainInstance.setVisionMeasurementStdDevs(.5, .5, 9999999);

    //   drivetrainInstance.addVisionMeasurement(
    //       megaTagEstimate.pose, megaTagEstimate.timestampSeconds);
    // }

    return Pair.of(megaTagEstimate, acceptUpdate);
  }

  private Pair<PoseEstimate, Boolean> getMegaTag2RobotPoseEstimate() {

    boolean acceptUpdate = true;

    LimelightHelpers.PoseEstimate megaTagEstimate =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(networkTableName);

    if (megaTagEstimate.tagCount == 0) acceptUpdate = false;

    if (acceptUpdate) {

      if (megaTagEstimate.tagCount == 1
          && megaTagEstimate.rawFiducials.length == 1
          && (megaTagEstimate.rawFiducials[0].ambiguity > .1
              || megaTagEstimate.rawFiducials[0].distToCamera > 3)) acceptUpdate = false;
      LimelightHelpers.SetRobotOrientation(
          networkTableName,
          drivetrainInstance
              .getBlueAbsoluteHeading(), // maybe change to blue absolute, idk if it changes
          // anything.
          0,
          0,
          0,
          0,
          0);

      if (Math.abs(drivetrainInstance.getRate()) > 720) acceptUpdate = false;
    }

    // if (acceptUpdate) {

    //   drivetrainInstance.setVisionMeasurementStdDevs(.7, .7, 9999999);

    //   drivetrainInstance.addVisionMeasurement(
    //       megaTagEstimate.pose, megaTagEstimate.timestampSeconds);
    // }

    return Pair.of(megaTagEstimate, acceptUpdate);
  }

  public double timeBetweenTagSighting() {
    if (tagSeenTimestampStack.size() >= 2) {
      return Math.abs(tagSeenTimestampStack.pop() - tagSeenTimestampStack.pop());
    }
    return Double.MAX_VALUE;
  }
}
