// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.LimelightHelpers.PoseEstimate;

public class LimelightIO implements LimelightVisionIO {
  private String networkTableName;
  private StructPublisher<Pose2d> limelightRobotPose;
  private static Drivetrain drivetrainInstance = Drivetrain.getInstance();

  public LimelightIO(String networkTableName) {
    this.networkTableName = networkTableName;
    this.limelightRobotPose =
        NetworkTableInstance.getDefault()
            .getTable("Goldfish")
            .getStructTopic("Limelight Vision Pose/" + networkTableName, Pose2d.struct)
            .publish();
  }

  public LimelightIO(String networkTableName, Pose3d limelightPose) {
    this(networkTableName);
    this.setLimelightLocation(limelightPose);
  }

  @Override
  public synchronized void updateInputs(VisionIOInputs inputs) {
    inputs.connected = NetworkTableInstance.getDefault().getTable(networkTableName) == null;
    inputs.hasTargets = LimelightHelpers.getTV(networkTableName);

    if (inputs.hasTargets) {
      inputs.tagId = (int) LimelightHelpers.getFiducialID(networkTableName);
      // Vertical Angle to Tag
      inputs.pitch = LimelightHelpers.getTY(networkTableName);
      // Horizontal Angle to Tag
      inputs.yaw = LimelightHelpers.getTX(networkTableName);
      inputs.area = LimelightHelpers.getTA(networkTableName);

      Pair<PoseEstimate, Boolean> poseEstimateAndStatus = this.getRobotPoseEstimate(true);
      var poseEstimate = poseEstimateAndStatus.getFirst();
      var acceptUpdate = poseEstimateAndStatus.getSecond();
      if (acceptUpdate) {
        limelightRobotPose.set(inputs.visionPoseEstimate);
      }
      inputs.updateAccepted = acceptUpdate;
      inputs.ambiguity = poseEstimate.rawFiducials[0].ambiguity;
      inputs.amountOfTagsInView = poseEstimate.tagCount;
      inputs.visionPoseEstimate = poseEstimate.pose;
      inputs.distanceToTagMeters = poseEstimate.avgTagDist;
    }
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

  private Pair<PoseEstimate, Boolean> getRobotPoseEstimate(boolean useMegaTag2) {
    boolean acceptUpdate = true;
    LimelightHelpers.PoseEstimate megaTagEstimate =
        (useMegaTag2)
            ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(networkTableName)
            : LimelightHelpers.getBotPoseEstimate_wpiBlue(networkTableName);

    if (megaTagEstimate.tagCount == 0) acceptUpdate = false;

    // Case: true
    if (useMegaTag2 && acceptUpdate) {

      LimelightHelpers.SetRobotOrientation(
          networkTableName,
          drivetrainInstance.getHeadingRotation2d().getDegrees(), //maybe change to blue absolute, idk if it changes anything.
          0,
          0,
          0,
          0,
          0);

      if (Math.abs(drivetrainInstance.getRate()) > 720) acceptUpdate = false;

    } else if (!useMegaTag2 && acceptUpdate) {

      if (megaTagEstimate.tagCount == 1
          && megaTagEstimate.rawFiducials.length == 1
          && (megaTagEstimate.rawFiducials[0].ambiguity > .1
              || megaTagEstimate.rawFiducials[0].distToCamera > 3)) acceptUpdate = false;
    }

    if (acceptUpdate) {

      if (useMegaTag2) drivetrainInstance.setVisionMeasurementStdDevs(.7, .7, 9999999);
      else drivetrainInstance.setVisionMeasurementStdDevs(.5, .5, 9999999);

      drivetrainInstance.addVisionMeasurement(
          megaTagEstimate.pose, megaTagEstimate.timestampSeconds);
    }

    return Pair.of(megaTagEstimate, acceptUpdate);
  }
}
