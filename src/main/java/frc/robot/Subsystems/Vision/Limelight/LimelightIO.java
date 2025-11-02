// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision.Limelight;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.Limelight.LimelightHelpers.PoseEstimate;
import java.util.ArrayList;
import java.util.Collections;

public class LimelightIO implements LimelightVisionIO {
  private String networkTableName;
  private static Drivetrain drivetrainInstance = RobotContainer.m_Drivetrain;
  public static boolean AlignedVar = false;

  public LimelightIO(String networkTableName) {
    this.networkTableName = networkTableName;
    drivetrainInstance.poseEstimator.addPoseEstimateUpdaters(this::getPairPoseEstimate);
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
      inputs.pitch = LimelightHelpers.getTY(networkTableName);
      inputs.yaw = LimelightHelpers.getTX(networkTableName);
      inputs.area = LimelightHelpers.getTA(networkTableName);

      inputs.timeInSeconds = Timer.getFPGATimestamp();
      var speeds = drivetrainInstance.getRobotRelativeSpeeds();
      inputs.velocity =
          Math.sqrt(Math.pow(speeds.vxMetersPerSecond, 2) + Math.pow(speeds.vyMetersPerSecond, 2));

      inputs.generatedStddevs =
          NetworkTableInstance.getDefault()
              .getTable("limelight")
              .getEntry("stddevs")
              .getDoubleArray(new double[12]);
    }

    // this.setDynamicCrop();
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

  public PoseEstimate getPoseEstimate() {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(networkTableName);
  }

  public Pair<String, PoseEstimate> getPairPoseEstimate() {
    return Pair.of(
        this.networkTableName,
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(networkTableName));
  }

  public void setRobotOrientation(double yaw) {
    LimelightHelpers.SetRobotOrientation(networkTableName, yaw, 0, 0, 0, 0, 0);
  }

  public static void setRobotOrientation(String limelightName, double yaw) {
    LimelightHelpers.SetRobotOrientation(limelightName, yaw, 0, 0, 0, 0, 0);
  }



  public static boolean isBlueAlliance() {
    return DriverStation.getAlliance().get() == Alliance.Blue;
  }
}
