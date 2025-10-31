// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.PoseBasedAligns;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.Limelight.Limelight;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants;
import frc.robot.Subsystems.Vision.Limelight.LimelightIO;
import frc.robot.Subsystems.Vision.TagMap;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BargeAlign extends Command {

  private final PIDController rotationPID;
  private final PIDController forwardPID;
  private final PIDController strafePID;

  private Drivetrain drivetrain;
  private TagMap map;
  private Limelight limelight;

  public BargeAlign(Drivetrain drivetrain, Limelight limelight, TagMap map) {
    rotationPID =
        new PIDController(
            LimelightConstants.PIDConstants.ROTATION_KP,
            LimelightConstants.PIDConstants.ROTATION_KI,
            LimelightConstants.PIDConstants.ROTATION_KD);
    rotationPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_ANGLE_ERROR);
    rotationPID.enableContinuousInput(-180, 180);

    forwardPID =
        new PIDController(
            LimelightConstants.PIDConstants.FORWARD_KP,
            LimelightConstants.PIDConstants.FORWARD_KI,
            LimelightConstants.PIDConstants.FORWARD_KD);
    forwardPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_DISTANCE_ERROR);

    strafePID =
        new PIDController(
            LimelightConstants.PIDConstants.STRAFE_KP,
            LimelightConstants.PIDConstants.STRAFE_KI,
            LimelightConstants.PIDConstants.STRAFE_KD);
    strafePID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_STRAFE_ERROR);

    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.map = map;
    // lalitha is the best media markeitng officer ever
    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardPID.reset();
    strafePID.reset();
    rotationPID.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d drivePose = drivetrain.getPose();

    Pose2d bargeAlignPose =
        map.getBargeAlignmentPose(
            limelight.getTagID(LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY),
            LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_BARGE,
            drivePose);

    if (bargeAlignPose == null) {
      return;
    }
    // strafePID.calculate(drivePose.getY(), bargeAlignPose.getY());
    double strafeOutput =
        -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.03)
            * LimelightConstants.PIDConstants
                .STRAFE_MULTIPLIER; // -RobotContainer.driverController.getLeftX() * 5.5;

    if (strafeOutput > 0) {
      rotationPID.setP(
          LimelightConstants.PIDConstants.ROTATION_KP
              + LimelightConstants.PIDConstants.ROTATION_DRIFT_CORRECTION);
      forwardPID.setP(
          LimelightConstants.PIDConstants.FORWARD_KP
              + LimelightConstants.PIDConstants.ROTATION_DRIFT_CORRECTION);
    }

    double forwardOutput = forwardPID.calculate(drivePose.getX(), bargeAlignPose.getX());
    double rotationOutput =
        rotationPID.calculate(
            drivePose.getRotation().getDegrees(), bargeAlignPose.getRotation().getDegrees());

    drivetrain.drive(new Translation2d(forwardOutput, strafeOutput), rotationOutput, true, true);

    if (strafeOutput > 0) {
      rotationPID.setP(LimelightConstants.PIDConstants.ROTATION_KP);
      forwardPID.setP(LimelightConstants.PIDConstants.FORWARD_KP);
    }

    if (forwardPID.atSetpoint() && rotationPID.atSetpoint()) {
      LimelightIO.AlignedVar = true;
    }
    Logger.recordOutput("Barge Align/Forward Output", forwardOutput);
    Logger.recordOutput("Barge Align/Strafe Output", strafeOutput);
    Logger.recordOutput("Barge Align/Rotation Output", rotationOutput);
    Logger.recordOutput("Barge Align/Align Distance", bargeAlignPose.getX());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightIO.AlignedVar = false;
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
