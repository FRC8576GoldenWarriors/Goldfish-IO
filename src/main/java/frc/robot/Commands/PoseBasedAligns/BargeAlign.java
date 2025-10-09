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
import frc.robot.Subsystems.Vision.TagMap;

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
            LimelightConstants.PIDConstants.rotationkP,
            LimelightConstants.PIDConstants.rotationkI,
            LimelightConstants.PIDConstants.rotationkD);
    rotationPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_ANGLE_ERROR);
    rotationPID.enableContinuousInput(-180, 180);

    forwardPID =
        new PIDController(
            LimelightConstants.PIDConstants.forwardkP,
            LimelightConstants.PIDConstants.forwardkI,
            LimelightConstants.PIDConstants.forwardkD);
    forwardPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_DISTANCE_ERROR);

    strafePID =
        new PIDController(
            LimelightConstants.PIDConstants.strafekP,
            LimelightConstants.PIDConstants.strafekI,
            LimelightConstants.PIDConstants.strafekD);
    strafePID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_STRAFE_ERROR);

    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.map = map;

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

    // strafePID.calculate(drivePose.getY(), bargeAlignPose.getY());
    double strafeOutput =
        -MathUtil.applyDeadband(RobotContainer.driverController.getLeftX(), 0.03)
            * 4.5; // -RobotContainer.driverController.getLeftX() * 5.5;

    double forwardOutput = forwardPID.calculate(drivePose.getX(), bargeAlignPose.getX());
    double rotationOutput =
        rotationPID.calculate(
            drivePose.getRotation().getDegrees(), bargeAlignPose.getRotation().getDegrees());

    drivetrain.drive(new Translation2d(forwardOutput, strafeOutput), rotationOutput, true, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
