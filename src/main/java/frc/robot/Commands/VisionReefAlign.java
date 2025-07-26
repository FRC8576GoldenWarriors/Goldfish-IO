// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.Limelight;
import frc.robot.Subsystems.Vision.LimelightConstants;
import frc.robot.Subsystems.Vision.LimelightIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionReefAlign extends Command {

  private final Drivetrain drivetrain;
  private final Limelight limelight;
  private final String limelightName = LimelightConstants.NameConstants.REEF_NETWORKTABLE_KEY;

  private final PIDController rotationPID;
  private final PIDController forwardPID;
  private final PIDController strafePID;

  private double driveOutput;
  private double rotationOutput;
  private double strafeOutput;

  private LimelightConstants.reefOffsets offset;

  public VisionReefAlign(
      Drivetrain drivetrain, Limelight limelight, LimelightConstants.reefOffsets offset) {

    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.offset = offset;

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

    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!limelight.hasTargets(limelightName)) return;

    double distanceToTagMeters = limelight.getDistanceToTag(limelightName, true);
    double verticalAngle = limelight.getPitch(limelightName);

    double currentHeading = drivetrain.getHeading();

    double cameraPitchDegrees =
        Units.radiansToDegrees(
            LimelightConstants.PositionalConstants.REEF_LIMELIGHT_LOCATION.getRotation().getY());

    double distanceToWall =
        distanceToTagMeters * Math.cos(Units.degreesToRadians(cameraPitchDegrees + verticalAngle));

    double horizontalAngle = limelight.getYaw(limelightName);

    driveOutput =
        forwardPID.calculate(
            distanceToWall, LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_REEF);

    rotationOutput = rotationPID.calculate(currentHeading, 0);

    double strafeDistance = distanceToWall * Math.tan(Units.degreesToRadians(horizontalAngle));

    double offsetVal;

    switch (offset) {
      case LEFT:
        offsetVal = LimelightConstants.PhysicalConstants.LEFT_STICK_OFFSET;
        break;
      case CENTER:
        offsetVal = 0;
        break;
      case RIGHT:
        offsetVal = LimelightConstants.PhysicalConstants.RIGHT_STICK_OFFSET;
        break;
      default:
        offsetVal = 0;
        break;
    }

    strafeOutput = strafePID.calculate(strafeDistance, offsetVal);

    LimelightIO.isAligned =
        rotationPID.atSetpoint() && forwardPID.atSetpoint() && strafePID.atSetpoint();

    drivetrain.drive(new Translation2d(-driveOutput, strafeOutput), rotationOutput, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightIO.isAligned = false;
    drivetrain.drive(new Translation2d(), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
