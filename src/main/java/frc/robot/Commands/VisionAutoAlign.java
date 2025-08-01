// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.SwerveDrive.SwerveConstants;
import frc.robot.Subsystems.Vision.Limelight;
import frc.robot.Subsystems.Vision.LimelightConstants;
import frc.robot.Subsystems.Vision.LimelightIO;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionAutoAlign extends Command {

  private final Drivetrain drivetrain;
  private final Limelight limelight;
  private final String limelightName = LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY;

  private final ProfiledPIDController rotationPID;
  private final ProfiledPIDController forwardPID;
  private final ProfiledPIDController strafePID;

  private double driveOutput;
  private double rotationOutput;
  private double strafeOutput;

  private Alliance alliance;

  public VisionAutoAlign(Drivetrain drivetrain, Limelight limelight) {

    this.drivetrain = drivetrain;
    this.limelight = limelight;

    rotationPID =
        new ProfiledPIDController(
            LimelightConstants.PIDConstants.rotationkP,
            LimelightConstants.PIDConstants.rotationkI,
            LimelightConstants.PIDConstants.rotationkD,
            new Constraints(
                SwerveConstants.DRIVETRAIN_MAX_ANGULAR_SPEED,
                SwerveConstants.TELE_DRIVE_MAX_ANGULAR_ACCELERATION));
    rotationPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_ANGLE_ERROR);
    rotationPID.enableContinuousInput(-180, 180);

    forwardPID =
        new ProfiledPIDController(
            LimelightConstants.PIDConstants.forwardkP,
            LimelightConstants.PIDConstants.forwardkI,
            LimelightConstants.PIDConstants.forwardkD,
            new Constraints(
                SwerveConstants.DRIVETRAIN_MAX_SPEED, SwerveConstants.TELE_DRIVE_MAX_ACCELERATION));
    forwardPID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_DISTANCE_ERROR);

    strafePID =
        new ProfiledPIDController(
            LimelightConstants.PIDConstants.strafekP,
            LimelightConstants.PIDConstants.strafekI,
            LimelightConstants.PIDConstants.strafekD,
            new Constraints(
                SwerveConstants.DRIVETRAIN_MAX_SPEED, SwerveConstants.TELE_DRIVE_MAX_ACCELERATION));
    strafePID.setTolerance(LimelightConstants.PIDConstants.ALLOWED_STRAFE_ERROR);

    DriverStation.getAlliance()
        .ifPresentOrElse(
            color -> {
              alliance = color;
            },
            () -> {
              alliance = Alliance.Blue;
            });

    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    forwardPID.reset(0);
    strafePID.reset(0);
    rotationPID.reset(drivetrain.getHeading());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!limelight.hasTargets(limelightName)) return;

    int tagID = limelight.getTagID(limelightName);
    // drive
    double distanceToTagMeters = limelight.getDistanceToTag(limelightName, true);
    double verticalAngle = limelight.getPitch(limelightName);
    double cameraPitchDegrees =
        Units.radiansToDegrees(
            LimelightConstants.PositionalConstants.BARGE_LIMELIGHT_LOCATION.getRotation().getY());

    double distanceToWall =
        distanceToTagMeters * Math.cos(Units.degreesToRadians(cameraPitchDegrees + verticalAngle));

    // strafe
    double horizontalAngle = limelight.getYaw(limelightName);
    double strafeDistance = distanceToWall * Math.tan(Units.degreesToRadians(horizontalAngle));

    // strafeOutput = strafePID.calculate(strafeDistance, 0);
    strafeOutput = -RobotContainer.driverController.getLeftX() * 5.5;

    // rotation
    double currentHeading = drivetrain.getHeading();
    switch (alliance) {
      case Blue:
        switch (tagID) {
          case 14:
            driveOutput =
                forwardPID.calculate(
                    distanceToWall,
                    LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_BARGE);
            rotationOutput = rotationPID.calculate(currentHeading, 0);
            break;
          case 4:
            driveOutput =
                forwardPID.calculate(
                    distanceToWall,
                    LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_BARGE);
            rotationOutput = rotationPID.calculate(currentHeading, 180);
            break;
          case 12:
            driveOutput = forwardPID.calculate(distanceToWall, 1.1);
            rotationOutput = rotationPID.calculate(currentHeading, -122);
            strafeOutput = strafePID.calculate(strafeDistance, -0.15);
            break;
          case 13:
            driveOutput = forwardPID.calculate(distanceToWall, 1.1);
            rotationOutput = rotationPID.calculate(currentHeading, 122);
            strafeOutput = strafePID.calculate(strafeDistance, -0.15);
            break;
          default:
            driveOutput = 0;
            rotationOutput = 0;
            strafeOutput = 0;
            break;
        }
        break;

      case Red:
        switch (tagID) {
          case 5:
            driveOutput =
                forwardPID.calculate(
                    distanceToWall,
                    LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_BARGE);
            rotationOutput = rotationPID.calculate(currentHeading, 0);
            break;

          case 15:
            driveOutput =
                forwardPID.calculate(
                    distanceToWall,
                    LimelightConstants.PhysicalConstants.DESIRED_APRIL_TAG_DISTANCE_BARGE);
            rotationOutput = rotationPID.calculate(currentHeading, 180);
            break;

          case 1:
            driveOutput = forwardPID.calculate(distanceToWall, 1.1);
            rotationOutput = rotationPID.calculate(currentHeading, 122);
            strafeOutput = strafePID.calculate(strafeDistance, -0.15);
            break;
          case 2:
            driveOutput = forwardPID.calculate(distanceToWall, 1.1);
            rotationOutput = rotationPID.calculate(currentHeading, -122);
            strafeOutput = strafePID.calculate(strafeDistance, -0.15);
            break;

          default:
            driveOutput = 0;
            rotationOutput = 0;
            strafeOutput = 0;
            break;
        }
        break;

      default:
        driveOutput = 0;
        rotationOutput = 0;
        strafeOutput = 0;
        break;
    }

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
