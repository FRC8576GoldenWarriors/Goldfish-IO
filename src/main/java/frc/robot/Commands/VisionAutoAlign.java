// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.SwerveDrive.Drivetrain;
import frc.robot.Subsystems.Vision.Limelight.Limelight;
import frc.robot.Subsystems.Vision.Limelight.LimelightConstants;
import frc.robot.Subsystems.Vision.Limelight.LimelightIO;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class VisionAutoAlign extends Command {

  private final Drivetrain drivetrain;
  private final Limelight limelight;
  private final String limelightName = LimelightConstants.NameConstants.BARGE_NETWORKTABLE_KEY;

  private final PIDController rotationPID;
  private final PIDController forwardPID;
  private final PIDController strafePID;

  private double driveOutput;
  private double rotationOutput;
  private double strafeOutput;

  private Alliance alliance;

  public VisionAutoAlign(Drivetrain drivetrain, Limelight limelight) {

    this.drivetrain = drivetrain;
    this.limelight = limelight;

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
  public void initialize() {
    alliance = DriverStation.getAlliance().get();

    Logger.recordOutput("Allinace Color", alliance.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!limelight.hasTargets(limelightName))
      return; // || limelight.getTimeBetweenTagSighting(limelightName) > 0.06) return;

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

    // Logger.recordOutput("Barge/Rotation PID At Goal", rotationPID.atGoal());
    // Logger.recordOutput("Barge/Forward PID At Goal", forwardPID.atGoal());
    // Logger.recordOutput("Barge/Forward PID Setpoint", forwardPID.getSetpoint().position);
    // Logger.recordOutput("Barge/Rotation PID Setpoint",rotationPID.getSetpoint().position);
    // Logger.recordOutput("Barge/Forward PID Goal", forwardPID.getGoal().position);
    // Logger.recordOutput("Barge/Rotation PID Goal",rotationPID.getGoal().position);

    Logger.recordOutput("Forward Pid", driveOutput);
    Logger.recordOutput("Rotation Pid", rotationOutput);
    Logger.recordOutput("Strafe Pid", strafeOutput);

    Logger.recordOutput("Forward Pid Setpoint", forwardPID.atSetpoint());
    Logger.recordOutput("Rotation Pid Setpoint", rotationPID.atSetpoint());
    Logger.recordOutput("Strafe Pid Setpoint", strafePID.atSetpoint());

    Logger.recordOutput(
        "Command Align Status", forwardPID.atSetpoint() && rotationPID.atSetpoint());

    LimelightIO.AlignedVar = forwardPID.atSetpoint() && rotationPID.atSetpoint();

    drivetrain.drive(new Translation2d(-driveOutput, strafeOutput), rotationOutput, false, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimelightIO.AlignedVar = false;
    drivetrain.drive(new Translation2d(), 0, false, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
