// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive.Module;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.SwerveDrive.SwerveConstants;
import org.littletonrobotics.junction.Logger;

public class Module extends SubsystemBase {
  /** Creates a new Module. */
  private ModuleIO io;

  private ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private PIDController turnPidController;
  private Rotation2d lastAngle;

  public Module(ModuleIO io) {
    turnPidController = new PIDController(SwerveConstants.KP_TURNING, 0, 0.001);
    turnPidController.enableContinuousInput(-Math.PI, Math.PI);
    lastAngle = getState().angle;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve Module " + inputs.moduleNumebr, inputs);
    // This method will be called once per scheduler run
  }

  public void setDriveState(SwerveModuleState state) {
    io.setDriveSpeed(state.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED);
  }

  public void setTurnState(SwerveModuleState state) {
    Rotation2d angle =
        (Math.abs(state.speedMetersPerSecond) <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01))
            ? lastAngle
            : state.angle;
    io.setTurnSpeed(
        turnPidController.calculate(
            getTurnMotorPosition(),
            state.angle.getRadians())); // CHANGE OUT TO ANGLE.GETRADIANS ONCE WORKING
    lastAngle = angle;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(inputs.driveVelocity, new Rotation2d(inputs.turnEncoderValue));
  }

  public void resetEncoders() {
    io.resetDriveEncoder();
    io.resetTurnEncoder();
  }

  public void stop() {
    io.setDriveVoltage(0);
    io.setTurnVoltage(0);
  }

  public double getDriveMotorPosition() {
    return inputs.driveEncoderValue;
  }

  public double getDriveMotorVelocity() {
    return inputs.driveVelocity;
  }

  public double getTurnMotorPosition() {
    return inputs.turnEncoderValue;
  }

  public double getTurnMotorVelocity() {
    return inputs.turnVelocity;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState wantedState) {
    wantedState = SwerveModuleState.optimize(wantedState, getState().angle);
    // SmartDashboard.putNumber("Post-optimized", desiredState.speedMetersPerSecond);
    setDriveState(wantedState);
    setTurnState(wantedState);
  }
}
