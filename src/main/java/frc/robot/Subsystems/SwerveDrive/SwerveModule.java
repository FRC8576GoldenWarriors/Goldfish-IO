// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.drivers.WarriorSparkMax;
import org.littletonrobotics.junction.Logger;

public class SwerveModule extends SubsystemBase {

  private int turnMotorId;
  private int driveMotorId;

  private WarriorSparkMax driveMotor;
  private WarriorSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private PIDController turnPIDController;
  private CANcoder absoluteEncoder;

  private double absoluteEncoderOffset;
  private Rotation2d lastAngle;

  /** Creates a new SwerveModule. */
  public SwerveModule(
      int driveMotorId,
      int turnMotorId,
      boolean driveMotorReversed,
      boolean turnMotorReversed,
      int absoluteEncoderId,
      double absoluteEncoderOffset) {
    this.absoluteEncoderOffset = absoluteEncoderOffset;

    this.turnMotorId = turnMotorId;
    this.driveMotorId = driveMotorId;

    driveMotor =
        new WarriorSparkMax(
            driveMotorId,
            MotorType.kBrushless,
            driveMotorReversed,
            IdleMode.kBrake,
            SwerveConstants.DRIVE_CURRENT_LIMIT);
    turnMotor =
        new WarriorSparkMax(
            turnMotorId,
            MotorType.kBrushless,
            turnMotorReversed,
            IdleMode.kBrake,
            SwerveConstants.ROTATION_CURRENT_LIMIT);

    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();

    absoluteEncoder = new CANcoder(absoluteEncoderId);

    turnPIDController = new PIDController(SwerveConstants.KP_TURNING, 0, 0.001);
    turnPIDController.enableContinuousInput(-Math.PI, Math.PI);

    resetEncoders();
    // last angle in degrees
    lastAngle = getState().angle;
  }

  public double getDriveMotorVoltage() {
    return driveMotor.getBusVoltage();
  }

  public double getTurnMotorVoltage() {
    return turnMotor.getBusVoltage();
  }

  public double getDriveMotorCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public double getTurnMotorCurrent() {
    return turnMotor.getOutputCurrent();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber(turnMotorId+"", turnMotor.getEncoder().getPosition());
    // SmartDashboard.putNumber(driveMotorId+" Current: ", driveMotor.getOutputCurrent() );
    Logger.recordOutput(
        "Drive Motor " + absoluteEncoder.getDeviceID() + " Voltage", getDriveMotorVoltage());
    Logger.recordOutput(
        "Turn Motor " + absoluteEncoder.getDeviceID() + " Voltage", getTurnMotorVoltage());
    Logger.recordOutput(
        "Drive Motor " + absoluteEncoder.getDeviceID() + " Current", getDriveMotorCurrent());
    Logger.recordOutput(
        "Turn Motor " + absoluteEncoder.getDeviceID() + " Current", getTurnMotorCurrent());
  }

  public void setBrake(boolean brake) {
    if (brake) {
      driveMotor.setIdleMode(IdleMode.kBrake);
      turnMotor.setIdleMode(IdleMode.kBrake);
    } else {
      driveMotor.setIdleMode(IdleMode.kCoast);
      turnMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public double getDriveMotorPosition() {
    return driveEncoder.getPosition() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
  }

  public double getDriveMotorVelocity() {
    return driveEncoder.getVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
  }

  public double getTurnMotorPosition() {
    return turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;
  }

  public double getTurnMotorVelocity() {
    return turnEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
  }

  // returning rotations
  public double getAbsoluteEncoderAngle() {
    double angle = absoluteEncoder.getAbsolutePosition().getValueAsDouble();
    angle -= absoluteEncoderOffset;
    angle *= (Math.PI * 2);
    return angle;
  }

  public void resetEncoders() {
    driveEncoder.setPosition(0);
    turnEncoder.setPosition((getAbsoluteEncoderAngle() / SwerveConstants.TURN_MOTOR_PCONVERSION));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMotorVelocity(), new Rotation2d(getTurnMotorPosition()));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveMotorPosition(), new Rotation2d(getTurnMotorPosition()));
  }

  public void setDesiredState(SwerveModuleState desiredState) {
    // SmartDashboard.putNumber("Pre-optimized", desiredState.speedMetersPerSecond);
    desiredState = SwerveModuleState.optimize(desiredState, getState().angle);
    // SmartDashboard.putNumber("Post-optimized", desiredState.speedMetersPerSecond);
    setAngle(desiredState);
    setSpeed(desiredState);
    // SmartDashboard.putString("Swerve [" + driveMotor.getDeviceId() + "] State",
    // getState().toString());
  }

  public void setSpeed(SwerveModuleState desiredState) {
    driveMotor.set(desiredState.speedMetersPerSecond / SwerveConstants.DRIVETRAIN_MAX_SPEED);
  }

  public void setAngle(SwerveModuleState desiredState) {
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond)
                <= (SwerveConstants.DRIVETRAIN_MAX_SPEED * 0.01))
            ? lastAngle
            : desiredState.angle;

    turnMotor.set(
        turnPIDController.calculate(getTurnMotorPosition(), desiredState.angle.getRadians()));

    lastAngle = angle;
  }

  public void stop() {
    driveMotor.set(0);
    turnMotor.set(0);
  }
}
