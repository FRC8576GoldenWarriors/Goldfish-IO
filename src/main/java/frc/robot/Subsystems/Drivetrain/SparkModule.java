package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Subsystems.SwerveDrive.SwerveConstants;

public class SparkModule implements ModuleIO {

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

  public SparkModule(
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

    // resetEncoders();
    // last angle in degrees

  }

  @Override
  public void setTurnVoltage(double turnVoltage) {
    turnVoltage = turnMotor.getBusVoltage();
  }

  @Override
  public void setTurnSpeed(double turnSpeed) {
    turnMotor.set(turnSpeed);
  }

  @Override
  public void setDriveVoltage(double driveVoltage) {
    driveMotor.setVoltage(driveVoltage);
  }

  @Override
  public void setDriveSpeed(double driveSpeed) {
    driveMotor.set(driveSpeed);
  }

  @Override
  public void setDriveEncoderPosition(double encoderPosition) {
    driveEncoder.setPosition(encoderPosition);
  }

  @Override
  public void setTurnEncoderPosition(double encoderPosition) {
    turnEncoder.setPosition(encoderPosition);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.turnVoltage = turnMotor.getBusVoltage();
    inputs.turnSpeed = turnEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;
    inputs.driveVoltage = driveMotor.getBusVoltage();
    inputs.driveSpeed = driveEncoder.getVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
    inputs.driveEncoderPosition = driveEncoder.getPosition();
    inputs.turnEncoderPosition = turnEncoder.getPosition();
    inputs.absoluteEncoderPosition = absoluteEncoder.getPosition().getValueAsDouble();
  }
}
