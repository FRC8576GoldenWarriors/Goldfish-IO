package frc.robot.Subsystems.SwerveDrive.Module;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.lib.drivers.WarriorSparkMax;
import frc.robot.Subsystems.SwerveDrive.*;

public class ModuleIOSparkMax implements ModuleIO {
  private WarriorSparkMax driveMotor;
  private WarriorSparkMax turnMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder turnEncoder;

  private double absEncoderOffset;

  private int moduleNum;
  private CANcoder absEncoder;

  public ModuleIOSparkMax(int moduleNum) {
    this.moduleNum = moduleNum;
    switch (moduleNum) {
      case 1:
        driveMotor =
            new WarriorSparkMax(
                SwerveConstants.LEFT_FRONT_DRIVE_ID,
                MotorType.kBrushless,
                SwerveConstants.LEFT_FRONT_DRIVE_INVERTED,
                IdleMode.kBrake);
        turnMotor =
            new WarriorSparkMax(
                SwerveConstants.LEFT_FRONT_TURN_ID,
                MotorType.kBrushless,
                SwerveConstants.LEFT_FRONT_TURN_INVERTED,
                IdleMode.kBrake);
        absEncoder = new CANcoder(SwerveConstants.LEFT_FRONT_CANCODER_ID);
        absEncoderOffset = SwerveConstants.LEFT_FRONT_OFFSET;
        break;
      case 2:
        driveMotor =
            new WarriorSparkMax(
                SwerveConstants.RIGHT_FRONT_DRIVE_ID,
                MotorType.kBrushless,
                SwerveConstants.RIGHT_FRONT_DRIVE_INVERTED,
                IdleMode.kBrake);
        turnMotor =
            new WarriorSparkMax(
                SwerveConstants.RIGHT_FRONT_TURN_ID,
                MotorType.kBrushless,
                SwerveConstants.RIGHT_FRONT_TURN_INVERTED,
                IdleMode.kBrake);
        absEncoder = new CANcoder(SwerveConstants.RIGHT_FRONT_CANCODER_ID);
        absEncoderOffset = SwerveConstants.RIGHT_FRONT_OFFSET;
        break;
      case 3:
        driveMotor =
            new WarriorSparkMax(
                SwerveConstants.LEFT_BACK_DRIVE_ID,
                MotorType.kBrushless,
                SwerveConstants.LEFT_BACK_DRIVE_INVERTED,
                IdleMode.kBrake);
        turnMotor =
            new WarriorSparkMax(
                SwerveConstants.LEFT_BACK_TURN_ID,
                MotorType.kBrushless,
                SwerveConstants.LEFT_BACK_TURN_INVERTED,
                IdleMode.kBrake);
        absEncoder = new CANcoder(SwerveConstants.LEFT_BACK_CANCODER_ID);
        absEncoderOffset = SwerveConstants.LEFT_BACK_OFFSET;
        break;
      case 4:
        driveMotor =
            new WarriorSparkMax(
                SwerveConstants.RIGHT_BACK_DRIVE_ID,
                MotorType.kBrushless,
                SwerveConstants.RIGHT_BACK_DRIVE_INVERTED,
                IdleMode.kBrake);
        turnMotor =
            new WarriorSparkMax(
                SwerveConstants.RIGHT_BACK_TURN_ID,
                MotorType.kBrushless,
                SwerveConstants.RIGHT_BACK_TURN_INVERTED,
                IdleMode.kBrake);
        absEncoder = new CANcoder(SwerveConstants.RIGHT_BACK_CANCODER_ID);
        absEncoderOffset = SwerveConstants.RIGHT_BACK_OFFSET;
        break;
      default:
        break;
    }
    driveEncoder = driveMotor.getEncoder();
    turnEncoder = turnMotor.getEncoder();
  }

  @Override
  public void setDriveSpeed(double speed) {
    driveMotor.set(speed);
  }

  @Override
  public void setTurnSpeed(double speed) {
    turnMotor.set(speed);
  }

  @Override
  public void setDriveVoltage(double voltage) {
    driveMotor.setVoltage(voltage);
  }

  @Override
  public void setTurnVoltage(double voltage) {
    turnMotor.setVoltage(voltage);
  }

  @Override
  public void resetDriveEncoder() {
    driveEncoder.setPosition(0);
  }

  @Override
  public void resetTurnEncoder() {
    turnEncoder.setPosition(getAbsPosition() / SwerveConstants.TURN_MOTOR_PCONVERSION);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.driveVoltage = driveMotor.getAppliedOutput();
    inputs.turnVoltage = turnMotor.getAppliedOutput();

    inputs.driveCurrent = driveMotor.getOutputCurrent();
    inputs.turnCurrent = turnMotor.getOutputCurrent();

    inputs.absEncoderValue = getAbsPosition() / SwerveConstants.TURN_MOTOR_PCONVERSION;
    inputs.absEncoderConnected = absEncoder.isConnected();

    inputs.driveEncoderValue = driveEncoder.getPosition() * SwerveConstants.DRIVE_MOTOR_PCONVERSION;
    inputs.turnEncoderValue = turnEncoder.getPosition() * SwerveConstants.TURN_MOTOR_PCONVERSION;

    inputs.driveVelocity = driveEncoder.getVelocity() * SwerveConstants.DRIVE_MOTOR_VCONVERSION;
    inputs.turnVelocity = driveEncoder.getVelocity() * SwerveConstants.TURN_MOTOR_VCONVERSION;

    inputs.moduleNumebr = moduleNum;
  }

  public double getAbsPosition() {
    double angle = absEncoder.getAbsolutePosition().getValueAsDouble();
    angle -= absEncoderOffset;
    angle *= (Math.PI * 2);
    return angle;
  }
}
