package frc.robot.Subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {

  @AutoLog
  public class ModuleIOInputs {
    public double turnVoltage = 0.0;
    public double turnSpeed = 0.0;
    public double driveVoltage = 0.0;
    public double driveSpeed = 0.0;
    public double driveEncoderPosition = 0.0;
    public double turnEncoderPosition = 0.0;
  }

  default void setTurnVoltage(double turnVoltage) {}

  default void setTurnSpeed(double turnSpeed) {}

  default void setDriveVoltage(double driveVoltage) {}

  default void setDriveSpeed(double driveSpeed) {}

  default void setDriveEncoderPosition(double endcoderPosition) {}

  default void setTurnEncoderPosition(double endcoderPosition) {}

  default void updateInputs(ModuleIOInputs inputs) {}
}
