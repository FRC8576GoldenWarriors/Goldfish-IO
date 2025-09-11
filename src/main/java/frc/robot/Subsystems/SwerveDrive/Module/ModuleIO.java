package frc.robot.Subsystems.SwerveDrive.Module;

import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  default void updateInputs(ModuleIOInputs inputs) {}

  @AutoLog
  public class ModuleIOInputs {
    public double driveVoltage = 0.0;
    public double turnVoltage = 0.0;

    public double driveCurrent = 0.0;
    public double turnCurrent = 0.0;

    public double absEncoderValue = 0.0;
    public boolean absEncoderConnected = false;

    public double driveEncoderValue = 0.0;
    public double turnEncoderValue = 0.0;

    public double driveVelocity = 0.0;
    public double turnVelocity = 0.0;

    public int moduleNumebr = 0;
  }

  default void setDriveVoltage(double voltage) {}

  default void setTurnVoltage(double voltage) {}

  default void setDriveSpeed(double speed) {}

  default void setTurnSpeed(double speed) {}

  default void resetDriveEncoder() {}

  default void resetTurnEncoder() {}
}
