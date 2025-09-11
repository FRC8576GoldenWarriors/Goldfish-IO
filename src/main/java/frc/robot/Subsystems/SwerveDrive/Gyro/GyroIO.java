package frc.robot.Subsystems.SwerveDrive.Gyro;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {

  default void updateInputs(GyroIOInputs inputs) {}

  @AutoLog
  public class GyroIOInputs {
    public boolean isConnected = false;

    public double yaw = 0.0;
    public double yawRate = 0.0;

    public double xVelocity = 0.0;
    public double yVelocity = 0.0;
    public double zVelocity = 0.0;

    public double xAcceleration = 0.0;
    public double yAcceleration = 0.0;
    public double zAcceleration = 0.0;

    public boolean resetOccured = false;

    public double supplyVoltage = 0.0;
    public double temperature = 0.0;
  }

  default void setYawDegrees(double yaw) {}
}
