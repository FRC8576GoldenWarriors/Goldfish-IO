package frc.robot.Subsystems.Drivetrain;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public class GyroIOInputs {
    public boolean isConnected = false;
    public double yaw = 0.0;
    public double xVelocity = 0.0;
    public double xAcceleration = 0.0;
    public double yVelocity = 0.0;
    public double yAcceleration = 0.0;
    public double zVelocity = 0.0;
    public double zAcceleration = 0.0;
    public double gyroRate = 0.0;
  }

  default void setYaw(double yaw) {}

  default void updateInputs(GyroIOInputs inputs) {}
}
