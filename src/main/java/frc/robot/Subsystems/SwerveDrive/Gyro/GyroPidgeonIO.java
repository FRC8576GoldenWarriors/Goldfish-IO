package frc.robot.Subsystems.SwerveDrive.Gyro;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroPidgeonIO implements GyroIO {
  private Pigeon2 gyro;

  public GyroPidgeonIO() {
    gyro = new Pigeon2(0);
  }

  @Override
  public void setYawDegrees(double yaw) {
    gyro.setYaw(yaw);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = gyro.isConnected();

    inputs.yaw = gyro.getYaw().getValueAsDouble();
    inputs.yawRate = -gyro.getAngularVelocityZWorld().getValueAsDouble();

    inputs.xVelocity = gyro.getAngularVelocityXWorld().getValueAsDouble();
    inputs.yVelocity = gyro.getAngularVelocityYWorld().getValueAsDouble();
    inputs.zVelocity = gyro.getAngularVelocityZWorld().getValueAsDouble();

    inputs.xAcceleration = gyro.getAccelerationX().getValueAsDouble();
    inputs.yAcceleration = gyro.getAccelerationY().getValueAsDouble();
    inputs.zAcceleration = gyro.getAccelerationZ().getValueAsDouble();

    inputs.resetOccured = gyro.hasResetOccurred();

    inputs.supplyVoltage = gyro.getSupplyVoltage().getValueAsDouble();
    inputs.temperature = gyro.getTemperature().getValueAsDouble();
  }
}
