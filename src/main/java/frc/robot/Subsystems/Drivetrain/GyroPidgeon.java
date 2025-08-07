package frc.robot.Subsystems.Drivetrain;

import com.ctre.phoenix6.hardware.Pigeon2;

public class GyroPidgeon implements GyroIO {
  private Pigeon2 gyro;

  public GyroPidgeon() {
    gyro = new Pigeon2(0);
  }

  @Override
  public void setYaw(double heading) {
    gyro.setYaw(heading);
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.isConnected = gyro.isConnected();
    inputs.yaw = (Math.IEEEremainder(gyro.getYaw().getValueAsDouble(), 360));
    inputs.xVelocity = gyro.getAngularVelocityXDevice().getValueAsDouble();
    inputs.xAcceleration = gyro.getAccelerationX().getValueAsDouble();
    inputs.yVelocity = gyro.getAngularVelocityYDevice().getValueAsDouble();
    inputs.yAcceleration = gyro.getAccelerationY().getValueAsDouble();
    inputs.zVelocity = gyro.getAngularVelocityZDevice().getValueAsDouble();
    inputs.zAcceleration = gyro.getAccelerationZ().getValueAsDouble();
    inputs.gyroRate = gyro.getAngularVelocityZWorld().getValueAsDouble();
  }
}
