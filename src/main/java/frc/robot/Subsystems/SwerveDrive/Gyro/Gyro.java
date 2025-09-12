// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.SwerveDrive.Gyro;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Gyro extends SubsystemBase {
  /** Creates a new Gyro. */
  private GyroIOInputsAutoLogged inputs = new GyroIOInputsAutoLogged();

  private GyroIO io;

  public Gyro(GyroIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Gyro", inputs);
    // This method will be called once per scheduler run
  }

  public double getYaw() {
    return inputs.yaw;
  }

  public double getRotationVel() {
    return inputs.zVelocity;
  }

  public double getRate() {
    return inputs.yawRate;
  }

  public double getForwardVelocity() {
    return inputs.xVelocity;
  }

  public void setYawDegrees(double yaw) {
    io.setYawDegrees(yaw);
  }

  public void zero() {
    io.setYawDegrees(0);
  }
}
