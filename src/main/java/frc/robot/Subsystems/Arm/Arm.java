// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double wantedPosition;
  private double currentPosition;
  private ProfiledPIDController PID;
  private ArmFeedforward FF;
  private double inputVoltage;
  private double PIDVoltage;
  private double FFVoltage;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
    wantedPosition = -1;
    PID =
        new ProfiledPIDController(
            ArmConstants.ControlConstants.kP,
            ArmConstants.ControlConstants.kI,
            ArmConstants.ControlConstants.kD,
            new Constraints(15, 20));
    FF =
        new ArmFeedforward(
            ArmConstants.ControlConstants.kS,
            ArmConstants.ControlConstants.kG,
            ArmConstants.ControlConstants.kV,
            ArmConstants.ControlConstants.kA);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    currentPosition = getEncoderPosition();
    if (wantedPosition >= 0) {
      PIDVoltage = PID.calculate(currentPosition, wantedPosition);
      FFVoltage =
          FF.calculate(
              ((wantedPosition + ArmConstants.ControlConstants.COMOffset - .25) * Math.PI * 2),
              2.0);
      inputVoltage = PIDVoltage + FFVoltage;
      io.setVoltage(inputVoltage);
    }
    // This method will be called once per scheduler run
  }

  public double getVoltage() {
    return inputs.voltage;
  }

  public double getEncoderPosition() {
    return inputs.encoderValue;
  }

  public double getCurrent() {
    return inputs.current;
  }

  public double getVelocity() {
    return inputs.velocity;
  }

  public void setWantedPosition(double wantedPosition) {
    this.wantedPosition = wantedPosition;
  }

  public void setWantedPosition(double wantedPosition, Constraints constraints) {
    this.wantedPosition = wantedPosition;
    PID.setConstraints(constraints);
  }
}
