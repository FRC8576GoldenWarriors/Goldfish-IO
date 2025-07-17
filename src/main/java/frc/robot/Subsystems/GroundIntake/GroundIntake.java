// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.GroundIntake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private GroundIntakeIO io;

  private GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();
  private double wantedPosition = GroundIntakeConstants.ControlConstants.groundIntakeUpPosition;
  private double wantedSpeed = 0;
  private PIDController PID;
  private ArmFeedforward FF;
  private double currentPosition;
  private double PIDVoltage;
  private double FFVoltage;
  private double inputVoltage;

  public GroundIntake(GroundIntakeIO io) {
    PID =
        new PIDController(
            GroundIntakeConstants.ControlConstants.kP,
            GroundIntakeConstants.ControlConstants.kI,
            GroundIntakeConstants.ControlConstants.kD);
    FF =
        new ArmFeedforward(
            GroundIntakeConstants.ControlConstants.kS,
            GroundIntakeConstants.ControlConstants.kG,
            GroundIntakeConstants.ControlConstants.kV,
            GroundIntakeConstants.ControlConstants.kA);
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Ground Intake", inputs);

      PIDVoltage = -PID.calculate(getPosition(), wantedPosition);
      FFVoltage =
          FF.calculate(
              (-wantedPosition + 0.25) * Math.PI * 2, 2.0); // position in radians, 0 is horizontal
      inputVoltage = PIDVoltage + FFVoltage;

      Logger.recordOutput("Wanted Position", wantedPosition);
      Logger.recordOutput("Wanted Speed", wantedSpeed);
      Logger.recordOutput("PID Setpoint", PID.getSetpoint());
      Logger.recordOutput("PID Voltage", PIDVoltage);
      io.setPivotVoltage(inputVoltage);
      io.setRollerSpeed(wantedSpeed);
    // This method will be called once per scheduler run
  }

  public void setGroundIntake(double position, double speed) {
    wantedPosition = position;
    wantedSpeed = speed;
  }

  public double getPosition() {
    return inputs.encoderPosition;
  }

  public double getSpeed() {
    return inputs.rollerSpeed;
  }
}
