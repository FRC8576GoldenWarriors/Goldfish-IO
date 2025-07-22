// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.GroundIntake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class GroundIntake extends SubsystemBase {
  /** Creates a new GroundIntake. */
  private GroundIntakeIO io;

  public enum GroundIntakeStates {
    Intake,
    Hold,
    Rest,
    Outtake,
    Idle,
    Shoot
  }

  private GroundIntakeIOInputsAutoLogged inputs = new GroundIntakeIOInputsAutoLogged();
  private GroundIntakeStates wantedState = GroundIntakeStates.Idle;
  private PIDController PID;
  private ArmFeedforward FF;
  private double currentPosition;
  private double wantedSpeed;
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
    currentPosition = getPosition();
    if (DriverStation.isEnabled()) {
        switch (wantedState) {
          case Intake:
            PIDVoltage =
                -PID.calculate(
                    currentPosition, GroundIntakeConstants.ControlConstants.algaeInPosition);
            FFVoltage =
                FF.calculate(
                    (-GroundIntakeConstants.ControlConstants.algaeInPosition + 0.25) * Math.PI * 2,
                    2.0);
            wantedSpeed = GroundIntakeConstants.ControlConstants.algaeInSpeed;
            break;
          case Hold:
            PIDVoltage =
                -PID.calculate(
                    currentPosition, GroundIntakeConstants.ControlConstants.algaeHoldPosition);
            FFVoltage =
                FF.calculate(
                    (-GroundIntakeConstants.ControlConstants.algaeHoldPosition + 0.25)
                        * Math.PI
                        * 2,
                    2.0);
            wantedSpeed = 0;
            break;
          case Rest:
            PIDVoltage =
                -PID.calculate(
                    currentPosition, GroundIntakeConstants.ControlConstants.groundIntakeUpPosition);
            FFVoltage =
                FF.calculate(
                    (-GroundIntakeConstants.ControlConstants.groundIntakeUpPosition + 0.25)
                        * Math.PI
                        * 2,
                    2.0);
            wantedSpeed = 0;
            break;
          case Outtake:
            PIDVoltage =
                -PID.calculate(
                    currentPosition, GroundIntakeConstants.ControlConstants.groundIntakeUpPosition);
            FFVoltage =
                FF.calculate(
                    (-GroundIntakeConstants.ControlConstants.groundIntakeUpPosition + 0.25)
                        * Math.PI
                        * 2,
                    2.0);
            wantedSpeed = GroundIntakeConstants.ControlConstants.algaeOutSpeed;
            if(!getAlgaeDetected()){
              wantedState = GroundIntakeStates.Rest;
            }
            break;
          case Shoot:
          PIDVoltage =
                -PID.calculate(
                    currentPosition, GroundIntakeConstants.ControlConstants.algaeHoldPosition);
            FFVoltage =
                FF.calculate(
                    (-GroundIntakeConstants.ControlConstants.algaeHoldPosition + 0.25)
                        * Math.PI
                        * 2,
                    2.0);
            wantedSpeed = GroundIntakeConstants.ControlConstants.algaeShootSpeed;
            break;
          case Idle:
            PIDVoltage = 0;
            FFVoltage = 0;
            wantedSpeed = 0.0;
            break;
          default:
            PIDVoltage = 0;
            FFVoltage = 0;
            wantedSpeed = 0.0;
        }
      }
    else {
      wantedState = GroundIntakeStates.Idle;
    }
    inputVoltage = PIDVoltage + FFVoltage;

    Logger.recordOutput("Wanted State", wantedState);
    Logger.recordOutput("Wanted Speed", wantedSpeed);
    Logger.recordOutput("PID Setpoint", PID.getSetpoint());
    Logger.recordOutput("PID Voltage", PIDVoltage);
    io.setPivotVoltage(inputVoltage);
    io.setRollerSpeed(wantedSpeed);
    // This method will be called once per scheduler run
  }

  public void setWantedState(GroundIntakeStates wantedState) {
    this.wantedState = wantedState;
  }

  public double getPosition() {
    return inputs.encoderPosition;
  }

  public double getSpeed() {
    return inputs.rollerSpeed;
  }

  public GroundIntakeStates getState() {
    return wantedState;
  }
  public boolean getAlgaeDetected(){
    return inputs.algaeDetected;
  }
}
