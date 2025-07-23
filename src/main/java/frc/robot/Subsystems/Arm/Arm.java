// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private ArmIO io;
  private ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();
  private double currentPosition;
  private ProfiledPIDController PID;
  private ArmFeedforward FF;
  private double inputVoltage;
  private double PIDVoltage;
  private double FFVoltage;

  public enum ArmPositions {
    A1,
    A2,
    Holding,
    Handoff,
    Idle,
    L1,
    L2,
    L3,
    Station,
    Lolipop
  }

  private ArmPositions wantedPosition;

  /** Creates a new Arm. */
  public Arm(ArmIO io) {
    this.io = io;
    wantedPosition = ArmPositions.Idle;
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
    Logger.recordOutput("Arm/Wanted State", wantedPosition);
    currentPosition = getEncoderPosition();
    if (DriverStation.isEnabled()) {
      switch (wantedPosition) {
        case Lolipop:
          PIDVoltage =
              PID.calculate(currentPosition, ArmConstants.ControlConstants.LolipopPosition);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.LolipopPosition
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case A1:
          PIDVoltage = PID.calculate(currentPosition, ArmConstants.ControlConstants.A1Position);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.A1Position
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case A2:
          PIDVoltage = PID.calculate(currentPosition, ArmConstants.ControlConstants.A2Position);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.A2Position
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case Holding:
          PIDVoltage = PID.calculate(currentPosition, ArmConstants.ControlConstants.storedPosition);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.storedPosition
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case Handoff:
          PIDVoltage =
              PID.calculate(currentPosition, ArmConstants.ControlConstants.handoffPosition);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.handoffPosition
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case L1:
          PIDVoltage = PID.calculate(currentPosition, ArmConstants.ControlConstants.L1Position);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.L1Position
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case L2:
          PIDVoltage = PID.calculate(currentPosition, ArmConstants.ControlConstants.L2Position);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.L2Position
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case L3:
          PIDVoltage = PID.calculate(currentPosition, ArmConstants.ControlConstants.L3Position);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.L3Position
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case Station:
          PIDVoltage =
              PID.calculate(currentPosition, ArmConstants.ControlConstants.coralStationPosition);
          FFVoltage =
              FF.calculate(
                  (ArmConstants.ControlConstants.coralStationPosition
                          + ArmConstants.ControlConstants.COMOffset
                          - .25)
                      * Math.PI
                      * 2,
                  2.0);
          break;
        case Idle:
          PIDVoltage = 0;
          FFVoltage = 0;
          break;
        default:
          PIDVoltage = 0;
          FFVoltage = 0;
      }
      Logger.recordOutput("Arm/PID Voltage", PIDVoltage);
      Logger.recordOutput("Arm/FF Voltage", FFVoltage);
      Logger.recordOutput("Arm/PID Setpoint", PID.getSetpoint().position);
      Logger.recordOutput("Arm/Near Setpoint", armNearPosition());
      inputVoltage = PIDVoltage + FFVoltage;
      io.setVoltage(inputVoltage);
    } else {
      wantedPosition = ArmPositions.Idle;
    }
  }

  // This method will be called once per scheduler run

  public boolean armNearPosition() {
    return Math.abs(getEncoderPosition() - PID.getGoal().position) < 0.015;
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

  public ArmPositions getPosition() {
    return wantedPosition;
  }

  public void setWantedPosition(ArmPositions wantedPosition) {
    if (wantedPosition.equals(ArmPositions.L1)
        || wantedPosition.equals(ArmPositions.L2)
        || wantedPosition.equals(ArmPositions.L3)) {
      PID.setConstraints(new Constraints(0.5, 1));
    } else {
      PID.setConstraints(new Constraints(15, 20));
    }
    this.wantedPosition = wantedPosition;
  }

  public void setWantedPosition(ArmPositions wantedPosition, Constraints constraints) {
    this.wantedPosition = wantedPosition;
    PID.setConstraints(constraints);
  }
}
