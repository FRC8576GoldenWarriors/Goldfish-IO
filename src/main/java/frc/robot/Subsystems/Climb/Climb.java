// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Climb;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  /** Creates a new Climb. */
  private ClimbIO io;

  private ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();
  private double climbAngle = -1;
  private double motorOutput;

  public Climb(ClimbIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb", inputs);
    if(DriverStation.isEnabled()){
    if (RobotContainer.driverController.povUp().getAsBoolean()) {
      io.setSpeed(0.9);
    } else if (RobotContainer.driverController.povDown().getAsBoolean()) {
      io.setSpeed(-0.9);
    } 
    else  {
      if(climbAngle >= 0) {
      if (getPosition() < climbAngle) { // pivot going down, robot climbing up
        motorOutput = 1.0;
        io.setSpeed(motorOutput);

        if (Math.abs(getPosition() - climbAngle) < 0.005) {
          motorOutput = 0.0;
          io.setSpeed(motorOutput);
        }
      } else { // pivot going up, robot going down
        motorOutput = -1.0;
        io.setSpeed(motorOutput);
        if (Math.abs(getPosition() - climbAngle) < 0.005) {
          motorOutput = 0.0;
          io.setSpeed(motorOutput);
        }
      }}
      else{
        motorOutput = 0.0;
        io.setSpeed(motorOutput);
      }

      if (getPosition() > 0.26) {
        motorOutput = 0.0;
        io.setSpeed(motorOutput);
      }
    }
  }
  else{
    setClimbAngle(-1);
  }

      
    Logger.recordOutput("Climb/Wanted Angle", climbAngle);
    // This method will be called once per scheduler run
  }

  public void setClimbAngle(double climbAngle) {
    this.climbAngle = climbAngle;
  }

  public double getPosition() {
    return inputs.climbPosition;
  }

  public double getVoltage() {
    return inputs.voltage;
  }

  public double getCurrent() {
    return inputs.current;
  }

  public double getRelativePosition() {
    return inputs.relativeEncoderPosition;
  }

  public boolean nearSetpoint() {
    return Math.abs(getPosition() - climbAngle) <= 0.005;
  }
}
