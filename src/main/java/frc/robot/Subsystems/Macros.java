// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.config.RobotConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Arm.Arm;
import frc.robot.Subsystems.Arm.Arm.ArmPositions;
import frc.robot.Subsystems.Climb.Climb;
import frc.robot.Subsystems.EndEffector.EndEffector;
import frc.robot.Subsystems.EndEffector.EndEffector.EndEffectorStates;
import frc.robot.Subsystems.GroundIntake.GroundIntake;
import frc.robot.Subsystems.GroundIntake.GroundIntake.GroundIntakeStates;
import frc.robot.Subsystems.Shintake.Shintake;
import frc.robot.Subsystems.Shintake.Shintake.ShintakeStates;

public class Macros extends SubsystemBase {
  private Arm m_Arm;
  private Climb m_Climb;
  private EndEffector m_EndEffector;
  private GroundIntake m_GroundIntake;
  private Shintake m_Shintake;
  public enum states{
    Idle,
    A1,
    A2,
    Lolipop,
    Score,
    Processor,
    GroundIntake,
    L1,
    L2,
    L3, 
    Rest 
  }
  private states wantedState = states.Idle;
  /** Creates a new Macros. */
  public Macros(Arm m_Arm, Climb m_Climb, EndEffector m_EndEffector,
  GroundIntake m_GroundIntake, Shintake m_Shintake) {
    this.m_Arm = m_Arm;
    this.m_Climb = m_Climb;
    this.m_EndEffector = m_EndEffector;
    this.m_GroundIntake = m_GroundIntake;
    this.m_Shintake = m_Shintake;
  }

  @Override
  public void periodic() {
    if(DriverStation.isEnabled()){
    switch (wantedState) {
      case GroundIntake:
        groundIntake();
        break;
      case Processor:
        processor();
        break;
      case A1:
        A1();
        break;
      case A2:
        A2();
        break;
      case Lolipop:
        lolipop();
        break;
      case L1:
        l1();
        break;
      case L2:
        l2();
        break;
      case L3:
        l3();
        break;
      case Score:
        score();
        break;
      case Rest:
        idle();
        if(!RobotContainer.operatorButtons.getRawButton(10)){
          wantedState = states.Idle;
        }
        break;
      default:
        break;
    }
  }
  else{
    wantedState = states.Idle;
  }
  Logger.recordOutput("Robot/Wanted State", wantedState);
  
    // This method will be called once per scheduler run
  }
  private void groundIntake(){
    m_GroundIntake.setWantedState(GroundIntakeStates.Intake);
    if(m_GroundIntake.getAlgaeDetected()){
      m_GroundIntake.setWantedState(GroundIntakeStates.Hold);
    }
  }
  private void processor(){
    m_GroundIntake.setWantedState(GroundIntakeStates.Outtake);
    m_Shintake.setWantedState(ShintakeStates.AlgaeOuttake);
    if(!m_GroundIntake.getAlgaeDetected()){
      m_GroundIntake.setWantedState(GroundIntakeStates.Rest);
      m_Shintake.setWantedState(ShintakeStates.Rest);
    }
  }
  private void A1(){
    if(m_Arm.getPosition()!=ArmPositions.Holding&&!m_EndEffector.getAlgaeInput()){
    m_Arm.setWantedPosition(ArmPositions.A1);
    m_EndEffector.setWantedState(EndEffectorStates.AlgaeIntake);
    }
    if(m_EndEffector.getAlgaeInput()){
      m_Arm.setWantedPosition(ArmPositions.Handoff);
      m_GroundIntake.setWantedState(GroundIntakeStates.Hold);
      m_Shintake.setWantedState(ShintakeStates.Transfer);
      m_EndEffector.setWantedState(EndEffectorStates.Rest);
    }
    if(m_Arm.getPosition()==ArmPositions.Handoff&&m_Arm.armNearPosition()){
      m_EndEffector.setWantedState(EndEffectorStates.AlgaeIntake);
      m_Shintake.setWantedState(ShintakeStates.AlgaeIntake);
      m_GroundIntake.setWantedState(GroundIntakeStates.Intake);
    }
    if(m_GroundIntake.getAlgaeDetected()){
      m_Arm.setWantedPosition(ArmPositions.Holding);
      m_EndEffector.setWantedState(EndEffectorStates.Rest);
      m_Shintake.setWantedState(ShintakeStates.AlgaeIntake);
    }
    if(!m_Shintake.getAlgaeDetected()&&m_Arm.getPosition()==ArmPositions.Holding){
      m_Shintake.setWantedState(ShintakeStates.Rest);
      m_GroundIntake.setWantedState(GroundIntakeStates.Hold);
      m_Arm.setWantedPosition(ArmPositions.Idle);
    }
  }
  private void A2(){
    if(m_Arm.getPosition()!=ArmPositions.Holding&&!m_EndEffector.getAlgaeInput()){
    m_Arm.setWantedPosition(ArmPositions.A2);
    m_EndEffector.setWantedState(EndEffectorStates.AlgaeIntake);
    }
    if(m_EndEffector.getAlgaeInput()){
      m_Arm.setWantedPosition(ArmPositions.Handoff);
      m_GroundIntake.setWantedState(GroundIntakeStates.Hold);
      m_Shintake.setWantedState(ShintakeStates.Transfer);
      m_EndEffector.setWantedState(EndEffectorStates.Rest);
    }
    if(m_Arm.getPosition()==ArmPositions.Handoff&&m_Arm.armNearPosition()){
      m_EndEffector.setWantedState(EndEffectorStates.AlgaeIntake);
      m_Shintake.setWantedState(ShintakeStates.AlgaeIntake);
      m_GroundIntake.setWantedState(GroundIntakeStates.Intake);
    }
    if(m_GroundIntake.getAlgaeDetected()){
      m_Arm.setWantedPosition(ArmPositions.Holding);
      m_EndEffector.setWantedState(EndEffectorStates.Rest);
      m_Shintake.setWantedState(ShintakeStates.AlgaeIntake);
    }
    if(!m_Shintake.getAlgaeDetected()&&m_Arm.getPosition()==ArmPositions.Holding){
      m_Shintake.setWantedState(ShintakeStates.Rest);
      m_GroundIntake.setWantedState(GroundIntakeStates.Hold);
      m_Arm.setWantedPosition(ArmPositions.Idle);
    }
  }
  private void lolipop(){
    if(m_Arm.getPosition()!=ArmPositions.Holding&&!m_EndEffector.getAlgaeInput()){
    m_Arm.setWantedPosition(ArmPositions.Lolipop);
    m_EndEffector.setWantedState(EndEffectorStates.AlgaeIntake);
    }
    if(m_EndEffector.getAlgaeInput()){
      m_Arm.setWantedPosition(ArmPositions.Handoff);
      m_GroundIntake.setWantedState(GroundIntakeStates.Hold);
      m_Shintake.setWantedState(ShintakeStates.Transfer);
      m_EndEffector.setWantedState(EndEffectorStates.Rest);
    }
    if(m_Arm.getPosition()==ArmPositions.Handoff&&m_Arm.armNearPosition()){
      m_EndEffector.setWantedState(EndEffectorStates.AlgaeIntake);
      m_Shintake.setWantedState(ShintakeStates.AlgaeIntake);
      m_GroundIntake.setWantedState(GroundIntakeStates.Intake);
    }
    if(m_GroundIntake.getAlgaeDetected()){
      m_Arm.setWantedPosition(ArmPositions.Holding);
      m_EndEffector.setWantedState(EndEffectorStates.Rest);
      m_Shintake.setWantedState(ShintakeStates.AlgaeIntake);
    }
    if(!m_Shintake.getAlgaeDetected()&&m_Arm.getPosition()==ArmPositions.Holding){
      m_Shintake.setWantedState(ShintakeStates.Rest);
      m_GroundIntake.setWantedState(GroundIntakeStates.Hold);
      m_Arm.setWantedPosition(ArmPositions.Idle);
    }
  }
  private void l1(){
    if(m_Arm.getPosition()!=ArmPositions.L1){
      m_Arm.setWantedPosition(ArmPositions.Station);
      m_EndEffector.setWantedState(EndEffectorStates.CoralIntake);
    }
    if(m_EndEffector.getCoralInput()){
      m_Arm.setWantedPosition(ArmPositions.L1);
      m_EndEffector.setWantedState(EndEffectorStates.CoralHold);
    }
  }

  private void l2(){
    if(m_Arm.getPosition()!=ArmPositions.L2){
      m_Arm.setWantedPosition(ArmPositions.Station);
      m_EndEffector.setWantedState(EndEffectorStates.CoralIntake);
    }
    if(m_EndEffector.getCoralInput()){
      m_Arm.setWantedPosition(ArmPositions.L2);
      m_EndEffector.setWantedState(EndEffectorStates.CoralHold);
    }
  }

  private void l3(){
    if(m_Arm.getPosition()!=ArmPositions.L3){
      m_Arm.setWantedPosition(ArmPositions.Station);
      m_EndEffector.setWantedState(EndEffectorStates.CoralIntake);
    }
    if(m_EndEffector.getCoralInput()){
      m_Arm.setWantedPosition(ArmPositions.L3);
      m_EndEffector.setWantedState(EndEffectorStates.CoralHold);
    }
  }

  private void score(){
    if(m_Arm.getPosition()==ArmPositions.L1||m_Arm.getPosition()==ArmPositions.L2){
      if(m_EndEffector.getCoralInput()){
        m_EndEffector.setWantedState(EndEffectorStates.CoralOut);
      }
      else{
        m_Arm.setWantedPosition(ArmPositions.Idle);
        m_EndEffector.setWantedState(EndEffectorStates.Rest);
      }
    }
    else if(m_Arm.getPosition()==ArmPositions.L3){
      if(m_EndEffector.getCoralInput()){
        m_EndEffector.setWantedState(EndEffectorStates.L3Out);
      }
      else{
        m_Arm.setWantedPosition(ArmPositions.Idle);
        m_EndEffector.setWantedState(EndEffectorStates.Rest);
      }
    }
    else{
      if(m_GroundIntake.getAlgaeDetected()){
        m_Shintake.setWantedState(ShintakeStates.Shoot);
        m_GroundIntake.setWantedState(GroundIntakeStates.Shoot);
      }
      else{
        m_Shintake.setWantedState(ShintakeStates.Rest);
      }
      if(!m_GroundIntake.getAlgaeDetected()&&m_Shintake.getShooterRMPs()[0]<1000){
        m_GroundIntake.setWantedState(GroundIntakeStates.Rest);
      }
    }
  }
  public void idle(){
    m_Arm.setWantedPosition(ArmPositions.Idle);
    m_Shintake.setWantedState(ShintakeStates.Rest);
    m_GroundIntake.setWantedState(GroundIntakeStates.Rest);
    m_EndEffector.setWantedState(EndEffectorStates.Rest);

  }

  public void setWantedState(states wantedState){
    this.wantedState = wantedState;
  }

}
