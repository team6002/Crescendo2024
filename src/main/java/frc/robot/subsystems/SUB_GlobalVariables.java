// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LocationConstants;

public class SUB_GlobalVariables extends SubsystemBase {
  /** place for subsystems to talk to each other */
  private boolean m_autofire;
  private boolean m_autoaim;
  private boolean m_hasItem;
  private boolean m_readyDrop;
  private boolean m_contShooting; // continous shooting mode
  private int m_intakeType = 0;
  private int m_outputType = 0;
  private int m_robotStage = 0;
  private int m_syncNumber = 0;
  private Pose2d m_syncLocation = LocationConstants.SubwooferBlue;
  public SUB_GlobalVariables() {}
  //*returns if the robot is automatically shooting */
  public boolean getAutofire(){
    return m_autofire;
  }

  public void setAutofire(boolean p_autofire){
    m_autofire = p_autofire;
  }

  public Command CMDsetAutofire(boolean p_autofire) {
    return Commands.runOnce(()->setAutofire(p_autofire),this);
  }

  public boolean getAutoaim(){
    return m_autoaim;
  }

  public void setAutoaim(boolean p_autoaim){
    m_autoaim = p_autoaim;
  }

  public Command CMDsetAutoaim(boolean p_autoaim) {
    return Commands.runOnce(()->setAutoaim(p_autoaim),this);
  }

  public boolean getHasItem(){
    return m_hasItem;
  }

  public void setHasItem(boolean p_hasitem){
    m_hasItem = p_hasitem;
  }

  public Command CMDsetHasItem(boolean p_hasitem) {
    return Commands.runOnce(()->setHasItem(p_hasitem),this);
  }

  public boolean getReadyDrop(){
    return m_readyDrop;
  }

  public void setReadyDrop(boolean p_readydrop){
    m_readyDrop = p_readydrop;
  }

  public Command CMDsetReadyDrop(boolean p_readydrop) {
    return Commands.runOnce(()->setReadyDrop(p_readydrop),this);
  }

  public boolean getContShooting(){
    return m_contShooting;
  }

  public void setContShooting(boolean p_contshooting){
    m_contShooting = p_contshooting;
  }

  public Command CMDsetContShooting(boolean p_contshooting) {
    return Commands.runOnce(()->setContShooting(p_contshooting),this);
  }

  public int getIntakeType(){
    return m_intakeType;
  }

  public void setIntakeType(int p_intaketype){
    m_intakeType = p_intaketype;
  }

  public Command CMDsetIntakeType(int p_intakeType) {
    return Commands.runOnce(()->setIntakeType(p_intakeType),this);
  }

  public int getOutputType(){
    return m_outputType;
  }

  public void setOutputType(int p_outputtype){
    m_outputType = p_outputtype;
  }

  public Command CMDsetOutputType(int p_outputType) {
    return Commands.runOnce(()->setOutputType(p_outputType),this);
  }

  public int getRobotStage(){
    return m_robotStage;
  }

  public void setRobotStage(int p_robotstage){
    m_robotStage = p_robotstage;
  }

  public Command CMDsetRobotStage(int p_robotStage) {
    return Commands.runOnce(()->setRobotStage(p_robotStage),this);
  }
  public int getSyncNumber(){
    return m_syncNumber;
  }

  public void setSyncNumber(int p_syncNumber){
    m_syncNumber = p_syncNumber;
  }

  public Command CMDsetSyncNumber(int p_syncNumber) {
    return Commands.runOnce(()->setSyncNumber(p_syncNumber),this);
  }
  public Pose2d getSyncLocation(){
    return m_syncLocation;
  }

  public void setSyncLocation(Pose2d p_pose2d){
    m_syncLocation = p_pose2d;
  }

  public Command CMDsetSyncLocation(Pose2d p_pose2d) {
    return Commands.runOnce(()->setSyncLocation(p_pose2d),this);
  }
  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("HasItem", m_hasItem);
    // SmartDashboard.putBoolean("ReadyDrop", m_readyDrop);
    SmartDashboard.putNumber("IntakeType", m_intakeType);
    SmartDashboard.putNumber("OutputType", m_outputType);
    // SmartDashboard.putNumber("RobotStage", m_robotStage);
    SmartDashboard.putNumber("SyncNumber", m_syncNumber);
    SmartDashboard.putBoolean("ContShoot", m_contShooting);
    // This method will be called once per scheduler run
  }
}
