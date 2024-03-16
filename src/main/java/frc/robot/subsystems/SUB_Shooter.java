// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.utils.LinearInterpolater;

public class SUB_Shooter extends SubsystemBase {
  /** Creates a new CMD_Shooter. */
  SUB_BotShooter m_botShooter;
  SUB_TopShooter m_topShooter;
  LinearInterpolater m_interpolater;
  double m_interpolatedValue;
  LinearInterpolater m_stockInterpolater;
  double m_stockInterpolatedValue;

  public SUB_Shooter(SUB_BotShooter p_botShooter, SUB_TopShooter p_topShooter) {
    m_botShooter = p_botShooter;
    m_topShooter = p_topShooter;
    m_interpolater = new LinearInterpolater(ShooterConstants.kShooterArray);
  }

  public double getBotShooterVelocity(){
    return m_botShooter.getVelocity();
  }

  public double getBotShooterCurrent(){
    return m_botShooter.getCurrent();
  }

  public boolean atBotSetpoint(){
    return m_botShooter.atSetpoint();
  }

  public double getBotSetpoint(){
    return m_botShooter.getSetpoint();
  }

  public void setBotPower(double p_power){
    m_botShooter.setPower(p_power);
  }
  
  public void enableBotShooter(){
    m_botShooter.enable();
  }

  public void disableBotShooter(){
    m_botShooter.disable();
  }

  public void setBotSetpoint(double RPMsetpoint){
    m_botShooter.setSetpoint(RPMsetpoint);
  }

  public double getTopShooterVelocity(){
    return m_topShooter.getVelocity();
  }

  public double getTopShooterCurrent(){
    return m_topShooter.getCurrent();
  }

  public void setTopPower(double p_power){
    m_topShooter.setPower(p_power);
  }

  public boolean atTopSetpoint(){
    return m_topShooter.atSetpoint();
  }

  public double getTopSetpoint(){
    return m_topShooter.getSetpoint();
  }

  public void enableTopShooter(){
    m_topShooter.enable();
  }

  public void disableTopShooter(){
    m_topShooter.disable();
  }

  public void setTopSetpoint(double RPMsetpoint){
    m_topShooter.setSetpoint(RPMsetpoint);
  }
  /**sets both of the shooters setpoints */
  public void setShooterSetpoint(double RPMsetpoint){
    setBotSetpoint(RPMsetpoint);
    setTopSetpoint(RPMsetpoint);
  }
  /**returns if the shooters are at the correct setpoints */
  public boolean getAtShooterSetpoint(){
    // return (atBotSetpoint() && atTopSetpoint());
    return (Math.abs(getBotSetpoint() - getBotShooterVelocity()) < ShooterConstants.kShooterTolerance && Math.abs(getTopSetpoint() - getTopShooterVelocity()) < ShooterConstants.kShooterTolerance );
    
  }
  /**turns on both shooters */
  public void enableShooter(){
    enableBotShooter();
    enableTopShooter();
  }

  public void disableShooter(){
    disableBotShooter();
    disableTopShooter();
  }

  public double interpolateSetpoint(double p_distance){
    m_interpolatedValue = m_interpolater.getInterpolatedValue(p_distance);
    return m_interpolatedValue;
  }
  
  public double getInterpolatedValue(){
    return m_interpolatedValue;
  }

  public double stockInterpolateSetpoint(double p_distance){
    m_stockInterpolatedValue = m_stockInterpolater.getInterpolatedValue(p_distance);
    return m_stockInterpolatedValue;
  }
  
  public double getStockInterpolatedValue(){
    return m_stockInterpolatedValue;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // m_botShooter.ShooterBotPIDTuning();
    // m_topShooter.ShooterTopPIDTuning();
    
    SmartDashboard.putBoolean("ShootFin", getAtShooterSetpoint());
    SmartDashboard.putNumber("ShooterGoal", getBotSetpoint());
    SmartDashboard.putNumber("Shooter Top Velocity", getTopShooterVelocity());
    // SmartDashboard.putNumber("Shooter Top Current", getTopShooterCurrent());

    SmartDashboard.putNumber("Shooter Bot Velocity", getBotShooterVelocity());
    // SmartDashboard.putNumber("Shooter Bot Current", getBotShooterCurrent());
    // SmartDashboard.putNumber("Shooter Follower Current", m_botShooter.getFollowerCurrent());

  }
}
