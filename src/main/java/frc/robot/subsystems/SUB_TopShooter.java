// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HardwareConstants;

public class SUB_TopShooter extends PIDSubsystem {
  private final CANSparkMax m_shooterTopMotor;
  private final RelativeEncoder m_shooterTopEncoder;
  private boolean firstPIDTesting = true;
  private SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kShooterTopSVolts, ShooterConstants.kTopVVoltSecondsPerRotation, ShooterConstants.kShooterBotA);

  /** The shooter subsystem for the robot. */
  public SUB_TopShooter() {
    super(new PIDController(ShooterConstants.kShooterTopP, ShooterConstants.kShooterTopI, ShooterConstants.kShooterTopD));
    m_shooterTopMotor  = new CANSparkMax(HardwareConstants.kShooterTopMotorCANID, MotorType.kBrushless);
    m_shooterTopEncoder = m_shooterTopMotor.getEncoder();
    
    m_shooterTopMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);

    m_shooterTopMotor.setInverted(true);
    m_shooterTopMotor.setIdleMode(IdleMode.kCoast);

    getController().setTolerance(ShooterConstants.kShooterTolerance);
    setSetpoint(ShooterConstants.kShooterTargetSpeed);



  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterTopMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    // m_shooterTopMotor.setVoltage(m_shooterFeedforward.calculate(setpoint));
  
  }

  @Override
  public double getMeasurement() {
    return m_shooterTopEncoder.getVelocity();
  }

  public void setShooterSetpoint(double p_speed){
    setSetpoint(p_speed);
  }

  public boolean atSetpoint() {
    return m_controller.atSetpoint();
  }

  // public void Interpolate(){
  //   MathUtil.interpolate();
  // }

  public double getVelocity(){
    return m_shooterTopEncoder.getVelocity();
  }

  public double getCurrent(){
    return m_shooterTopMotor.getOutputCurrent();
  }

  public void setPower(double p_power){
    m_shooterTopMotor.set(p_power);
  }

  private double m_ShooterP = ShooterConstants.kShooterTopP;
  private double m_ShooterI = ShooterConstants.kShooterTopI;
  private double m_ShooterD = ShooterConstants.kShooterTopD;
  private double m_ShooterFF = ShooterConstants.kShooterTopFF;
  private double m_ShooterS = ShooterConstants.kShooterTopSVolts;
  private double m_ShooterVV = ShooterConstants.kTopVVoltSecondsPerRotation;
  private double m_ShooterWantedVelocity = 0;

  public void ShooterTopPIDTuning(){
    if (firstPIDTesting){
      SmartDashboard.putNumber("ShooterP", m_ShooterP);
      SmartDashboard.putNumber("ShooterI", m_ShooterI);
      SmartDashboard.putNumber("ShooterD", m_ShooterD);
      SmartDashboard.putNumber("ShooterFF", m_ShooterFF);
      SmartDashboard.putNumber("ShooterS", m_ShooterS);
      SmartDashboard.putNumber("ShooterVV", m_ShooterVV);
      SmartDashboard.putNumber("ShooterWantedVelocity", 0);

      firstPIDTesting = false;
    }
  
    double m_ShooterP_ = SmartDashboard.getNumber("ShooterP", m_ShooterP);
    double m_ShooterI_ = SmartDashboard.getNumber("ShooterI", m_ShooterI);
    double m_ShooterD_ = SmartDashboard.getNumber("ShooterD", m_ShooterD);
    double m_ShooterFF_ = SmartDashboard.getNumber("ShooterFF", m_ShooterFF);
    double m_ShooterS_ = SmartDashboard.getNumber("ShooterS", m_ShooterS);
    double m_ShooterVV_ = SmartDashboard.getNumber("ShooterVV", m_ShooterVV);
    double m_ShooterWantedVelocity_ = SmartDashboard.getNumber("ShooterWantedVelocity", 0);
    if (m_ShooterP_ != m_ShooterP || m_ShooterI_ != m_ShooterI
        || m_ShooterD_ != m_ShooterD || m_ShooterFF_ != m_ShooterFF
        || m_ShooterWantedVelocity != m_ShooterWantedVelocity_
        || m_ShooterS != m_ShooterS_ || m_ShooterVV != m_ShooterVV_){
      
      m_ShooterP = m_ShooterP_;
      m_ShooterI = m_ShooterI_;
      m_ShooterD = m_ShooterD_;
      m_ShooterFF = m_ShooterFF_;
      m_ShooterS = m_ShooterS_;
      m_ShooterVV = m_ShooterVV_; 
      m_ShooterWantedVelocity = m_ShooterWantedVelocity_;
      m_shooterFeedforward = new SimpleMotorFeedforward(m_ShooterS, m_ShooterVV);  
      m_controller.setP(m_ShooterP);
      m_controller.setI(m_ShooterI);
      m_controller.setD(m_ShooterD);
      setSetpoint(m_ShooterWantedVelocity);
      SmartDashboard.putNumber("ShooterP", m_ShooterP);
      SmartDashboard.putNumber("ShooterI", m_ShooterI);
      SmartDashboard.putNumber("ShooterD", m_ShooterD);
      SmartDashboard.putNumber("ShooterFF", m_ShooterFF);
      SmartDashboard.putNumber("ShooterS", m_ShooterS);
      SmartDashboard.putNumber("ShooterVV", m_ShooterVV);
      SmartDashboard.putNumber("ShooterWantedVelocity", m_ShooterWantedVelocity);
    }
  }

  // @Override
  // public void periodic() {
  //   ShooterPIDTuning();
  //   // This method will be called once per scheduler run
  //   if (m_enabled) {
  //   useOutput(m_controller.calculate(getMeasurement()), getSetpoint());
  //   }
    // SmartDashboard.putNumber("Shooter Top Setpoint", getSetpoint());
  //   SmartDashboard.putNumber("Shooter Top Velocity", m_shooterTopEncoder.getVelocity());
  // }
}