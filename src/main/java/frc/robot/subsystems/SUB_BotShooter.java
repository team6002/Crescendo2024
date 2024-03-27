// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.HardwareConstants;

public class SUB_BotShooter extends PIDSubsystem {
  private final CANSparkMax m_shooterBotMotor;
  private final RelativeEncoder m_shooterBotEncoder;
  private final CANSparkMax m_shooterBotFollowerMotor;
  private final RelativeEncoder m_shooterBotFollowerEncoder;
  private boolean firstPIDTesting = true;
  private SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(
          ShooterConstants.kShooterBotSVolts, ShooterConstants.kBotVVoltSecondsPerRotation, ShooterConstants.kShooterBotA);

  /** The shooter subsystem for the robot. */
  public SUB_BotShooter() {
    super(new PIDController(ShooterConstants.kShooterBotP, ShooterConstants.kShooterBotI, ShooterConstants.kShooterBotD));
    // super(new PIDController(ShooterConstants.kShooterBotP, ShooterConstants.kShooterBotI, ShooterConstants.kShooterBotD));

    m_shooterBotMotor = new CANSparkMax(HardwareConstants.kShooterBotMotorCANID, MotorType.kBrushless);
    m_shooterBotEncoder = m_shooterBotMotor.getEncoder();

    m_shooterBotMotor.restoreFactoryDefaults(); 

    m_shooterBotMotor.setInverted(true);
    m_shooterBotMotor.setIdleMode(IdleMode.kCoast);

    m_shooterBotFollowerMotor = new CANSparkMax(HardwareConstants.kShooterBotFollowerCANID, MotorType.kBrushless);
    m_shooterBotFollowerEncoder = m_shooterBotFollowerMotor.getEncoder();
    
    m_shooterBotFollowerMotor.restoreFactoryDefaults(); 
    
    m_shooterBotMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);
    m_shooterBotFollowerMotor.setSmartCurrentLimit(ShooterConstants.kShooterCurrentLimit);
    

    m_shooterBotFollowerMotor.follow(m_shooterBotMotor, false);
    m_shooterBotFollowerMotor.setIdleMode(IdleMode.kCoast);

    m_shooterBotFollowerMotor.burnFlash();
    m_shooterBotMotor.burnFlash();

    getController().setTolerance(ShooterConstants.kShooterTolerance);
    setSetpoint(ShooterConstants.kShooterTargetSpeed);



  }

  @Override
  public void useOutput(double output, double setpoint) {
    m_shooterBotMotor.setVoltage(output + m_shooterFeedforward.calculate(setpoint));
    // m_shooterBotMotor.setVoltage(m_shooterFeedforward.calculate(setpoint));
  
  }

  @Override
  public double getMeasurement() {
    return m_shooterBotEncoder.getVelocity();
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
    return m_shooterBotEncoder.getVelocity();
  }

  public double getCurrent(){
    return m_shooterBotMotor.getOutputCurrent();
  }

  public double getFollowerCurrent(){
    return m_shooterBotFollowerMotor.getOutputCurrent();
  }

  public void setPower(double p_power){
    m_shooterBotMotor.set(p_power);
  }
  
  private double m_ShooterP = ShooterConstants.kShooterBotP;
  private double m_ShooterI = ShooterConstants.kShooterBotI;
  private double m_ShooterD = ShooterConstants.kShooterBotD;
  private double m_ShooterFF = ShooterConstants.kShooterBotFF;
  private double m_ShooterS = ShooterConstants.kShooterBotSVolts;
  private double m_ShooterVV = ShooterConstants.kBotVVoltSecondsPerRotation;
  private double m_ShooterWantedVelocity = 0;

  public void ShooterBotPIDTuning(){
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
    double m_ShooterWantedVelocity_ = SmartDashboard.getNumber("ShooterWantedVelocity", m_ShooterWantedVelocity);
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
  //   // ShooterPIDTuning();
  //   // This method will be called once per scheduler run
  //   if (m_enabled) {
  //   useOutput(m_controller.calculate(getMeasurement()), getSetpoint());
  //   }
  //   SmartDashboard.putNumber("Shooter Bot Setpoint", getSetpoint());
  //   SmartDashboard.putNumber("Shooter Bot Current", m_shooterBotMotor.getOutputCurrent());
  //   SmartDashboard.putNumber("Shooter Bot Velocity", m_shooterBotEncoder.getVelocity());
  // }
}