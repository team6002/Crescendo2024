// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.HardwareConstants;


public class SUB_Intake extends SubsystemBase {
  /** Creates a new SUB_Intake. */
  private final CANSparkMax m_groundIntakeMotor;
  private final SparkPIDController m_groundIntakePIDController;
  private final RelativeEncoder m_groundIntakeEncoder;
  private final SimpleMotorFeedforward m_intakeFeedFoward;

  // private final CANSparkMax m_shelfIntakeMotor;
  // private final SparkPIDController m_shelfIntakePIDController;

  private final CANSparkMax m_indexerMotor;
  private final RelativeEncoder m_indexerEncoder;
  private final SparkPIDController m_indexerPIDController;
  private final SimpleMotorFeedforward m_indexerFeedFoward;
  // private final SparkLimitSwitch m_indexerSensor;
  private final DigitalInput m_indexerSensor;

  public SUB_Intake() {
    m_groundIntakeMotor = new CANSparkMax(HardwareConstants.kGroundIntakeMotorCANID, MotorType.kBrushless);
    m_groundIntakeMotor.restoreFactoryDefaults();
    m_groundIntakePIDController = m_groundIntakeMotor.getPIDController();
    m_groundIntakeEncoder = m_groundIntakeMotor.getEncoder();
  
    m_groundIntakePIDController.setP(IntakeConstants.kIntakeP);
    m_groundIntakePIDController.setI(IntakeConstants.kIntakeI);
    m_groundIntakePIDController.setD(IntakeConstants.kIntakeD);
    m_groundIntakePIDController.setFF(IntakeConstants.kIntakeFF);

    m_intakeFeedFoward = new SimpleMotorFeedforward(IntakeConstants.kIntakeS, IntakeConstants.kIntakeV);
    
    m_groundIntakeMotor.setSmartCurrentLimit(IntakeConstants.kIntakeCurrentLimit);
    m_groundIntakeMotor.setIdleMode(IdleMode.kCoast);
    m_groundIntakeMotor.setInverted(true);
  
    // m_shelfIntakeMotor = new CANSparkMax(IntakeConstants.kShelfIntakeMotorCANID, MotorType.kBrushless);
    // m_shelfIntakePIDController = m_shelfIntakeMotor.getPIDController();
    // m_shelfIntakeMotor.setIdleMode(IdleMode.kBrake);

    m_indexerMotor = new CANSparkMax(HardwareConstants.kIndexerMotorCANID, MotorType.kBrushless);
    m_indexerPIDController = m_indexerMotor.getPIDController();
    m_indexerMotor.restoreFactoryDefaults();
    
    m_indexerMotor.setSmartCurrentLimit(IndexerConstants.kIndexerCurrentLimit);
  
    m_indexerPIDController.setP(IndexerConstants.kIndexerP);
    m_indexerPIDController.setI(IndexerConstants.kIndexerI);
    m_indexerPIDController.setD(IndexerConstants.kIndexerD);
    m_indexerPIDController.setFF(IndexerConstants.kIndexerFF);

    m_indexerFeedFoward = new SimpleMotorFeedforward(IndexerConstants.kIndexerS, IndexerConstants.kIndexerV);

    m_indexerEncoder = m_indexerMotor.getEncoder();
    // m_indexerMotor.
    // m_indexerSensor = m_indexerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen);
    m_indexerSensor = new DigitalInput(9);
    m_indexerMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    m_indexerMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyOpen).enableLimitSwitch(false);
    // m_indexerSensor.enableLimitSwitch(false);
    // m_indexerPIDController = m_indexerMotor.getPIDController();
    m_indexerMotor.setIdleMode(IdleMode.kBrake);
    m_indexerMotor.setInverted(true);
    m_indexerMotor.burnFlash();
  }

  public void stopGroundIntake(){
    m_groundIntakeMotor.set(0);
  }

  public void setGroundIntakePower(double p_power){
    m_groundIntakeMotor.set(p_power);
  }

  public double getGroundIntakeCurrent(){
    return m_groundIntakeMotor.getOutputCurrent();
  }

  public double getGroundIntakeVelocity(){
    return m_groundIntakeEncoder.getVelocity();
  }
  
  public void setGroundIntakeVelocity(double p_velocity){
    m_groundIntakePIDController.setReference(p_velocity, ControlType.kVelocity, 0, m_intakeFeedFoward.calculate(p_velocity));
  }
  // public Command setGroundIntakePower(double power) {
  //   return Commands.runOnce(()->setIndexerPower(power),this);
  // }

  public Command setIntakeVelocity(double velocity) {
    return Commands.runOnce(()->setIntakeVelocity(velocity),this);
  }
  // public void stopShelfIntake(){
  //   m_shelfIntakeMotor.set(0);
  // }

  // public void setShelfIntakePower(double p_power){
  //   m_shelfIntakeMotor.set(p_power);
  // }

  // public double getShelfIntakeCurrent(){
  //   return m_shelfIntakeMotor.getOutputCurrent();
  // }

  public void stopIndexer(){
    m_indexerMotor.set(0);
  }

  public void setIndexerPower(double p_power){
    m_indexerMotor.set(p_power);
  }


  public void setIndexerVelocity(double p_velocity){
    m_indexerPIDController.setReference(p_velocity, ControlType.kVelocity, 0, m_indexerFeedFoward.calculate(p_velocity));
  }

  public double getIndexerVelocity(){
    return m_indexerEncoder.getVelocity();
  }

  public double getIndexerCurrent(){
    return m_indexerMotor.getOutputCurrent();
  }

  public double getIndexerPosition(){
    return m_indexerEncoder.getPosition();
  }

  public boolean getIndexerSensor(){
    // return m_indexerSensor.isPressed();
    return !m_indexerSensor.get();
  }

  // public void enableIndexerLimit(boolean booolean){
  //   m_indexerSensor.enableLimitSwitch(booolean);
  // }
  public Command CMDsetIndexPower(double power) {
    return Commands.runOnce(()->setIndexerPower(power),this);
  }

  public Command CMDsetIndexVelocity(double velocity) {
    return Commands.runOnce(()->setIndexerVelocity(velocity),this);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("IntakeOutputCurrent", getGroundIntakeCurrent());
    // SmartDashboard.putNumber("IndexerOutputCurrent", getIndexerCurrent());
    // SmartDashboard.putNumber("Indexer Position", getIndexerPosition());
    // SmartDashboard.putNumber("Indexer Velocity", getIndexerVelocity());
    // SmartDashboard.putNumber("Intake Velocity", getGroundIntakeVelocity());
    // SmartDashboard.putBoolean("Indexer Sensor", getIndexerSensor());
    // This method will be called once per scheduler run
    
  }
}
