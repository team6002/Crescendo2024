// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.HardwareConstants;
import frc.utils.TrapezoidProfileSubsystem;

public class SUB_Elbow extends TrapezoidProfileSubsystem  {

  private final CANSparkMax m_elbowMotor;
  private SparkPIDController m_elbowPIDController;
  private AbsoluteEncoder m_elbowAbsoluteEncoder;
    
  private ArmFeedforward m_elbowFeedForward;

  private double m_currentGoal;
    private double m_positionTolerance = Math.toRadians(2);;
    
  //makes the PID tuning display all the Smartdashboard stuff it needs
  private boolean firstPIDTesting = true;

  //returns the angle the elbow should be to the ground
  // removed by hdn
  // private double m_elbowRelativeGround; 
  private static double deltaTime = 0.02;
  
  /** Creates a new SUB_Shooter. */
  public SUB_Elbow() {
    super(
        new TrapezoidProfile.Constraints(ElbowConstants.kMaxVelocityRadPerSecond
        , ElbowConstants.kMaxAccelerationRadPerSecSquared)
        ,Math.toDegrees(10),deltaTime);

    m_elbowMotor = new CANSparkMax(HardwareConstants.kElbowMotorCANID, MotorType.kBrushless);
    m_elbowMotor.restoreFactoryDefaults();
    m_elbowPIDController = m_elbowMotor.getPIDController();
    m_elbowAbsoluteEncoder = m_elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_elbowMotor.setIdleMode(IdleMode.kBrake);
    m_elbowMotor.setInverted(true);
    m_elbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowCurrentLimit);

    m_elbowAbsoluteEncoder.setInverted(true);
    m_elbowAbsoluteEncoder.setPositionConversionFactor(ElbowConstants.kPositionConversionFactor);
    m_elbowAbsoluteEncoder.setVelocityConversionFactor(ElbowConstants.kVelocityConversionFactor);

    m_elbowPIDController.setFeedbackDevice(m_elbowAbsoluteEncoder);

    m_elbowPIDController.setP(ElbowConstants.kElbowP,1);
    m_elbowPIDController.setI(ElbowConstants.kElbowI,1);
    m_elbowPIDController.setD(ElbowConstants.kElbowD,1);
    m_elbowPIDController.setFF(ElbowConstants.kElbowFF,1);
    
    m_elbowPIDController.setPositionPIDWrappingEnabled(false);
    // m_elbowPIDController.setPositionPIDWrappingMinInput(-180);
    // m_elbowPIDController.setPositionPIDWrappingMaxInput(180);
    m_elbowMotor.burnFlash();
    
    m_elbowFeedForward = new ArmFeedforward(ElbowConstants.kSVolts,ElbowConstants.kGVolts
        , ElbowConstants.kVVoltSecondPerRad,ElbowConstants.kAVoltSecondSquaredPerRad);  

    setGoal(getPositionRad()); // set goal to current position to prevent it from moving
    setSetpoint(getPositionRad());

  }

    public double getPositionRad(){
      return MathUtil.angleModulus(m_elbowAbsoluteEncoder.getPosition());
    }

    public double getVelocity(){
      return m_elbowAbsoluteEncoder.getVelocity();
    }
  
    public double getCurrent(){
      return m_elbowMotor.getOutputCurrent();
    }
    
    // public void elbowManualMove(double p_power){
    //   double newReference = (ElbowConstants.kMaxVelocityRadPerSecond  * deltaTime * p_power) 
    //     + m_elbowGoal.position;
    //   setElbowReference(newReference);
    // }
    
    public double getGoal(){
        return m_currentGoal;
    }

    public boolean atGoal() {    
        return atGoal(m_positionTolerance);
    }

    public boolean atGoal(double tolerateRad) {
        return (Math.abs(m_currentGoal - getPositionRad()) <= tolerateRad);
    }

    public void setGoalRad(double goalRad) {
        m_currentGoal = goalRad;
        setGoal(goalRad);
    }

    private double m_ElbowP = ElbowConstants.kElbowP;
    private double m_ElbowI = ElbowConstants.kElbowI;
    private double m_ElbowD = ElbowConstants.kElbowD;
    private double m_ElbowFF = ElbowConstants.kElbowFF;
    private double m_ElbowS = ElbowConstants.kSVolts;
    private double m_ElbowG = ElbowConstants.kGVolts;
    private double m_ElbowVV = ElbowConstants.kVVoltSecondPerRad;
    private double m_ElbowAV = ElbowConstants.kAVoltSecondSquaredPerRad;
    private double m_ElbowWantedPosition = 0;

    // public void ElbowPIDTuning(){
    //     if (firstPIDTesting){
    //       SmartDashboard.putNumber("ElbowP", m_ElbowP);
    //       SmartDashboard.putNumber("ElbowI", m_ElbowI);
    //       SmartDashboard.putNumber("ElbowD", m_ElbowD);
    //       SmartDashboard.putNumber("ElbowFF", m_ElbowFF);
    //       SmartDashboard.putNumber("ElbowS", m_ElbowS);
    //       SmartDashboard.putNumber("ElbowG", m_ElbowG);
    //       SmartDashboard.putNumber("ElbowVV", m_ElbowVV);
    //       SmartDashboard.putNumber("ElbowAV", m_ElbowAV);
          
    //       // SmartDashboard.putNumber("ElbowWantedPosition", 15);

    //       firstPIDTesting = false;
    //     }
      
    //     double m_ElbowP_ = SmartDashboard.getNumber("ElbowP", m_ElbowP);
    //     double m_ElbowI_ = SmartDashboard.getNumber("ElbowI", m_ElbowI);
    //     double m_ElbowD_ = SmartDashboard.getNumber("ElbowD", m_ElbowD);
    //     double m_ElbowFF_ = SmartDashboard.getNumber("ElbowFF", m_ElbowFF);
    //     double m_ElbowS_ = SmartDashboard.getNumber("ElbowS", m_ElbowS);
    //     double m_ElbowG_ = SmartDashboard.getNumber("ElbowG", m_ElbowG);
    //     double m_ElbowVV_ = SmartDashboard.getNumber("ElbowVV", m_ElbowVV);
    //     double m_ElbowAV_ = SmartDashboard.getNumber("ElbowAV", m_ElbowAV);
    //     double m_ElbowWantedPosition_ = SmartDashboard.getNumber("ElbowWantedPosition", m_ElbowWantedPosition);
    //     if (m_ElbowP_ != m_ElbowP || m_ElbowI_ != m_ElbowI
    //         || m_ElbowD_ != m_ElbowD || m_ElbowFF_ != m_ElbowFF
    //         || m_ElbowWantedPosition != m_ElbowWantedPosition_
    //         || m_ElbowS != m_ElbowS_ || m_ElbowG != m_ElbowG_
    //         || m_ElbowVV != m_ElbowVV_ || m_ElbowAV != m_ElbowAV_){
          
    //       m_ElbowP = m_ElbowP_;
    //       m_ElbowI = m_ElbowI_;
    //       m_ElbowD = m_ElbowD_;
    //       m_ElbowFF = m_ElbowFF_;
    //       m_ElbowS = m_ElbowS_;
    //       m_ElbowG = m_ElbowG_;
    //       m_ElbowVV = m_ElbowVV_;
    //       m_ElbowAV = m_ElbowAV_;    
    //       m_ElbowWantedPosition = m_ElbowWantedPosition_;
    //       m_elbowFeedForward = new ArmFeedforward(m_ElbowS, m_ElbowG, m_ElbowVV, m_ElbowAV);  
    //       m_elbowPIDController.setP(m_ElbowP,1);
    //       m_elbowPIDController.setI(m_ElbowI,1);
    //       m_elbowPIDController.setD(m_ElbowD,1);
    //       m_elbowPIDController.setFF(m_ElbowFF,1);
    //       setGoalRad(MathUtil.clamp(Math.toRadians(m_ElbowWantedPosition),Math.toRadians(15),Math.toRadians(115)));
    //       SmartDashboard.putNumber("ElbowP", m_ElbowP);
    //       SmartDashboard.putNumber("ElbowI", m_ElbowI);
    //       SmartDashboard.putNumber("ElbowD", m_ElbowD);
    //       SmartDashboard.putNumber("ElbowFF", m_ElbowFF);
    //       SmartDashboard.putNumber("ElbowS", m_ElbowS);
    //       SmartDashboard.putNumber("ElbowG", m_ElbowG);
    //       SmartDashboard.putNumber("ElbowVV", m_ElbowVV);
    //       SmartDashboard.putNumber("ElbowAV", m_ElbowAV);
    //       SmartDashboard.putNumber("ElbowWantedPosition", m_ElbowWantedPosition);
          
    //     }
  
    //   }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    m_elbowPIDController.setReference(
      setpoint.position, 
      CANSparkMax.ControlType.kPosition,(1),
      m_elbowFeedForward.calculate(setpoint.position,setpoint.velocity)
    );  
    SmartDashboard.putNumber("Elbow Setpoint", Math.toDegrees(setpoint.position));
    // SmartDashboard.putNumber("Elbow Setpoint Velocity", setpoint.velocity);
    // SmartDashboard.putNumber("Elbow feedforward", m_elbowFeedForward.calculate(setpoint.position, setpoint.velocity));
  }  
}
