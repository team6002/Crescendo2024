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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.HardwareConstants;
import frc.utils.LinearInterpolater;
import frc.utils.TrapezoidProfileSubsystem;


public class SUB_Shoulder extends TrapezoidProfileSubsystem {

  private LinearInterpolater m_interpolater;

  private final CANSparkMax m_shoulderMotor;
  private final CANSparkMax m_shoulderFollowerMotor;
  private SparkPIDController m_shoulderPIDController;
  private AbsoluteEncoder m_shoulderAbsoluteEncoder;

  private Servo m_hookLeft;
  private Servo m_hookRight;
  // private final RelativeEncoder m_shoulderEncoder;

  private double m_currentGoal;
  private double m_positionTolerance = Math.toRadians(2);

  private double m_shoulderPIDset;
  
  
  private ArmFeedforward m_shoulderFeedForward;

  //makes the PID tuning display all the Smartdashboard stuff it needs
  private boolean firstPIDTesting = true;

  private double m_interpolatedValue;

  //returns the angle the shoulder should be to the ground
  // removed by hdn
  // private double m_shoulderRelativeGround; 
  private static double deltaTime = 0.02;
  /** Creates a new SUB_Shooter. */
  public SUB_Shoulder() {
    super(
        new TrapezoidProfile.Constraints(ShoulderConstants.kMaxVelocityRadPerSecond
        , ShoulderConstants.kMaxAccelerationRadPerSecSquared)
        ,Math.toRadians(-47), deltaTime);
    
    m_shoulderMotor = new CANSparkMax(HardwareConstants.kShoulderMotorCANID, MotorType.kBrushless);
    m_shoulderFollowerMotor = new CANSparkMax(HardwareConstants.kShoulderFollowerMotorCANID, MotorType.kBrushless);
    m_shoulderPIDController = m_shoulderMotor.getPIDController();
    m_shoulderAbsoluteEncoder = m_shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
    m_shoulderMotor.setInverted(true);
    m_shoulderFollowerMotor.follow(m_shoulderMotor, true);

    m_shoulderMotor.setIdleMode(IdleMode.kCoast);
    m_shoulderFollowerMotor.setIdleMode(IdleMode.kBrake);
    m_shoulderMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderCurrentLimit);
    m_shoulderFollowerMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderCurrentLimit);

    m_shoulderAbsoluteEncoder.setPositionConversionFactor(ShoulderConstants.kPositionConversionFactor);
    m_shoulderAbsoluteEncoder.setVelocityConversionFactor(ShoulderConstants.kVelocityConversionFactor);

    m_shoulderPIDController.setFeedbackDevice(m_shoulderAbsoluteEncoder);

    m_shoulderPIDController.setP(ShoulderConstants.kShoulderP,1);
    m_shoulderPIDController.setI(ShoulderConstants.kShoulderI,1);
    m_shoulderPIDController.setD(ShoulderConstants.kShoulderD,1);
    m_shoulderPIDController.setFF(ShoulderConstants.kShoulderFF,1);

    m_shoulderPIDController.setPositionPIDWrappingEnabled(true); 
    m_shoulderPIDController.setPositionPIDWrappingMinInput(-Math.PI);
    m_shoulderPIDController.setPositionPIDWrappingMaxInput(Math.PI);
    m_shoulderMotor.burnFlash();      
    
    m_hookLeft = new Servo(HardwareConstants.kHookLeft);
    m_hookRight = new Servo(HardwareConstants.kHookRight);

    m_shoulderFeedForward = new ArmFeedforward(ShoulderConstants.kSVolts, ShoulderConstants.kGVolts
                ,ShoulderConstants.kVVoltSecondPerRad,ShoulderConstants.kAVoltSecondSquaredPerRad);   
    
    m_interpolater = new LinearInterpolater(ShoulderConstants.kShoulderArray);
   
    setGoal(getPositionRad()); // set goal to current position to prevent it from moving 
    setSetpoint(getPositionRad());
  }

    
    // public void shoulderInit(){
    //   // m_shoulderSetpoint = new TrapezoidProfile.State(getShoulderPosition(), 0); 
    //   // m_shoulderGoal = m_shoulderSetpoint;
    //    m_shoulderGoal = new TrapezoidProfile.State(getShoulderPositionRad(), 0); 
    // }

    /**Only moves the shoulder*/
    // public void setShoulderReferenceWithoutShoulder(double rad){
    //   // m_shoulderSetpoint = new TrapezoidProfile.State(getShoulderPosition(),0);
    //   m_shoulderGoal = new TrapezoidProfile.State(rad, 0);
    // }

    // /**sets a reference to both the shoulder and the shoulder to keep the shoulder position the same realtive to the ground*/
    // public void setShoulderReference(double rad){
    //   double shoulderRelativeGoal = (m_shoulderGoal.position - m_shoulderGoal.position);
    //   m_shoulderGoal = new TrapezoidProfile.State(rad, 0);
    //   setShoulderReference(shoulderRelativeGoal);

    //   // m_shoulderGoal = new TrapezoidProfile.State(
    //   //   MathUtil.clamp(getShoulderRelativePosition() + MathUtil.inputModulus(p_reference, -180, 180), 17, 115), 0); 
    // }

    // public double getShoulderPosition(){
    //   return m_shoulderEncoder.getPosition();
    // }


    public double getPositionRad(){
      return MathUtil.angleModulus(m_shoulderAbsoluteEncoder.getPosition());
    }

    public void setLHookPWM(double p_PWM){
      m_hookLeft.set(p_PWM);
    }
    
    public void setRHookPWM(double p_PWM){
      m_hookRight.set(p_PWM);
    }
    
    // public void setHooksPWM(double p_PWM){
    //   setRHookPWM(p_PWM);
    //   setLHookPWM(p_PWM);
    // }

    public Command CMDsetLHookPWM(double p_PWM) {
      return Commands.runOnce(()->setLHookPWM(p_PWM),this);
    }

    public Command CMDsetRHookPWM(double p_PWM) {
      return Commands.runOnce(()->setRHookPWM(p_PWM),this);
    }

    // public Command CMDsetHooksPWM(double p_PWM) {
    //   return Commands.runOnce(()->setHooksPWM(p_PWM),this);
    // }
    

    public double getVelocity(){
      return m_shoulderAbsoluteEncoder.getVelocity();
    }
     
    public double getCurrent(){
      return m_shoulderMotor.getOutputCurrent();
    }
    
    // public void shoulderManualMove(double p_power){
    //   // double joystickIntepertation = MathUtil.clamp(p_power, -1, 1);
    //   double newReference = (ShoulderConstants.kMaxVelocityRadPerSecond * deltaTime * p_power) 
    //       + m_shoulderGoal.position;
    //   setShoulderReference(MathUtil.clamp(newReference, Math.toRadians(100),Math.toRadians(360)));
    // }

    public double getGoal(){
        return m_currentGoal;
    }
   

    public boolean atGoal() {    
        return atGoal(m_positionTolerance);
    }

    public boolean atGoal(double tolerateRad) {
        return (Math.abs(m_currentGoal - getPositionRad()) < tolerateRad);
    }

    public void setGoalRad(double goalRad) {
        m_currentGoal = goalRad;
        setGoal(goalRad);
    }

    public double interpolateSetpoint(double p_distance){
      m_interpolatedValue = m_interpolater.getInterpolatedValue(p_distance);
      return m_interpolatedValue;
    }

    public double getInterpolateValue(){
      return m_interpolatedValue;
    }

    public void switchClimbPID(){
      m_shoulderPIDController.setP(ShoulderConstants.kShoulderClimbP);
      m_shoulderPIDController.setP(ShoulderConstants.kShoulderClimbI);
      m_shoulderPIDController.setP(ShoulderConstants.kShoulderClimbD);
      m_shoulderPIDController.setP(ShoulderConstants.kShoulderClimbFF);
    }

    // private double m_ShoulderP = ShoulderConstants.kShoulderP;
    // private double m_ShoulderI = ShoulderConstants.kShoulderI;
    // private double m_ShoulderD = ShoulderConstants.kShoulderD;
    // private double m_ShoulderFF = ShoulderConstants.kShoulderFF;
    // private double m_ShoulderS = ShoulderConstants.kSVolts;
    // private double m_ShoulderG = ShoulderConstants.kGVolts;
    // private double m_ShoulderVV = ShoulderConstants.kVVoltSecondPerRad;
    // private double m_ShoulderAV = ShoulderConstants.kAVoltSecondSquaredPerRad;
    // private double m_ShoulderWantedPosition = 0;

    // public void ShoulderPIDTuning(){
    //     if (firstPIDTesting){
    //       SmartDashboard.putNumber("ShoulderP", m_ShoulderP);
    //       SmartDashboard.putNumber("ShoulderI", m_ShoulderI);
    //       SmartDashboard.putNumber("ShoulderD", m_ShoulderD);
    //       SmartDashboard.putNumber("ShoulderFF", m_ShoulderFF);
    //       SmartDashboard.putNumber("ShoulderS", m_ShoulderS);
    //       SmartDashboard.putNumber("ShoulderG", m_ShoulderG);
    //       SmartDashboard.putNumber("ShoulderVV", m_ShoulderVV);
    //       SmartDashboard.putNumber("ShoulderAV", m_ShoulderAV);
          
    //       SmartDashboard.putNumber("ShoulderWantedPosition", -44);

    //       firstPIDTesting = false;
    //     }
      
    //     double m_ShoulderP_ = SmartDashboard.getNumber("ShoulderP", m_ShoulderP);
    //     double m_ShoulderI_ = SmartDashboard.getNumber("ShoulderI", m_ShoulderI);
    //     double m_ShoulderD_ = SmartDashboard.getNumber("ShoulderD", m_ShoulderD);
    //     double m_ShoulderFF_ = SmartDashboard.getNumber("ShoulderFF", m_ShoulderFF);
    //     double m_ShoulderS_ = SmartDashboard.getNumber("ShoulderS", m_ShoulderS);
    //     double m_ShoulderG_ = SmartDashboard.getNumber("ShoulderG", m_ShoulderG);
    //     double m_ShoulderVV_ = SmartDashboard.getNumber("ShoulderVV", m_ShoulderVV);
    //     double m_ShoulderAV_ = SmartDashboard.getNumber("ShoulderAV", m_ShoulderAV);
    //     double m_ShoulderWantedPosition_ = SmartDashboard.getNumber("ShoulderWantedPosition", m_ShoulderWantedPosition);
    //     if (m_ShoulderP_ != m_ShoulderP || m_ShoulderI_ != m_ShoulderI
    //         || m_ShoulderD_ != m_ShoulderD || m_ShoulderFF_ != m_ShoulderFF
    //         || m_ShoulderWantedPosition != m_ShoulderWantedPosition_
    //         || m_ShoulderS != m_ShoulderS_ || m_ShoulderG != m_ShoulderG_
    //         || m_ShoulderVV != m_ShoulderVV_ || m_ShoulderAV != m_ShoulderAV_){
          
    //       m_ShoulderP = m_ShoulderP_;
    //       m_ShoulderI = m_ShoulderI_;
    //       m_ShoulderD = m_ShoulderD_;
    //       m_ShoulderFF = m_ShoulderFF_;
    //       m_ShoulderS = m_ShoulderS_;
    //       m_ShoulderG = m_ShoulderG_;
    //       m_ShoulderVV = m_ShoulderVV_;
    //       m_ShoulderAV = m_ShoulderAV_;    
    //       m_ShoulderWantedPosition = m_ShoulderWantedPosition_;
    //       m_shoulderFeedForward = new ArmFeedforward(m_ShoulderS, m_ShoulderG, m_ShoulderVV, m_ShoulderAV);  
    //       m_shoulderPIDController.setP(m_ShoulderP,1);
    //       m_shoulderPIDController.setI(m_ShoulderI,1);
    //       m_shoulderPIDController.setD(m_ShoulderD,1);
    //       m_shoulderPIDController.setFF(m_ShoulderFF,1);
    //       setGoalRad(Math.toRadians(m_ShoulderWantedPosition));
    //       SmartDashboard.putNumber("ShoulderP", m_ShoulderP);
    //       SmartDashboard.putNumber("ShoulderI", m_ShoulderI);
    //       SmartDashboard.putNumber("ShoulderD", m_ShoulderD);
    //       SmartDashboard.putNumber("ShoulderFF", m_ShoulderFF);
    //       SmartDashboard.putNumber("ShoulderS", m_ShoulderS);
    //       SmartDashboard.putNumber("ShoulderG", m_ShoulderG);
    //       SmartDashboard.putNumber("ShoulderVV", m_ShoulderVV);
    //       SmartDashboard.putNumber("ShoulderAV", m_ShoulderAV);
    //       SmartDashboard.putNumber("ShoulderWantedPosition", m_ShoulderWantedPosition);
    //     }
    //   }

      public void setConstraint(TrapezoidProfile.Constraints p_constraints){
        setConstraints(p_constraints);
      }

  @Override
  public void useState(TrapezoidProfile.State setpoint) {
    m_shoulderPIDController.setReference(
      setpoint.position, 
      CANSparkMax.ControlType.kPosition,(1),
      m_shoulderFeedForward.calculate(setpoint.position,setpoint.velocity)
    );
    
    // SmartDashboard.putNumber("Shoulder feedforward", m_shoulderFeedForward.calculate(setpoint.position, setpoint.velocity));
    SmartDashboard.putNumber("ShoulderSetpoint", Math.toDegrees(setpoint.position));
    // SmartDashboard.putNumber("ShoulderSetpointVelocity", setpoint.velocity);
  }
  
}
