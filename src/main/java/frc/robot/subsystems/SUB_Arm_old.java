// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import com.revrobotics.CANSparkMax;
// import com.revrobotics.SparkPIDController;
// import com.revrobotics.CANSparkBase.IdleMode;
// import com.revrobotics.AbsoluteEncoder;
// import com.revrobotics.CANSparkLowLevel.MotorType;
// import com.revrobotics.SparkAbsoluteEncoder.Type;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants.ElbowConstants;
// import frc.robot.Constants.ShoulderConstants;

// public class SUB_Arm_old extends SubsystemBase {

//   private final CANSparkMax m_elbowMotor;
//   private SparkPIDController m_elbowPIDController;
//   private AbsoluteEncoder m_elbowAbsoluteEncoder;
    
//   private ArmFeedforward m_elbowFeedForward;
//   private TrapezoidProfile.Constraints m_elbowConstraints;
//   private TrapezoidProfile.State m_elbowGoal;
//   private TrapezoidProfile.State m_elbowSetpoint;
//   private TrapezoidProfile m_elbowProfile;
  
//   private final CANSparkMax m_shoulderMotor;
//   private final CANSparkMax m_shoulderFollowerMotor;
//   private SparkPIDController m_shoulderPIDController;
//   private AbsoluteEncoder m_shoulderAbsoluteEncoder;
//   // private final RelativeEncoder m_shoulderEncoder;

  
//   private ArmFeedforward m_shoulderFeedForward;
//   private TrapezoidProfile.Constraints m_shoulderConstraints;
//   private TrapezoidProfile.State m_shoulderGoal;
//   private TrapezoidProfile.State m_shoulderSetpoint;
//   private TrapezoidProfile m_shoulderProfile;
//   //makes the PID tuning display all the Smartdashboard stuff it needs
//   private boolean firstPIDTesting = true;

//   //returns the angle the elbow should be to the ground
//   // removed by hdn
//   // private double m_elbowRelativeGround; 
//   private static double deltaTime = 0.02;
  
//   /** Creates a new SUB_Shooter. */
//   public SUB_Arm_old() {
//     m_elbowMotor = new CANSparkMax(ElbowConstants.kElbowMotorCANID, MotorType.kBrushless);
//     m_elbowMotor.restoreFactoryDefaults();
//     m_elbowPIDController = m_elbowMotor.getPIDController();
//     m_elbowAbsoluteEncoder = m_elbowMotor.getAbsoluteEncoder(Type.kDutyCycle);
//     m_elbowMotor.setIdleMode(IdleMode.kBrake);
//     m_elbowMotor.setInverted(true);
//     m_elbowMotor.setSmartCurrentLimit(ElbowConstants.kElbowCurrentLimit);
    
//     m_elbowAbsoluteEncoder.setPositionConversionFactor(ElbowConstants.kPositionConversionFactor);
//     m_elbowAbsoluteEncoder.setVelocityConversionFactor(ElbowConstants.kVelocityConversionFactor);

//     m_elbowPIDController.setFeedbackDevice(m_elbowAbsoluteEncoder);

//     m_elbowPIDController.setP(ElbowConstants.kElbowP,1);
//     m_elbowPIDController.setI(ElbowConstants.kElbowI,1);
//     m_elbowPIDController.setD(ElbowConstants.kElbowD,1);
//     m_elbowPIDController.setFF(ElbowConstants.kElbowFF,1);
    
//     m_elbowPIDController.setPositionPIDWrappingEnabled(false);
//     // m_elbowPIDController.setPositionPIDWrappingMinInput(-180);
//     // m_elbowPIDController.setPositionPIDWrappingMaxInput(180);

//     m_elbowFeedForward = new ArmFeedforward(ElbowConstants.kSVolts,ElbowConstants.kGVolts
//         , ElbowConstants.kVVoltSecondPerRad,ElbowConstants.kAVoltSecondSquaredPerRad);  

//     m_elbowConstraints = new TrapezoidProfile.Constraints(ElbowConstants.kMaxVelocityRadPerSecond, ElbowConstants.kMaxAccelerationRadPerSecSquared);
//     // m_elbowSetpoint = new TrapezoidProfile.State(getElbowPosition(), 0); 
//     // m_elbowGoal = m_elbowSetpoint;
//     m_elbowGoal = new TrapezoidProfile.State(getElbowPositionRad(), 0); 
//     m_elbowProfile = new TrapezoidProfile(m_elbowConstraints);
    
//     m_shoulderMotor = new CANSparkMax(ShoulderConstants.kShoulderMotorCANID, MotorType.kBrushless);
//     m_shoulderFollowerMotor = new CANSparkMax(ShoulderConstants.kShoulderFollowerMotorCANID, MotorType.kBrushless);
//     m_shoulderPIDController = m_shoulderMotor.getPIDController();
//     m_shoulderAbsoluteEncoder = m_shoulderMotor.getAbsoluteEncoder(Type.kDutyCycle);
//     // m_shoulderEncoder = m_shoulderMotor.getEncoder();
//     m_shoulderMotor.setInverted(true);
//     m_shoulderFollowerMotor.follow(m_shoulderMotor, true);

//     m_shoulderMotor.setIdleMode(IdleMode.kBrake);
//     m_shoulderFollowerMotor.setIdleMode(IdleMode.kBrake);
//     m_shoulderMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderCurrentLimit);
//     m_shoulderFollowerMotor.setSmartCurrentLimit(ShoulderConstants.kShoulderCurrentLimit);

//     // m_shoulderEncoder.setPositionConversionFactor(ShoulderConstants.kShoulderPositionFactor);
//     m_shoulderAbsoluteEncoder.setPositionConversionFactor(ShoulderConstants.kPositionConversionFactor);
//     m_shoulderAbsoluteEncoder.setVelocityConversionFactor(ShoulderConstants.kVelocityConversionFactor);

//     m_shoulderPIDController.setFeedbackDevice(m_shoulderAbsoluteEncoder);

//     m_shoulderPIDController.setP(ShoulderConstants.kShoulderP,1);
//     m_shoulderPIDController.setI(ShoulderConstants.kShoulderI,1);
//     m_shoulderPIDController.setD(ShoulderConstants.kShoulderD,1);
//     m_shoulderPIDController.setFF(ShoulderConstants.kShoulderFF,1);

//     m_shoulderPIDController.setPositionPIDWrappingEnabled(false); 
//     // m_shoulderPIDController.setPositionPIDWrappingMinInput(-180);
//     // m_shoulderPIDController.setPositionPIDWrappingMaxInput(180);
       
//     m_shoulderFeedForward = new ArmFeedforward(ShoulderConstants.kSVolts, ShoulderConstants.kGVolts
//                 ,ShoulderConstants.kVVoltSecondPerRad,ShoulderConstants.kAVoltSecondSquaredPerRad);   
//     m_shoulderConstraints = new TrapezoidProfile.Constraints(ShoulderConstants.kMaxVelocityRadPerSecond, ShoulderConstants.kMaxAccelerationRadPerSecSquared);
//     // m_shoulderSetpoint = new TrapezoidProfile.State(getShoulderPosition(), 0); 
//     // m_shoulderGoal = m_shoulderSetpoint;
//     m_shoulderGoal = new TrapezoidProfile.State(getShoulderPositionRad(), 0); 
//     m_shoulderProfile = new TrapezoidProfile(m_shoulderConstraints);

//     m_elbowMotor.burnFlash();
//     m_shoulderMotor.burnFlash();

//   }

//      public void elbowInit(){
//       // m_elbowRelativeGround = getElbowPosition() + getShoulderPositionWrapped();
//       // m_elbowSetpoint = new TrapezoidProfile.State(getElbowPosition(), 0); 
//       // m_elbowGoal = m_elbowSetpoint;
//       m_elbowGoal = new TrapezoidProfile.State(getElbowPositionRad(), 0); 
//     }

//     /**sets a elbow reference based off the ground*/ 
//     public void setElbowReference(double rad){
//       double relativeReference = MathUtil.clamp(rad + getShoulderPositionRad(), Math.toRadians(17),Math.toRadians(115));
//       m_elbowGoal = new TrapezoidProfile.State(relativeReference, 0);
//     }

//     //Wraps the Shoulder position around 180
//     // public double getShoulderPositionWrapped(){
//     //   return MathUtil.inputModulus(getShoulderPosition(), -180, 180);
//     // }

//     public double getElbowPositionRad(){
//       return MathUtil.angleModulus(m_elbowAbsoluteEncoder.getPosition());
//     }

//     public void setElbowConstraints(double p_velocity, double p_acceleration){
//       m_elbowConstraints = new TrapezoidProfile.Constraints(p_velocity, p_acceleration);
//     }

//     public double getElbowVelocity(){
//       return m_elbowAbsoluteEncoder.getVelocity();
//     }
  
//     public double getElbowCurrent(){
//       return m_elbowMotor.getOutputCurrent();
//     }
    
//     public void elbowManualMove(double p_power){
//       double newReference = (ElbowConstants.kMaxVelocityRadPerSecond  * deltaTime * p_power) 
//         + m_elbowGoal.position;
//       setElbowReference(newReference);
//     }

//     public double getElbowWantedPosition(){
//       return m_elbowGoal.position;
//     }
  
//     public void shoulderInit(){
//       // m_shoulderSetpoint = new TrapezoidProfile.State(getShoulderPosition(), 0); 
//       // m_shoulderGoal = m_shoulderSetpoint;
//        m_shoulderGoal = new TrapezoidProfile.State(getShoulderPositionRad(), 0); 
//     }

//     /**Only moves the shoulder*/
//     public void setShoulderReferenceWithoutElbow(double rad){
//       // m_shoulderSetpoint = new TrapezoidProfile.State(getShoulderPosition(),0);
//       m_shoulderGoal = new TrapezoidProfile.State(rad, 0);
//     }

//     /**sets a reference to both the shoulder and the elbow to keep the elbow position the same realtive to the ground*/
//     public void setShoulderReference(double rad){
//       double elbowRelativeGoal = (m_elbowGoal.position - m_shoulderGoal.position);
//       m_shoulderGoal = new TrapezoidProfile.State(rad, 0);
//       setElbowReference(elbowRelativeGoal);

//       // m_elbowGoal = new TrapezoidProfile.State(
//       //   MathUtil.clamp(getElbowRelativePosition() + MathUtil.inputModulus(p_reference, -180, 180), 17, 115), 0); 
//     }

//     // public double getShoulderPosition(){
//     //   return m_shoulderEncoder.getPosition();
//     // }

//     public double getShoulderPositionRad(){
//       return MathUtil.angleModulus(m_shoulderAbsoluteEncoder.getPosition());
//     }

//     public double getShoulderVelocity(){
//       return m_shoulderAbsoluteEncoder.getVelocity();
//     }
    
//     public void setShoulderConstraints(double p_velocity, double p_acceleration){
//       m_shoulderConstraints = new TrapezoidProfile.Constraints(p_velocity, p_acceleration);
//     }
  
//     public double getShoulderCurrent(){
//       return m_shoulderMotor.getOutputCurrent();
//     }
    
//     public double getShoulderWantedPosition(){
//       return m_shoulderGoal.position;
//     }
    
//     public boolean getShoulderAtGoal(double tolerate) {
//       return ( Math.abs(getShoulderPositionRad() - m_shoulderGoal.position) <= tolerate);
//     }

//     public void shoulderManualMove(double p_power){
//       // double joystickIntepertation = MathUtil.clamp(p_power, -1, 1);
//       double newReference = (ShoulderConstants.kMaxVelocityRadPerSecond * deltaTime * p_power) 
//           + m_shoulderGoal.position;
//       setShoulderReference(MathUtil.clamp(newReference, Math.toRadians(100),Math.toRadians(360)));
//     }

//     // private double m_ShoulderP = ShoulderConstants.kShoulderP;
//     // private double m_ShoulderI = ShoulderConstants.kShoulderI;
//     // private double m_ShoulderD = ShoulderConstants.kShoulderD;
//     // private double m_ShoulderFF = ShoulderConstants.kShoulderFF;
//     // private double m_ShoulderWantedPosition = 0;

//     // public void ShoulderPIDTuning(){
//     //   if (firstPIDTesting){
//     //     SmartDashboard.putNumber("ShoulderP", m_ShoulderP);
//     //     SmartDashboard.putNumber("ShoulderI", m_ShoulderI);
//     //     SmartDashboard.putNumber("ShoulderD", m_ShoulderD);
//     //     SmartDashboard.putNumber("ShoulderFF", m_ShoulderFF);
//     //     SmartDashboard.putNumber("ShoulderWantedPosititon", m_ShoulderWantedPosition);
//     //     firstPIDTesting = false;
//     //   }
    
//     //   double m_ShoulderP_ = SmartDashboard.getNumber("ShoulderP", m_ShoulderP);
//     //   double m_ShoulderI_ = SmartDashboard.getNumber("ShoulderI", m_ShoulderI);
//     //   double m_ShoulderD_ = SmartDashboard.getNumber("ShoulderD", m_ShoulderD);
//     //   double m_ShoulderFF_ = SmartDashboard.getNumber("ShoulderFF", m_ShoulderFF);
//     //   double m_ShoulderWantedPosition_ = SmartDashboard.getNumber("ShoulderWantedPosition", 0);
//     //   if (m_ShoulderP_ != m_ShoulderP || m_ShoulderI_ != m_ShoulderI
//     //       || m_ShoulderD_ != m_ShoulderD || m_ShoulderFF_ != m_ShoulderFF
//     //       || m_ShoulderWantedPosition_ != m_ShoulderWantedPosition){
//     //     m_ShoulderP = m_ShoulderP_;
//     //     m_ShoulderI = m_ShoulderI_;
//     //     m_ShoulderD = m_ShoulderD_;
//     //     m_ShoulderFF = m_ShoulderFF_;  
//     //     m_ShoulderWantedPosition = m_ShoulderWantedPosition_;
//     //     m_shoulderPIDController.setP(m_ShoulderP);
//     //     m_shoulderPIDController.setI(m_ShoulderI);
//     //     m_shoulderPIDController.setD(m_ShoulderD);
//     //     m_shoulderPIDController.setFF(m_ShoulderFF);
//     //     setShoulderReferenceWithoutElbow(m_ShoulderWantedPosition);
//     //     SmartDashboard.putNumber("ShoulderP", m_ShoulderP);
//     //     SmartDashboard.putNumber("ShoulderI", m_ShoulderI);
//     //     SmartDashboard.putNumber("ShoulderD", m_ShoulderD);
//     //     SmartDashboard.putNumber("ShoulderFF", m_ShoulderFF);
//     //     SmartDashboard.putNumber("ShoulderWantedPosititon", m_ShoulderWantedPosition);
//     //   }

//     // }

 
//     // private double m_ElbowP = ElbowConstants.kElbowP;
//     // private double m_ElbowI = ElbowConstants.kElbowI;
//     // private double m_ElbowD = ElbowConstants.kElbowD;
//     // private double m_ElbowFF = ElbowConstants.kElbowFF;
//     // private double m_ElbowWantedPosition = 90;

//     // public void ElbowPIDTuning(){
//     //   if (firstPIDTesting){
//     //     SmartDashboard.putNumber("ElbowP", m_ElbowP);
//     //     SmartDashboard.putNumber("ElbowI", m_ElbowI);
//     //     SmartDashboard.putNumber("ElbowD", m_ElbowD);
//     //     SmartDashboard.putNumber("ElbowFF", m_ElbowFF);
//     //     SmartDashboard.putNumber("ElbowWantedPosititon", m_ElbowWantedPosition);
//     //     firstPIDTesting = false;
//     //   }
    
//     //   double m_ElbowP_ = SmartDashboard.getNumber("ElbowP", m_ElbowP);
//     //   double m_ElbowI_ = SmartDashboard.getNumber("ElbowI", m_ElbowI);
//     //   double m_ElbowD_ = SmartDashboard.getNumber("ElbowD", m_ElbowD);
//     //   double m_ElbowFF_ = SmartDashboard.getNumber("ElbowFF", m_ElbowFF);
//     //   double m_ElbowWantedPosition_ = SmartDashboard.getNumber("ElbowWantedPosition", 90);
//     // // noice coding :D
//     //   if (m_ElbowP_ != m_ElbowP || m_ElbowI_ != m_ElbowI
//     //       || m_ElbowD_ != m_ElbowD || m_ElbowFF_ != m_ElbowFF
//     //       || m_ElbowWantedPosition_ != m_ElbowWantedPosition){
//     //     m_ElbowP = m_ElbowP_;
//     //     m_ElbowI = m_ElbowI_;
//     //     m_ElbowD = m_ElbowD_;
//     //     m_ElbowFF = m_ElbowFF_;  
//     //     m_ElbowWantedPosition = m_ElbowWantedPosition_;
//     //     m_elbowPIDController.setP(m_ElbowP);
//     //     m_elbowPIDController.setI(m_ElbowI);
//     //     m_elbowPIDController.setD(m_ElbowD);
//     //     m_elbowPIDController.setFF(m_ElbowFF);
//     //     setElbowReference(m_ElbowWantedPosition);
//     //     SmartDashboard.putNumber("ElbowP", m_ElbowP);
//     //     SmartDashboard.putNumber("ElbowI", m_ElbowI);
//     //     SmartDashboard.putNumber("ElbowD", m_ElbowD);
//     //     SmartDashboard.putNumber("ElbowFF", m_ElbowFF);
//     //     SmartDashboard.putNumber("ElbowWantedPosititon", m_ElbowWantedPosition);
//     //   }

//     // }

//     public double getElbowRelativeGround() {
//       return Math.toDegrees(getElbowPositionRad() - getShoulderPositionRad());
//     }

//   @Override
//   public void periodic() {
//     // ShoulderPIDTuning();
//     // ElbowPIDTuning();
//     // // This method will be called once per scheduler run
//     m_elbowSetpoint = m_elbowProfile.calculate(deltaTime, m_elbowSetpoint, m_elbowGoal);
//     m_elbowPIDController.setReference(
//       m_elbowSetpoint.position, 
//       CANSparkMax.ControlType.kPosition,(1),
//       m_elbowFeedForward.calculate(m_elbowSetpoint.position,m_elbowSetpoint.velocity)
//     );

//     m_shoulderSetpoint = m_shoulderProfile.calculate(deltaTime, m_shoulderSetpoint, m_shoulderGoal);
//     m_shoulderPIDController.setReference(
//       m_shoulderSetpoint.position, 
//       CANSparkMax.ControlType.kPosition,(1),
//       m_shoulderFeedForward.calculate(m_shoulderSetpoint.position,m_shoulderSetpoint.velocity)
//     );


//     // SmartDashboard.putNumber("Shoulder Position", m_shoulderEncoder.getPosition());
//     SmartDashboard.putNumber("Absoulute Shoulder Position", Math.toDegrees(getShoulderPositionRad()));
//     SmartDashboard.putNumber("ShoulderVoltage", m_shoulderMotor.getOutputCurrent());
//     SmartDashboard.putNumber("ShoulderWantedPosition", getShoulderWantedPosition());

//     SmartDashboard.putNumber("Elbow Relative Position", getElbowRelativeGround());
//     SmartDashboard.putNumber("Absoulute Elbow Position", getElbowPositionRad());
//     SmartDashboard.putNumber("ElbowVoltage", m_elbowMotor.getOutputCurrent());
//     SmartDashboard.putNumber("ElbowWantedPosition", getElbowWantedPosition());
//   }
// }
