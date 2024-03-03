package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SUB_Arm extends SubsystemBase {
    SUB_Elbow m_elbow;
    SUB_Shoulder m_shoulder;
    double ShooterAngleMod = 0;
    public SUB_Arm(SUB_Elbow p_elbow, SUB_Shoulder p_shoulder) {
        m_elbow = p_elbow;
        m_shoulder = p_shoulder;
        m_shoulder.enable();
        m_elbow.enable();
        
    }
    
    
    public double interpolateShoulder(double p_distance){
      return m_shoulder.interpolateSetpoint(p_distance);
    }
    
    public double getShoulderInterpolatedValue(){
        return m_shoulder.getInterpolateValue();
    }
    /**returns the current Shoulder goal */
    public double getShoulderGoal(){
      return m_shoulder.getGoal();
    }
    /** returns the shoulder position*/
    public double getShoulderPosition(){
      return m_shoulder.getPositionRad();
    }
    
    public double getShoulderVelocity(){
      return m_shoulder.getVelocity();
    }

    /** returns the Shoulder current */
    public double getShoulderCurrent(){
      return m_shoulder.getCurrent();
    }
    /** returns if the Shoulder is at the goal */
    public boolean atShoulderGoal(){
      return m_shoulder.atGoal();
    }   

    public void setShoulderConstraints(TrapezoidProfile.Constraints p_constraints){
        m_shoulder.setConstraints(p_constraints);
    }
    
    /** sets the Shoulder goal*/
    public void setShoulderGoalWithoutElbow(double goalRad){
        m_shoulder.setGoalRad(goalRad);
    }
    /** sets the Shoulder goal and moves the elbow so its same relative to the ground*/
    public void setShoulderGoalWithElbow(double goalRad){
        setShoulderGoalWithoutElbow(goalRad);
        double elbowGoal = (getElbowGoal() - getShoulderGoal());
        setElbowGoalRelative(elbowGoal);
    }
    /** turns on the Shoulder Trapazoid */
    public void useShoulder(){
        m_shoulder.enable();
    }
    
    public Command CMDsetShoulderConstraints(TrapezoidProfile.Constraints p_constraints){
        return Commands.runOnce(()->setShoulderConstraints(p_constraints),this);
    }
    /**returns the current Elbow goal */
    public double getElbowGoal(){
        return m_elbow.getGoal();
    }
    /** returns the elbow position in relation to the Shoulder */
    public double getElbowPosition(){
        return m_elbow.getPositionRad();
    }
    /** returns the Elbow current */
    public double getElbowCurrent(){
        return m_elbow.getCurrent();
    }
    /** returns if the Elbow is at the goal */
    public boolean atElbowGoal(){
        return m_elbow.atGoal();
    }
    public double interpolateElbow(double p_distance){
      return m_elbow.interpolateSetpoint(p_distance);
    }
    
    public double getElbowInterpolatedValue(){
        return m_elbow.getInterpolateValue();
    }
    
    public double interpolateShortElbow(double p_distance){
      return m_elbow.shortInterpolateSetpoint(p_distance);
    }
    
    public double getElbowShortInterpolatedValue(){
        return m_elbow.getShortInterpolateValue();
    }
    /** sets the Elbow goal in relation to the Shoulder */
    public void setElbowGoalRelative(double goalRad){
        m_elbow.setGoalRad(MathUtil.clamp(goalRad, Math.toRadians(10), Math.toRadians(115)));
    }
    /** sets the Elbow goal in relation to the Ground */
    public void setElbowGoalAbsolute(double goalRad){
        // double newGoal = goalRad + getShoulderPosition();
        double newGoal = goalRad + getShoulderGoal();
        m_elbow.setGoalRad(MathUtil.clamp(newGoal, Math.toRadians(10), Math.toRadians(115)));
    }
    /** turns on the Shoulder Trapazoid */
    public void useElbow(){
        m_elbow.enable();
    }
    /** sets both the elbow and arm at the same time */
    public void setArmGoals(double elbowGoalRad, double shoulderGoalRad){
        setElbowGoalAbsolute(elbowGoalRad);
        setShoulderGoalWithElbow(shoulderGoalRad);
    }
    
    public void setLHookPWM(double p_PWM){
        m_shoulder.setLHookPWM(p_PWM);
    }
    
    public void setRHookPWM(double p_PWM){
        m_shoulder.setRHookPWM(p_PWM);
    }

    public Command CMDsetLHookPWM(double p_PWM){
        return m_shoulder.CMDsetLHookPWM(p_PWM);
    }
    public Command CMDsetRHookPWM(double p_PWM){
        return m_shoulder.CMDsetRHookPWM(p_PWM);
    }

    // public Command CMDsetHooksPWM(double p_PWM){
    //     return m_shoulder.CMDsetHooksPWM(p_PWM);
    // }
    public double getShooterAngMod(){
        return ShooterAngleMod;
    }
    @Override
    public void periodic() {
        

        SmartDashboard.putBoolean("ShoulderFin", m_shoulder.atGoal());
        SmartDashboard.putBoolean("ElbowFin", m_elbow.atGoal());
        // SmartDashboard.putNumber("ShooterAngleMod", ShooterAngleMod);
        // ShooterAngleMod = SmartDashboard.getNumber("ShooterAngleMod", 0);
        // m_elbow.ElbowPIDTuning();
        // m_shoulder.ShoulderPIDTuning();

        // SmartDashboard.putNumber("Elbow Current", getElbowCurrent());
        // SmartDashboard.putNumber("Elbow Goal", Math.toDegrees(getElbowGoal()));
        SmartDashboard.putNumber("Elbow Position", Math.toDegrees(getElbowPosition()));
        // SmartDashboard.putBoolean("ElbowAtGoal", m_elbow.atGoal());

        // SmartDashboard.putNumber("Shoulder Current", getShoulderCurrent());
        // SmartDashboard.putNumber("Shoulder Goal", Math.toDegrees(getShoulderGoal()));
        SmartDashboard.putNumber("Shoulder Position", Math.toDegrees(getShoulderPosition()));
        // SmartDashboard.putNumber("Shoulder Velocity", getShoulderVelocity());
    }
  
}
