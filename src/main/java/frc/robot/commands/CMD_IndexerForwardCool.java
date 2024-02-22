
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IndexerForwardCool extends Command {
  SUB_Intake m_intake;
  boolean m_detected;
  boolean m_finished;
  double m_detectTimer;
  double m_stopTimer;
  double m_finshedTimer;
  double m_power;
  double m_IntialTimer;
  /** Creates a new CMD_IntakeForward. */
  public CMD_IndexerForwardCool(SUB_Intake p_intake, double p_power) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = p_intake;
    m_detected = false;
    m_power = p_power;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_IntialTimer = 0;
    m_detected = false;
    m_finished = false;
    m_detectTimer = 0;
    m_finshedTimer = 0;
    m_intake.setIndexerPower(m_power);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntialTimer +=1;
    // if (m_intake.getIndexerVelocity() < 3){
    //   if (m_stopTimer >= 10){
    //     m_finished = true;
    //   }else{
    //     m_stopTimer += 1;
    //   }
    // }else {
    //   m_stopTimer = 0;
    // }
    
    if (m_intake.getIndexerCurrent() >= 36 && !m_detected 
    && m_IntialTimer >= 18
    ){
      if (m_detectTimer >= 4 ){
        m_detected = true;
        // System.out.println("Done");
      }else  
        m_detectTimer += 1;
    } 
    else{
      m_detectTimer = 0;
    }

    if (m_detected == true){
    //   if (m_finshedTimer >= 4){
        m_finished = true;  
      }else {
    //     m_finshedTimer +=1;
      }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_finished){
      m_intake.setIndexerPower(0);
      m_stopTimer = 0;
      m_detectTimer = 0;
    }
    return m_finished;
    
  }
}

