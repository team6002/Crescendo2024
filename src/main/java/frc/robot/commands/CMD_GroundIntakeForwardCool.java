
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Intake;

public class CMD_GroundIntakeForwardCool extends Command {
  SUB_Intake m_intake;
  boolean m_detected;
  boolean m_finished;
  double m_detectTimer;
  double m_stopTimer;
  double m_velocity;
  /** Creates a new CMD_IntakeForward. */
  public CMD_GroundIntakeForwardCool(SUB_Intake p_intake, double p_velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = p_intake;
    m_detected = false;
    m_velocity = p_velocity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_detected = false;
    m_finished = false;
    m_detectTimer = 0;
    m_intake.setIntakeVelocity(m_velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getIntakeVelocity() < 3){
      if (m_stopTimer >= 10){
      // m_finished = true;
      }else{
        m_stopTimer += 1;
      }
    }else {
      m_stopTimer = 0;
    }
    // SmartDashboard.putNumber("StopTimer", m_stopTimer);
    // SmartDashboard.putBoolean("Detected", m_detected);

    if (m_intake.getIntakeCurrent() >= 15 && !m_detected ){
      if (m_detectTimer >= 8 ){
        m_detected = true;
      }else  
        m_detectTimer += 1;
    } 
    else{
      // m_detected = false;
      m_detectTimer = 0;
    }
    if (m_detected == true && m_intake.getIntakeCurrent() <= 10){
      m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (m_finished){
    m_intake.setIntakePower(0);
    m_stopTimer = 0;
    m_detectTimer = 0;
    }
    return m_finished;
    
  }
}

