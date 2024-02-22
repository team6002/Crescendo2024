// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IndexerShelfIndex extends Command {
  /** Creates a new CMD_IndexerTest. */
  SUB_Intake m_intake;
  double m_startTimer;
  double m_detectedCount;
  boolean m_detected;
  boolean m_currentlyDetected;
  public CMD_IndexerShelfIndex(SUB_Intake p_intake) {
    m_intake = p_intake;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_startTimer = 0;
    m_detected = false;
    m_currentlyDetected = false;
    // m_intake.enableIndexerLimit(false);
    m_intake.setIndexerVelocity(-1000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_startTimer += 1;

    if (m_intake.getIndexerSensor() != m_currentlyDetected && m_intake.getIndexerSensor()){
      // System.out.println("Backed");
      m_currentlyDetected = true;
      m_detectedCount +=1;
    }
    if (m_detectedCount ==2){
      m_detected = true;
      m_intake.setIndexerVelocity(0);
  
    } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_detected;
  }
}
