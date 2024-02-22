// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IndexerMoveDistance extends Command {
  /** Moves the Indexer a set distance */
  SUB_Intake m_intake;
  double m_power;
  double m_distance;
  double m_newPosition;
  double m_tolerance = 10;
  boolean m_finished;
  public CMD_IndexerMoveDistance(SUB_Intake p_intake, double p_distance, double p_power) {
    m_intake = p_intake;
    m_distance = p_distance;
    m_power = p_power;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_newPosition = m_intake.getIndexerPosition() - m_distance; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(m_intake.getIndexerPosition() - m_newPosition) >= m_tolerance){
      m_finished = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_finished;
  }
}
