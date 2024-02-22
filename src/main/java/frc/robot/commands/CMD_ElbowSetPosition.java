// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Arm;

public class CMD_ElbowSetPosition extends Command {
  SUB_Arm m_elbow;
  double m_position;
  double m_tolerance = 5;
  /** Set the Elbow relative to the ground. */
  public CMD_ElbowSetPosition(SUB_Arm p_elbow, double p_position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_elbow = p_elbow;
    m_position = p_position;
    addRequirements(m_elbow);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elbow.setElbowGoalAbsolute(m_position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return true;//checks to see if elbow is at the wanted position
  }
}