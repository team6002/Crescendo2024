// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_GlobalVariables;

public class CMD_CycleIntakeType extends Command {
  /** Creates a new CMD_CycleIntakeType. */
  SUB_GlobalVariables m_variables;
  public CMD_CycleIntakeType(SUB_GlobalVariables p_variables) {
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_variables.getIntakeType() >= 1){
      m_variables.setIntakeType(0);
    }else {
      m_variables.setIntakeType(m_variables.getIntakeType() + 1);
    }
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
    return true;
  }
}
