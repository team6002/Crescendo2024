// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Arm;

public class CMD_ElbowCheck extends Command {
  /** looks to see if the elbow is at location. */
  SUB_Arm m_arm;
  double m_tolerance = 2;
  public CMD_ElbowCheck(SUB_Arm p_arm, double p_tolerance) {
    m_arm = p_arm;
    m_tolerance = p_tolerance;
    addRequirements(m_arm);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_arm.atElbowGoal(Math.toRadians(m_tolerance));//checks to see if elbow is at the wanted position;
  }
}
