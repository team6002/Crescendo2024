// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Arm;

public class CMD_ShoulderSetPosition extends Command {
  SUB_Arm m_arm;
  double m_position;
  double m_tolerance = 5;
  /** Creates a new CMD_IntakeForward. */
  public CMD_ShoulderSetPosition(SUB_Arm p_arm, double p_position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = p_arm;
    m_position = p_position;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.setShoulderGoalWithoutElbow(m_position);
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
      return true;//checks to see if arm is at the wanted position
  }
}