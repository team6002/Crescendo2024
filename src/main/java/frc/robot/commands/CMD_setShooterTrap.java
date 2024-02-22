// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_setShooterTrap extends Command {
  /** Creates a new CMD_setShooterSetpoint. */
  SUB_Shooter m_shooter;
  double m_speed;
  public CMD_setShooterTrap(SUB_Shooter p_shooter, double p_speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = p_shooter;
    m_speed = p_speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println(m_speed);
    m_shooter.setTopSetpoint(-m_speed);
    m_shooter.setBotSetpoint(m_speed);
    // m_shooter.setShooterSetpoint(m_speed);
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
