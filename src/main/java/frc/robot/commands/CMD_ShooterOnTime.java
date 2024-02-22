// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_ShooterOnTime extends Command {
  /** Creates a new CMD_ShooterOnTime. */
  SUB_Shooter m_shooter;
  double m_velocity;
  Timer m_shooterTimer;
  double m_modifier;
  boolean m_finished;
  public CMD_ShooterOnTime(SUB_Shooter p_shooter, double p_velocity) {
    m_shooter = p_shooter;
    m_velocity = p_velocity;
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_modifier = (m_velocity / 4000);
    m_shooterTimer.start();
    m_shooterTimer.reset();
    m_finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_shooterTimer.get() >= m_modifier * 2.5){
      m_finished = true;
    }else{

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
