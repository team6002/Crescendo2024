// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_LED;
import frc.robot.subsystems.SUB_Shooter;
import frc.utils.Color;
import frc.utils.pattern.ChasePattern;

public class CMD_LedHandler extends Command {
  /** Creates a new CMD_LedHandler. */
  SUB_LED m_led;
  SUB_Shooter m_shooter;
  SUB_GlobalVariables m_variables;
  Timer m_updateTimer;
  boolean m_prevHasItem;
  int m_prevOutType;
  public CMD_LedHandler(SUB_LED p_led, SUB_Shooter p_shooter, SUB_GlobalVariables p_variables) {
    m_led = p_led;
    m_shooter = p_shooter;
    m_variables = p_variables;
    m_requirements.add(m_led);
    m_updateTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_variables.setChangeState(false);
    m_prevHasItem = m_variables.getHasItem();
    m_prevOutType = m_variables.getOutputType();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_prevHasItem != m_variables.getHasItem() || m_prevOutType != m_variables.getOutputType()){
      m_variables.setChangeState(true);
    }
    if (m_variables.getChangeState()){
      m_variables.setChangeState(false);
      if (m_variables.getHasItem()){
        if (m_variables.getOutputType() == 0 || m_variables.getOutputType() == 2){
          m_led.LEDShootingMode();
        }else if (m_variables.getOutputType() ==1 || m_variables.getOutputType() == 3){
          m_led.LEDAmpMode();
        }
      }else {
        m_led.LEDIntakeMode();
      }
    }
    if (m_variables.getHasItem()){
      m_led.setLEDDuration(-MathUtil.clamp(Math.abs(500/m_shooter.getBotShooterVelocity()), .1, 1.5));
    }else {
      
    }

    m_prevHasItem = m_variables.getHasItem();
    m_prevOutType = m_variables.getOutputType();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
