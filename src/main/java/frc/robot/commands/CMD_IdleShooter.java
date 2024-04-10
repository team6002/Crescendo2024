// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_IdleShooter extends Command {
  /** Creates a new CMD_IdleShooting. */
  SUB_Shooter m_shooter;
  SUB_Drivetrain m_drivetrain;
  SUB_GlobalVariables m_variables;
  public CMD_IdleShooter(SUB_Shooter p_shooter, SUB_Drivetrain p_drivetrain, SUB_GlobalVariables p_variables) {
    m_shooter = p_shooter;
    m_drivetrain = p_drivetrain;
    m_variables = p_variables;
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Units.metersToInches(m_drivetrain.calculateTargetDistance()) <= 200 && m_variables.getHasItem()){
      m_shooter.setTopPower(.25);  
      m_shooter.setBotPower(.25);

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return false;
  // }
}
