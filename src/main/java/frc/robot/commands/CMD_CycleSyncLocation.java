// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LocationConstants;
import frc.robot.subsystems.SUB_GlobalVariables;

public class CMD_CycleSyncLocation extends Command {
  /** Creates a new CMD_CycleIntakeType. */
  SUB_GlobalVariables m_variables;
  double m_locationStage;
  public CMD_CycleSyncLocation(SUB_GlobalVariables p_variables) {
    m_variables = p_variables;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_locationStage < 2){
      m_locationStage += 1;
    }else{
      m_locationStage = 0;
    }
    if (DriverStation.getAlliance().get() == Alliance.Red){
      if (m_locationStage == 0){
        m_variables.setSyncLocation(LocationConstants.SubwooferRed);
      }else if (m_locationStage == 1){
        m_variables.setSyncLocation(LocationConstants.AmpRed);   
      }else if (m_locationStage == 2){
        m_variables.setSyncLocation(LocationConstants.LSourceRed);
      }
    }else{
      if (m_locationStage == 0){
        m_variables.setSyncLocation(LocationConstants.SubwooferBlue);
      }else if (m_locationStage == 1){
        m_variables.setSyncLocation(LocationConstants.AmpBlue);   
      }else if (m_locationStage == 2){
        m_variables.setSyncLocation(LocationConstants.LSourceBlue);
      }  
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
