// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Intake;

public class CMD_PickUpRumble extends Command {
  /** just rumbles the controller when we pick up*/
  SUB_Intake m_intake;
  XboxController m_driverController;
  double m_pickuptimer;
  public CMD_PickUpRumble(SUB_Intake p_intake, XboxController p_driverController) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = p_intake;
    m_driverController = p_driverController;
    m_requirements.add(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_intake.getIndexerSensor()){
      m_pickuptimer += 1;
      if (m_pickuptimer <= 150){
        m_driverController.setRumble(RumbleType.kBothRumble, 1); 
      }else {
        m_driverController.setRumble(RumbleType.kBothRumble, 0);
      }
    }else {
      m_driverController.setRumble(RumbleType.kBothRumble, 0);
      m_pickuptimer = 0;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_driverController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }
}
