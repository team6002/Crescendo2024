// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.hal.simulation.DriverStationDataJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;

public class CMD_OperatorRumble extends Command {
  /** Creates a new CMD_OperatoRumble. */
  Timer m_endgameTimer;
  XboxController m_operatorController;
  
  public CMD_OperatorRumble(XboxController p_operatorController) {
    m_endgameTimer = new Timer();
    m_operatorController = p_operatorController;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (DriverStation.getMatchTime() <= 40){
      if (DriverStation.getMatchTime() <= 37){
        m_operatorController.setRumble(RumbleType.kBothRumble, 0);
    }else {
      m_operatorController.setRumble(RumbleType.kBothRumble, 1);
    }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   return true;
  // }
}
