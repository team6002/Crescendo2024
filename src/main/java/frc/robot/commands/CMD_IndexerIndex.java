// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Intake;

public class CMD_IndexerIndex extends Command {
  /** Creates a new CMD_IndexerTest. */
  SUB_Intake m_intake;
  boolean m_detected;
  Timer m_intakerTimer;
  double m_indexerSpeed;
  public CMD_IndexerIndex(SUB_Intake p_intake) {
    m_intake = p_intake;
    m_intakerTimer = new Timer();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_detected = false;
    m_intakerTimer.reset();
    m_intakerTimer.stop();
    m_indexerSpeed = 0.2;
    // m_intake.enableIndexerLimit(true);
    m_intake.setIndexerPower(.5);
    // m_intake.setIndexerVelocity(2400);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (m_intake.getIndexerSensor()){
      m_intakerTimer.start();
      // System.out.println("FOUND");
      // if (m_intakerTimer.get() < 0.02){
      //   m_intake.setIndexerVelocity(-0.01);
      // }else{
      m_detected = true;
      // }
    }else if (m_intake.getIndexerCurrent() >= 24){
      m_indexerSpeed *= 0.8;
       m_intake.setIndexerPower(m_indexerSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_intake.setIndexerVelocity(-0.);
      m_intake.setIntakePower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_detected;
  }
}
