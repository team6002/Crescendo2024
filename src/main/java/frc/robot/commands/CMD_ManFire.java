// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_ManFire extends Command {
  /** Creates a new CMD_Autofire. */
  SUB_Arm m_arm;
  SUB_Drivetrain m_drivetrain;
  SUB_Intake m_intake;
  SUB_Shooter m_shooter;
  SUB_GlobalVariables m_variable;
  boolean m_shot;
  Timer m_shooterTimer;
  boolean m_firingStarted;
  public CMD_ManFire(SUB_Arm p_arm, SUB_Drivetrain p_drivetrain, SUB_Intake p_intake, SUB_Shooter p_shooter, SUB_GlobalVariables p_variables) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = p_arm;
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_shooter = p_shooter;
    m_variable = p_variables;
    m_firingStarted = false;
    m_shooterTimer = new Timer();
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shot = false;
    m_firingStarted = false;
    m_shooterTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!m_firingStarted){
      m_shooter.enableShooter();
      m_firingStarted = true;
    }
    m_shooter.setShooterSetpoint(m_shooter.interpolateSetpoint(120));
      m_arm.setShoulderGoalWithoutElbow(m_arm.interpolateShoulder(120));
      m_arm.setElbowGoalRelative(m_arm.interpolateShortElbow(120));
    // if (m_arm.atShoulderGoal() && m_arm.atElbowGoal() && m_shooter.getAtShooterSetpoint() && m_drivetrain.getOnTarget()){
    // }
    if (m_arm.atShoulderGoal() && m_arm.atElbowGoal() && m_shooter.getAtShooterSetpoint()){
      m_shooterTimer.start();
    }else{
      m_shooterTimer.stop();
    }
      if (m_shooterTimer.get() > 0.5){
        m_intake.setIndexerVelocity(4000);
        if (m_shooterTimer.get() > 0.7){
          m_shot = true;
        }
      }
  }
    
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("DONE");
    if (m_shot){
      if (m_variable.getContShooting()){
        m_shooter.setShooterSetpoint(2000);
      }else{
        m_shooter.disableShooter();
      }
      m_intake.setIndexerVelocity(0);
    }
    return m_shot;
  }
}
