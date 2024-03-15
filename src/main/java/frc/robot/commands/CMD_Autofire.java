// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_Autofire extends Command {
  /** Creates a new CMD_Autofire. */
  SUB_Arm m_arm;
  SUB_Drivetrain m_drivetrain;
  SUB_Intake m_intake;
  SUB_Shooter m_shooter;
  SUB_GlobalVariables m_variable;
  boolean m_shot;
  Timer m_shooterTimer;
  Timer m_altShooterTimer;
  Timer m_intialTimer;
  boolean m_firingStarted;
  boolean m_closeShooting;
  public CMD_Autofire(SUB_Arm p_arm, SUB_Drivetrain p_drivetrain, SUB_Intake p_intake, SUB_Shooter p_shooter, SUB_GlobalVariables p_variables) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = p_arm;
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_shooter = p_shooter;
    m_variable = p_variables;
    m_firingStarted = false;
    m_shooterTimer = new Timer();
    m_altShooterTimer = new Timer(); 
    m_intialTimer = new Timer();
    m_closeShooting = false;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shot = false;
    m_closeShooting = false;
    m_firingStarted = false;
    m_shooterTimer.restart();
    m_altShooterTimer.restart();
    m_intialTimer.restart();  
    m_shooterTimer.stop(); 
    if (Units.metersToInches(m_drivetrain.calculateTargetDistance()) < 80){
      m_closeShooting = true;
    }else{
      m_closeShooting = false;
    }
    System.out.println(m_closeShooting);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean m_shooterAtSetpoint = m_shooter.getAtShooterSetpoint();
    boolean m_shoulderAtSetpoint = m_arm.atShoulderGoal();
    boolean m_elbowAtSetpoint = m_arm.atElbowGoal();
    if (Units.metersToInches(m_drivetrain.getVelocity()) <= 40){
      
      if (m_closeShooting){
        m_shooter.setBotPower(1);
        m_shooter.setTopPower(1);
      }else{
        if (m_intialTimer.get() < 0.6 || m_shooterAtSetpoint){
          m_shooter.setBotPower(1);  
          m_shooter.setTopPower(1);
          m_shooter.setShooterSetpoint(m_shooter.interpolateSetpoint(Units.metersToInches(m_drivetrain.calculateTargetDistance())));
        
        }else{
          if (!m_firingStarted){
            m_shooter.enableShooter();
            m_firingStarted = true;
          }
          m_shooter.setShooterSetpoint(m_shooter.interpolateSetpoint(Units.metersToInches(m_drivetrain.calculateTargetDistance())));
        }
      }

      if (m_closeShooting){
        m_altShooterTimer.start();
        if (m_altShooterTimer.get() > 0.8 && m_elbowAtSetpoint && m_shoulderAtSetpoint){
          m_shooterTimer.start();
          // m_intake.setIndexerVelocity(4000);
          m_intake.setIndexerPower(1);
            // System.out.println("SHOT");
        }
      }else{
        m_altShooterTimer.reset();
      }

      m_arm.setShoulderGoalWithoutElbow(m_arm.interpolateShoulder(Units.metersToInches(m_drivetrain.calculateTargetDistance()) + Math.toRadians(m_arm.getShooterAngMod())));
      m_arm.setElbowGoalRelative(m_arm.interpolateShortElbow(Units.metersToInches(m_drivetrain.calculateTargetDistance())));

      if (m_shoulderAtSetpoint && m_elbowAtSetpoint && m_shooterAtSetpoint && m_drivetrain.getOnTarget()){
        // m_intake.setIndexerVelocity(4000);
        m_intake.setIndexerPower(1);
      }
      if (m_shooterAtSetpoint && m_shoulderAtSetpoint && m_drivetrain.getOnTarget() && m_elbowAtSetpoint){
        m_shooterTimer.start();

      }else{
        // m_shooterTimer.stop();
      }

      if (m_shooterTimer.get() > 0.3){
        m_shot = true;
      }
    }
    SmartDashboard.putNumber("ShooterTimer", m_shooterTimer.get());
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
