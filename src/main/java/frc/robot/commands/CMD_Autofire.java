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

public class CMD_Autofire extends Command {
  /** Creates a new CMD_Autofire. */
  SUB_Arm m_arm;
  SUB_Drivetrain m_drivetrain;
  SUB_Intake m_intake;
  SUB_Shooter m_shooter;
  SUB_GlobalVariables m_variable;
  boolean m_shot;
  double m_shooterTimer;
  double m_firingTimer;
  public CMD_Autofire(SUB_Arm p_arm, SUB_Drivetrain p_drivetrain, SUB_Intake p_intake, SUB_Shooter p_shooter, SUB_GlobalVariables p_variables) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = p_arm;
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_shooter = p_shooter;
    m_variable = p_variables;
    m_firingTimer = 0;
    m_shooterTimer = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shot = false;
    m_firingTimer = 0;
    m_shooterTimer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Units.metersToInches(m_drivetrain.getVelocity()) <= 40){
      if (m_firingTimer == 0){
        m_shooter.enableShooter();
      }
      m_shooter.setShooterSetpoint(m_shooter.interpolateSetpoint(Units.metersToInches(m_drivetrain.calculateTargetXError())));
      m_firingTimer += 0.02;
      if (m_firingTimer >= .1){
        m_arm.setShoulderGoalWithoutElbow(m_arm.interpolateShoulder(Units.metersToInches(m_drivetrain.calculateTargetXError())));
        if (Units.metersToInches(m_drivetrain.calculateTargetXError()) <= 70 ){
          if (Units.metersToInches(m_drivetrain.calculateTargetXError()) <= 55){
            m_arm.setElbowGoalRelative(Math.toRadians(22));
          }else{
            m_arm.setElbowGoalRelative(Math.toRadians(18));
          }
        }else {
          m_arm.setElbowGoalAbsolute(Math.toRadians(ElbowConstants.kElbowHome)); 
        }
      }
      if (m_arm.atShoulderGoal() && m_shooter.getAtShooterSetpoint()){
        m_intake.setIndexerVelocity(4000);
      }
      if (m_shooter.getAtShooterSetpoint() && m_arm.atShoulderGoal()){
        m_shooterTimer += 0.02;
      }
      if (m_shooterTimer > 0.0){
        m_shooterTimer +=0.02;
        if (m_shooterTimer > 1){
          m_shot = true;
        }
      }
    }else {
      m_firingTimer = 0;
    }
    // if (m_shooter.getTopShooterCurrent() >= 20){
    //   m_shooterTimer += 1;
    //   if (m_shooterTimer >= 7){
    //     m_shot = true;
    //   }
    // }
   }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println("DONE");
    if (m_shot){
      m_shooter.disableShooter();
      m_intake.setIndexerVelocity(0);
    }
    return m_shot;
  }
}
