// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_FireFromAmp extends Command {
  /** Creates a new CMD_Autofire. */
  SUB_Arm m_arm;
  SUB_Drivetrain m_drivetrain;
  SUB_Intake m_intake;
  SUB_Shooter m_shooter;
  SUB_GlobalVariables m_variable;
  boolean m_shot;
  Timer m_shooterTimer;
  Timer m_totalTimer;
  Timer m_intialTimer;
  boolean m_closeShooting;
  int m_CHECK;
  public CMD_FireFromAmp(SUB_Arm p_arm, SUB_Drivetrain p_drivetrain, SUB_Intake p_intake, SUB_Shooter p_shooter, SUB_GlobalVariables p_variables) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = p_arm;
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_shooter = p_shooter;
    m_variable = p_variables;
    m_shooterTimer = new Timer();
    m_totalTimer = new Timer(); 
    m_intialTimer = new Timer();
    m_closeShooting = false;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_shooter.disableShooter();
    m_CHECK = 0;

    m_drivetrain.setAutoAlignSetpoint();

    m_shooter.setShooterSetpoint(4000);
    m_shooter.enableShooter();

    double sh_sp = m_arm.interpolateShoulder(125);
    double el_sp = m_arm.interpolateShortElbow(125);
    m_arm.setShoulderGoalWithoutElbow(sh_sp);
    m_arm.setElbowGoalRelative(el_sp);

    m_shot = false;
    m_closeShooting = false;
    m_shooterTimer.restart();
    m_totalTimer.restart();
    m_intialTimer.restart();  
    m_intialTimer.start();
    m_shooterTimer.stop(); 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean m_shooterAtSetpoint = m_shooter.getOverShooterValue(3900, 3900);
    boolean m_shoulderAtSetpoint = m_arm.atShoulderGoal();
    boolean m_elbowAtSetpoint = m_arm.atElbowGoal();

    if (m_variable.getAutofire()){
      double rot = m_drivetrain.autoAlignTurn();

      m_drivetrain.drive(-0.0, 0, rot, false, false);
    }

    // if (Units.metersToInches(m_drivetrain.getVelocity()) <= 40){
    if (m_shooterAtSetpoint && m_shoulderAtSetpoint && m_drivetrain.getOnTarget() && m_elbowAtSetpoint){
        if (m_CHECK >= 5){
          m_shooterTimer.start();
          m_intake.setIndexerPower(.5);
        }
        m_CHECK +=1;
      }else{
        m_CHECK = 0;
        // m_shooterTimer.stop();
      }

    if (m_shooterTimer.get() > 0.3){
      m_shot = true;
    }

    }
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("Top" + m_shooter.getTopShooterVelocity());
      System.out.println("Bot" + m_shooter.getBotShooterVelocity());
      System.out.println("Shoulder" + Math.toDegrees(m_arm.getShoulderPosition()));
      System.out.println("Timer" + m_totalTimer.get());
      if (m_variable.getContShooting()){
        m_shooter.setShooterSetpoint(1250);
      }else{
        m_shooter.disableShooter();
      }

      m_intake.setIndexerVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shot;
  }
}
