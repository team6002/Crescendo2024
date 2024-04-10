// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  Timer m_totalTimer;
  Timer m_intialTimer;
  boolean m_closeShooting;
  Timer m_timeoutTimer;
  int m_CHECK;
  int m_ODOCHECK;
  public CMD_Autofire(SUB_Arm p_arm, SUB_Drivetrain p_drivetrain, SUB_Intake p_intake, SUB_Shooter p_shooter, SUB_GlobalVariables p_variables) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = p_arm;
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_shooter = p_shooter;
    m_variable = p_variables;
    m_shooterTimer = new Timer();
    m_totalTimer = new Timer(); 
    m_intialTimer = new Timer();
    m_timeoutTimer = new Timer();
    m_closeShooting = false;
    addRequirements(m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_shooter.disableShooter();
    m_CHECK = 0;
    m_ODOCHECK = 0;
    m_drivetrain.setAutoAlignSetpoint();

    m_shooter.setShooterSetpoint(4000);
    m_shooter.enableShooter();

    double sh_sp = m_arm.interpolateShoulder(Units.metersToInches(m_drivetrain.calculateTargetDistance()));
    double el_sp = m_arm.interpolateShortElbow(Units.metersToInches(m_drivetrain.calculateTargetDistance()));
    m_arm.setShoulderGoalWithoutElbow(sh_sp);
    m_arm.setElbowGoalRelative(el_sp);

    m_shot = false;
    m_closeShooting = false;
    m_shooterTimer.restart();
    m_totalTimer.restart();
    m_intialTimer.restart();  
    m_intialTimer.start();
    m_shooterTimer.stop(); 
    m_timeoutTimer.restart();
    m_timeoutTimer.start();
    
    // System.out.println("Shoulder" + Math.toDegrees(m_arm.getShoulderGoal()));
    // System.out.println("Distance" + Units.metersToInches(m_drivetrain.calculateTargetDistance()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean m_shooterAtSetpoint = m_shooter.getOverShooterValue(3900, 3900);
    boolean m_shoulderAtSetpoint = m_arm.atShoulderGoal();
    boolean m_elbowAtSetpoint = m_arm.atElbowGoal();

    // updates the arms incase we move to far from the original setpoint
    if (Math.abs(m_arm.getShoulderGoal() - m_arm.interpolateShoulder(Units.metersToInches(m_drivetrain.calculateTargetDistance()))) <= .75){
      double sh_sp = m_arm.interpolateShoulder(Units.metersToInches(m_drivetrain.calculateTargetDistance()));
      m_arm.setShoulderGoalWithoutElbow(sh_sp);
    }
    // if (Math.abs(m_arm.getElbowGoal() - m_arm.interpolateShortElbow(Units.metersToInches(m_drivetrain.calculateTargetDistance()))) <= 1){
    //   double el_sp = m_arm.interpolateElbow(Units.metersToInches(m_drivetrain.calculateTargetDistance()));
    //   m_arm.setElbowGoalRelative(el_sp);
    // }
    if (m_variable.getAutofire()){
      double rot = m_drivetrain.autoAlignTurn();

      m_drivetrain.drive(0.0, 0, rot, false, false);
    }

    // if (Units.metersToInches(m_drivetrain.getVelocity()) <= 40){
    if (m_drivetrain.getStableOdometry()){
      m_ODOCHECK ++;
    }else{
      m_ODOCHECK = 0;
    }

    if (m_shooterAtSetpoint && m_shoulderAtSetpoint && m_drivetrain.getOnTarget() && m_elbowAtSetpoint){
        if (m_CHECK >= 5 && m_ODOCHECK >=10){
          m_shooterTimer.start();
          m_intake.setIndexerPower(.5);
        }
        m_CHECK ++;
      }else{
        m_CHECK = 0;
        // m_shooterTimer.stop();
      }

    if (m_shooterTimer.get() > 0.3){
      m_shot = true;
    }

    if (m_timeoutTimer.get() > 3){
      m_shooterTimer.start();
      m_intake.setIndexerPower(.5);
    }

    }
  // }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      System.out.println("Top" + m_shooter.getTopShooterVelocity());
      System.out.println("Bot" + m_shooter.getBotShooterVelocity());
      System.out.println("Shoulder" + Math.toDegrees(m_arm.getShoulderPosition()));
      System.out.println("Distance" + Units.metersToInches(m_drivetrain.calculateTargetDistance()));
      System.out.println("Timer" + m_totalTimer.get());
      if (m_variable.getContShooting()){
        m_shooter.setShooterSetpoint(3000);
      }else{
        m_shooter.setShooterSetpoint(0);
      }

      m_intake.setIndexerVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_shot;
  }
}
