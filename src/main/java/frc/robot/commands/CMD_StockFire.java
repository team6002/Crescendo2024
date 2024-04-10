// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_StockFire extends Command {
  /** Creates a new StockFire. */
  SUB_Arm m_arm;
  SUB_Drivetrain m_drivetrain;
  SUB_Intake m_intake;
  SUB_Shooter m_shooter;
  CommandXboxController m_controller;
  SUB_GlobalVariables m_variable;
  Timer m_shooterTimer;
  double deadzone = 0.1;	//variable for amount of deadzone
  double y = 0;           //variable for forward/backward movement
  double x = 0;           //variable for side to side movement
  double rot = 0;        //variable for turning mo vement
  double sideMod = 1; // variable for which side is the robot on
  boolean m_autoSlew;
  
  public CMD_StockFire(SUB_Arm p_arm, SUB_Drivetrain p_drivetrain, SUB_Intake p_intake, SUB_Shooter p_shooter, SUB_GlobalVariables p_variables, CommandXboxController p_controller) {
    m_arm = p_arm;
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_shooter = p_shooter;
    m_variable = p_variables;
    m_controller = p_controller;
    m_shooterTimer = new Timer();
    addRequirements(m_shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_variable.setAutofire(true);
    m_drivetrain.setAmpTarget(); 
    m_shooter.setShooterSetpoint( m_shooter.stockInterpolateSetpoint(Units.metersToInches(m_drivetrain.calculateTargetDistance())));
    m_shooter.enableShooter();
    m_shooterTimer.restart();
    m_shooterTimer.start();
    m_arm.setShoulderGoalWithoutElbow(Math.toRadians(-45));
    m_arm.setElbowGoalRelative(Math.toRadians(10));
      if (DriverStation.getAlliance().get() == Alliance.Red){
      sideMod = -1;
    }else {
      sideMod = 1;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    var ySpeed = MathUtil.applyDeadband(-m_controller.getLeftX(),deadzone)*sideMod;

    var xSpeed = MathUtil.applyDeadband(-m_controller.getLeftY(),deadzone)*sideMod;

    // rot = MathUtil.applyDeadband(m_controller.getRightX(), deadzone);
    m_autoSlew = false;

    // System.out.println(m_drivetrain.autoAlignTurn(m_drivetrain.calculateTargetAngle()));
    m_drivetrain.drive( xSpeed, ySpeed, rot,true, m_autoSlew);
    if (m_shooterTimer.get() > 1 || m_shooter.getAtShooterSetpoint()){
      m_shooter.enableShooter();
      m_intake.setIndexerPower(1);
    }
    if (m_variable.getAutofire()){
      double rot = m_drivetrain.autoAlignTurn();

      m_drivetrain.drive(xSpeed, ySpeed, rot, true, false);
    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_variable.setAutofire(false);
    m_intake.setIndexerPower(0);
    // m_shooter.disableShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (m_shooterTimer.get() > 1.35);
  }
}
