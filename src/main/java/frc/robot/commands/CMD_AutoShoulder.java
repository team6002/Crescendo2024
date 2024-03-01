// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_AutoShoulder extends Command {
  /** Creates a new CMD_Autofire. */
  SUB_Arm m_arm;
  SUB_Drivetrain m_drivetrain;
  SUB_Intake m_intake;
  SUB_GlobalVariables m_variable;
  boolean m_shot;
  public CMD_AutoShoulder(SUB_Arm p_arm, SUB_Drivetrain p_drivetrain, SUB_Intake p_intake, SUB_GlobalVariables p_variables) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = p_arm;
    m_drivetrain = p_drivetrain;
    m_intake = p_intake;
    m_variable = p_variables;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.setShoulderGoalWithoutElbow(m_arm.interpolateShoulder(Units.metersToInches(m_drivetrain.calculateTargetDistance())));
    m_arm.setElbowGoalRelative(Math.toRadians(10));
    // m_shooter.setShooterSetpoint(m_shooter.interpolateSetpoint(Units.metersToInches(m_drivetrain.getPose().getX())));
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // System.out.println(m_variable.getAutoaim());
    return m_arm.atShoulderGoal();
  }
}
