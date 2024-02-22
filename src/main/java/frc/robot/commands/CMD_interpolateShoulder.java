// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Drivetrain;


public class CMD_interpolateShoulder extends Command {
  /** Sets the shoulder setpoint based on the interpolated value*/
  SUB_Arm m_shoulder;
  SUB_Drivetrain m_drivetrain;
  public CMD_interpolateShoulder(SUB_Drivetrain p_drivetrain, SUB_Arm p_shoulder) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    m_shoulder = p_shoulder;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shoulder.setShoulderGoalWithoutElbow(m_shoulder.interpolateShoulder(Units.metersToInches(m_drivetrain.getPose().getX())));
    // System.out.println(Math.toDegrees(m_shoulder.interpolateShoulder(Units.metersToInches(m_drivetrain.getPose().getX()))));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
