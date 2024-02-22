// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;

public class CMD_setOdometry extends Command {
  /** Creates a new CMD_ResetOdometry. */
  SUB_Drivetrain m_drivetrain;
  Pose2d m_newPose;
  public CMD_setOdometry(SUB_Drivetrain p_drivetrain, Pose2d p_newPose) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    m_newPose = p_newPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drivetrain.resetOdometry(m_newPose);
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
