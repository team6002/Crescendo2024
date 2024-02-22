// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Shooter;

public class CMD_interpolateShooterSetpoint extends Command {
  /** Sets the shooter setpoint based on the interpolated value*/
  SUB_Shooter m_shooter;
  SUB_Drivetrain m_drivetrain;
  public CMD_interpolateShooterSetpoint(SUB_Drivetrain p_drivetrain, SUB_Shooter p_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = p_drivetrain;
    m_shooter = p_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.setShooterSetpoint(m_shooter.interpolateSetpoint(Units.metersToInches(m_drivetrain.getPose().getX())));
    // System.out.println(m_shooter.interpolateSetpoint(Units.metersToInches(m_drivetrain.getPose().getX())));
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
