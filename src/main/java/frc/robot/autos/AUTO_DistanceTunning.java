// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//This makes the robot go step by step instead of the full distance
public class AUTO_DistanceTunning extends SequentialCommandGroup {
  /** Creates a new CMD_DistanceTunning. */
  AUTO_Trajectories m_trajectories;
  public AUTO_DistanceTunning(AUTO_Trajectories p_trajectories) {
    m_trajectories = p_trajectories;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_trajectories.driveTrajectory(m_trajectories.forward1ftTrajectory),
      new WaitCommand(1),
      m_trajectories.driveTrajectory(m_trajectories.backwards1ftTrajectory),
      m_trajectories.driveTrajectory(m_trajectories.forward2ftTrajectory),
      new WaitCommand(1),
      m_trajectories.driveTrajectory(m_trajectories.backwards2ftTrajectory),
      m_trajectories.driveTrajectory(m_trajectories.forward4ftTrajectory),
      new WaitCommand(1),
      m_trajectories.driveTrajectory(m_trajectories.backwards4ftTrajectory),
      m_trajectories.driveTrajectory(m_trajectories.forward8ftTrajectory),
      new WaitCommand(1),
      m_trajectories.driveTrajectory(m_trajectories.backwards8ftTrajectory),
      m_trajectories.driveTrajectory(m_trajectories.forwardTrajectory),
      new WaitCommand(1),
      m_trajectories.driveTrajectory(m_trajectories.backwardsTrajectory)
    );
  }
}
