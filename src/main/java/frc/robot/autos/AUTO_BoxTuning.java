// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//This makes the robot go straight down and back 160 inches.
public class AUTO_BoxTuning extends SequentialCommandGroup {
  /** Creates a new StraightTuning. */
  AUTO_Trajectories m_trajectories;
  public AUTO_BoxTuning(AUTO_Trajectories p_trajectories) {
    m_trajectories = p_trajectories;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //Runs back and forth 10 times
    addCommands(
      m_trajectories.driveTrajectory(m_trajectories.boxForwardTrajectory),
      new WaitCommand(1),
      m_trajectories.driveTrajectory(m_trajectories.boxLeftTrajectory),
      new WaitCommand(1),
      m_trajectories.driveTrajectory(m_trajectories.boxBackTrajectory),
      new WaitCommand(1),
      m_trajectories.driveTrajectory(m_trajectories.boxRightTrajectory),
      new WaitCommand(1)
      
          
    );
  }
}
