// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

//This makes the robot go straight down and back 160 inches.
public class AUTO_TurnTuning extends SequentialCommandGroup {
  /** Creates a new StraightTuning. */
  AUTO_Trajectories m_trajectories;
  public AUTO_TurnTuning(AUTO_Trajectories p_trajectories) {
    m_trajectories = p_trajectories;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    //Runs back and forth 10 times
    addCommands(
      m_trajectories.driveTrajectory(m_trajectories.forwardTurnTrajectory),
      new WaitCommand(2),
      m_trajectories.driveTrajectory(m_trajectories.backwardTurnTrajectory)
      
          
    );
  }
}
