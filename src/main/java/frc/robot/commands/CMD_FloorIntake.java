// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_FloorIntake extends SequentialCommandGroup {
  /** Creates a new CMD_GroundIntake. */
  public CMD_FloorIntake(SUB_GlobalVariables p_variables, SUB_Arm p_arm, SUB_Intake p_intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SequentialCommandGroup(
        p_variables.CMDsetAutofire(false),
        new CMD_ShoulderSetPosition(p_arm, Math.toRadians(-45)),
        new CMD_ShoulderCheck(p_arm, 2),
        new CMD_ElbowSetPositionRelative(p_arm, ElbowConstants.kElbowHome),
        // new CMD_GroundIntakeForwardCool(p_intake, 4000),
        new CMD_GroundIntakeSetPower(p_intake, .75),
        new CMD_IndexerIndex(p_intake),
        new WaitCommand(0.1),
        new CMD_PrepShot(p_intake),
        // new CMD_GroundIntakeSetVelocity(p_intake, 0),
        p_variables.CMDsetHasItem(true)
      )
    );
  }
}
