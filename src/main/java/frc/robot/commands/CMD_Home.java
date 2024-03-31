// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.ElbowConstants;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_Home extends SequentialCommandGroup {
  /** Creates a new CMD_Home. */
  public CMD_Home(SUB_Arm m_arm, SUB_Intake m_intake, SUB_Shooter m_shooter, SUB_GlobalVariables m_variables) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      m_variables.CMDsetReadyDrop(false),
      m_variables.CMDsetAutofire(false),
      m_arm.CMDsetShoulderConstraints(ShoulderConstants.kNormalConstaints),
      m_arm.CMDsetElbowConstraints(ElbowConstants.kNormalConstaints),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-49)),
      // new CMD_ShoulderCheck(m_arm, Math.toRadians(-45)),
      new CMD_ElbowSetPosition(m_arm, Math.toRadians(15)),
      new CMD_ShooterOff(m_shooter),
      m_intake.CMDsetIndexVelocity(0),
      new CMD_GroundIntakeSetPower(m_intake, 0)
    );
  }
}
