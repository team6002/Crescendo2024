// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_SourceIntake extends SequentialCommandGroup {
  /** Creates a new CMD_SourceIntake. */
  public CMD_SourceIntake(
    SUB_Arm p_arm,
    SUB_Shooter p_shooter,
    SUB_Intake p_intake
  ) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new CMD_ShoulderSetPosition(p_arm, Math.toRadians(-40)),
      // new CMD_ShoulderCheck(m_arm, Math.toRadians(0)),
      new WaitCommand(0.05),
      new CMD_ElbowSetPosition(p_arm, Math.toRadians(90)),
      new CMD_setShooterSetpoint(p_shooter, -600),
      new CMD_ShooterOn(p_shooter),
      new CMD_IndexerCheck(p_intake),
      new CMD_IndexerReady(p_intake),
      // new CMD_IndexerShelfIndex(m_intake),
      new CMD_ShooterOff(p_shooter)
    );
  }
}
