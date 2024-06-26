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
public class CMD_dropFrontAmp extends SequentialCommandGroup {
  /** Creates a new CMD_placeFrontAmp. */
  public CMD_dropFrontAmp(
    SUB_Arm p_arm,
    SUB_Shooter p_shooter,
    SUB_Intake p_intake
  ) {
    addRequirements(p_shooter);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    //   new CMD_setShooterSetpoint(p_shooter, 2000),
    //   new CMD_ShooterOn(p_shooter),
      // new CMD_ShooterCheck(p_shooter),
      // new WaitCommand(0.15),
      p_intake.CMDsetIndexVelocity(1500),
      new WaitCommand(0.75),
      new CMD_ShooterOff(p_shooter),
      p_intake.CMDsetIndexVelocity(0)
    );
  }
}
