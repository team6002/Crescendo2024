// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CMD_ShootSafeSubShotTeleop extends SequentialCommandGroup {
  /** Creates a new CMD_ShootSafeSubShotTeleop. */
  public CMD_ShootSafeSubShotTeleop(
    SUB_Arm p_arm,
    SUB_Shooter p_shooter,
    SUB_Intake p_intake
  ) {
    /* Shoot a safe sub shot by using preset values (shoulder, elbow, shooter)
     * 
     * This command is meant to be use during teleop when auto-targeting/visioning is not working.
    */
    addCommands(
      new ParallelCommandGroup(
        /* Start shooter */
        new SequentialCommandGroup(
          new CMD_setShooterSetpoint(p_shooter, 2500),
          new CMD_ShooterOn(p_shooter)
        ),

        /* Move elbow and shoulder to subshot positions. When shooter
         * is above a threshold, fire.
        */
        new SequentialCommandGroup(
          new CMD_ShoulderSetPosition(p_arm, Math.toRadians(-49)),
          new CMD_ElbowSetPositionRelative(p_arm, Math.toRadians(17)),
          // new CMD_ElbowCheck(p_arm, 2),

          /* Wait for shooter to be above threshold */
          new CMD_ShooterOverValue(p_shooter, 2300, 2300),

          /* Fire sequence */
          new CMD_IndexerSetPower(p_intake, .7),
          new WaitCommand(0.2),

          /* Turn off shooter + indexer */
          new CMD_IndexerSetPower(p_intake, 0),
          Commands.runOnce(()->p_shooter.disableShooter(), p_shooter)
        )
      )
    );
  }
}
