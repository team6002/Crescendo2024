
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.SUB_Intake;

// public class CMD_ShelfIntakeForwardCool extends Command {
//   SUB_Intake m_intake;
//   boolean m_detected;
//   boolean m_finished;
//   double m_power;
//   /** Creates a new CMD_IntakeForward. */
//   public CMD_ShelfIntakeForwardCool(SUB_Intake p_intake, double p_power) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_intake = p_intake;
//     m_detected = false;
//     m_power = p_power;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     m_detected = false;
//     m_finished = false;
//     m_intake.setShelfIntakePower(m_power);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (m_intake.getShelfIntakeCurrent() >= 30 && m_detected == false){
//       m_detected = true;
//     } else {
//       m_detected = false;
//     }
//     if (m_detected == true && m_intake.getShelfIntakeCurrent() <= 10){
//       m_finished = true;
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return m_finished;
//   }
// }

