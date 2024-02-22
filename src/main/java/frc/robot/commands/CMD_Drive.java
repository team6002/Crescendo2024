package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_GlobalVariables;

public class CMD_Drive extends Command {

  private final SUB_Drivetrain m_drivetrain;
  private final CommandXboxController m_controller;
  private final SUB_GlobalVariables m_variables;

  double deadzone = 0.1;	//variable for amount of deadzone
  double y = 0;           //variable for forward/backward movement
  double x = 0;           //variable for side to side movement
  double rot = 0;        //variable for turning mo vement
  
  public CMD_Drive(SUB_Drivetrain p_drivetrain, CommandXboxController p_controller, SUB_GlobalVariables p_variable) {
    m_drivetrain = p_drivetrain;
    m_controller = p_controller;
    m_variables = p_variable; 
    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
  }
 
  @Override
  public void execute() {
    var ySpeed = MathUtil.applyDeadband(-m_controller.getLeftX(),deadzone);

    var xSpeed = MathUtil.applyDeadband(-m_controller.getLeftY(),deadzone);
    
    if (m_variables.getAutofire()){
      rot = m_drivetrain.autoAlignTurn(m_drivetrain.calculateTargetAngle());
    }else{
      rot = MathUtil.applyDeadband(m_controller.getRightX(),deadzone);
    }
    // System.out.println(m_drivetrain.autoAlignTurn(m_drivetrain.calculateTargetAngle()));
    m_drivetrain.drive( xSpeed, ySpeed, rot,true,true);
    // m_drivetrain.drive( .4, 0, 0.0, true);
  }

  // private static double modifyAxis(double value) {
  //   double modifedValue;
  //   // Deadband
  //   // value = deadband(value, 0.2);

  //   // Square the axis
  //   modifedValue = value * value;
  //   modifedValue = Math.copySign(value, value);

  //   return modifedValue;
  // }
  @Override
  public void end(boolean interrupted) {
      m_drivetrain.drive(0.0, 0.0, 0.0, true,false);
  }

}