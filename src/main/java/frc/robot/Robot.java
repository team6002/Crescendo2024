// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.PrintCommand;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private final SendableChooser<Command> m_Chooser = new SendableChooser<Command>();
  private RobotContainer m_robotContainer;
  

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    // m_robotContainer.zeroOdometry();
    m_robotContainer.LED();
    m_robotContainer.resetHeading();
    m_robotContainer.subsystemInit();
    SmartDashboard.putData("AUTO", m_Chooser);
    
    // m_Chooser.setDefaultOption("4SlamRed", m_robotContainer.get4SlamRed());
    // m_Chooser.addOption("4SlamBlue", m_robotContainer.get4SlamBlue());

    // m_Chooser.addOption("4SlamBlue", m_robotContainer.get4SlamBlue());
    // m_Chooser.addOption("4SlamRed", m_robotContainer.get4SlamRed());
    // m_Chooser.addOption("4ShootRed", m_robotContainer.get4ShootRed());
    m_Chooser.addOption("4ShootBlue", m_robotContainer.get4ShootBlue());
    m_Chooser.addOption("4ShootBlueSafe_V2", m_robotContainer.get4ShootBlueSafeV2());
    m_Chooser.addOption("4ShootBlueSafeReverse", m_robotContainer.get4SafeBlueReverse());
    m_Chooser.addOption("4ShootBlueSafeUnder", m_robotContainer.get4SafeBlueUnder());

    m_Chooser.addOption("4ShootRed_V3", m_robotContainer.get4ShootRedV3());
    m_Chooser.addOption("4ShootRedSafe_V2", m_robotContainer.get4ShootRedSafeV2());
    m_Chooser.addOption("4ShootRedSafeReverse", m_robotContainer.get4SafeRedReverse());
    m_Chooser.addOption("4ShootRedSafeUnder", m_robotContainer.get4SafeRedUnder());

    // m_Chooser.addOption("5ShootBlue", m_robotContainer.get5ShootBlue());
    // m_Chooser.addOption("5ShootRed", m_robotContainer.get5ShootRed());
    m_Chooser.addOption("5ShootBlue_V3", m_robotContainer.get5ShootBlueV3());
    m_Chooser.addOption("5ShootRed_V3", m_robotContainer.get5ShootRedV3());

    m_Chooser.addOption("3InnerBlue", m_robotContainer.get3InnerBlue());
    m_Chooser.addOption("3InnerRed", m_robotContainer.get3InnerRed());

    m_Chooser.addOption("3OuterBlue", m_robotContainer.get3OuterBlue());
    m_Chooser.addOption("3OuterRed", m_robotContainer.get3OuterRed());

    m_Chooser.addOption("5ShootDangerousBlue", m_robotContainer.get5ShootBlueDangerous());
    m_Chooser.addOption("5ShootDangerousRed", m_robotContainer.get5ShootRedDangerous());
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

    m_autonomousCommand = 
    m_Chooser.getSelected();
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // m_robotContainer.zeroOdometry();
    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    // m_robotContainer.resetHeading();
    // m_robotContainer.zeroOdometry();
    m_robotContainer.subsystemInit();
    m_robotContainer.getShooterTarget();
    m_robotContainer.startRumbleTimer();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
