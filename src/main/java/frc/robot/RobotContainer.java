// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ElbowConstants;
import frc.robot.Constants.HookConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShoulderConstants;
import frc.robot.Constants.VariablesConstants;
import frc.robot.autos.*;
import frc.robot.subsystems.SUB_Arm;
import frc.robot.subsystems.SUB_BotShooter;
import frc.robot.subsystems.SUB_Drivetrain;
import frc.robot.subsystems.SUB_Elbow;
import frc.robot.subsystems.SUB_GlobalVariables;
import frc.robot.subsystems.SUB_Intake;
import frc.robot.subsystems.SUB_LED;
// import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Shooter;
import frc.robot.subsystems.SUB_Shoulder;
import frc.robot.subsystems.SUB_TopShooter;
import frc.robot.subsystems.SUB_Vision;
import frc.robot.subsystems.SUB_Drivetrain.TeleopPath;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SUB_Vision m_vision = new SUB_Vision();
  private final SUB_Drivetrain m_drivetrain = new SUB_Drivetrain(m_vision);
  // private final AUTO_Trajectories m_trajectories = new AUTO_Trajectories();
  private final SUB_Intake m_intake = new SUB_Intake();
  // private final SUB_Shooter m_shooter = new SUB_Shooter();
  private final SUB_BotShooter m_botShooter = new SUB_BotShooter();
  private final SUB_TopShooter m_topShooter = new SUB_TopShooter();
  private final SUB_Shooter m_shooter = new SUB_Shooter(m_botShooter, m_topShooter);
  private final SUB_Elbow m_elbow = new SUB_Elbow();
  private final SUB_Shoulder m_shoulder = new SUB_Shoulder();
  private final SUB_Arm m_arm = new SUB_Arm(m_elbow,m_shoulder);
  private final SUB_LED m_led = new SUB_LED();
  private final SUB_GlobalVariables m_variables = new SUB_GlobalVariables();
  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);
  //A non command Xbox controller so we can implement rumble
  XboxController m_driverXController = new XboxController(OIConstants.kDriverControllerPort);
  XboxController m_operatorXController = new XboxController(OIConstants.kOperatorControllerPort);
  
  private final BooleanSupplier AutoAim = () -> m_variables.getAutofire();
  private final BooleanSupplier HasItem = () -> m_variables.getHasItem();
  private final BooleanSupplier ReadyDrop = () -> m_variables.getReadyDrop();
  private final BooleanSupplier ContShoot = () -> m_variables.getContShooting();

  // private SendableChooser<Command> autoChooser; 
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
        // Build an auto chooser. This will use Commands.none() as the default option.
  
    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    
    // Configure the button bindings
    configureButtonBindings();
    //for shooting directly at the subwoofer
    NamedCommands.registerCommand("SubShot", 
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new CMD_setShooterSetpoint(m_shooter, 2500),
          // new CMD_ShooterOff(m_shooter),
          // new CMD_setShooterPower(m_shooter, 1),
          // new CMD_ShooterOverCheck(m_shooter),
          // new CMD_setShooterSetpoint(m_shooter, 2250),
          new CMD_ShooterOn(m_shooter)
        ),
        new SequentialCommandGroup(
          // new CMD_setShooterPower(m_shooter, 1),
          new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-45)),
          new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(21)),
          new CMD_ElbowCheck(m_arm, 2),
          new CMD_ShooterOverValue(m_shooter, 2300, 2300),
          new CMD_IndexerSetPower(m_intake, .7)
        )
    ));

    NamedCommands.registerCommand("PrepSubShot", 
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new CMD_setShooterSetpoint(m_shooter, 2500),
          new CMD_ShooterOn(m_shooter)
        ),
        new SequentialCommandGroup(
          // new CMD_setShooterPower(m_shooter, 1),
          new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-45)),
          new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(21))
        )
    ));

    NamedCommands.registerCommand("FireSubShot", 
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new CMD_ElbowCheck(m_arm, 2),
          new CMD_ShooterOverValue(m_shooter, 2300, 2300),
          new CMD_IndexerSetPower(m_intake, .7)
        )
    ));

    NamedCommands.registerCommand("ReadyShooterFirstRing", new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2150),
      new CMD_ShooterOn(m_shooter),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-45)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(15)),
      new CMD_ElbowCheck(m_arm, 2)
    ));

    NamedCommands.registerCommand("ReadyShooterMid", new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2600),
      new CMD_ShooterOn(m_shooter),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-30.6)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(15)),
      new CMD_ShoulderCheck(m_arm, 2)
    ));

    NamedCommands.registerCommand("ReadyShooter", new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2300),
      new CMD_ShooterOn(m_shooter),
      // m_intake.CMDsetIndexVelocity(2350),
      // new CMD_GroundIntakeSetPower(m_intake, .7),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-34.8)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(15)),
      new CMD_ElbowCheck(m_arm,2)
    ));

    NamedCommands.registerCommand("ShootyPosition1", new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2500),
      new CMD_ShooterOn(m_shooter),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-34.1)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(11)),
      new CMD_ElbowCheck(m_arm,2)
    ));

    NamedCommands.registerCommand("ShootyPosition2", new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2500),
      new CMD_ShooterOn(m_shooter),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-34.)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(11)),
      new CMD_ElbowCheck(m_arm,2)
    ));

    NamedCommands.registerCommand("ShootyPosition3", new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2500),
      new CMD_ShooterOn(m_shooter),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-34.1)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(11)),
      new CMD_ElbowCheck(m_arm,2)
    ));
    
    NamedCommands.registerCommand("ReadyShooterStart", new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2200),
      new CMD_ShooterOn(m_shooter),
      // m_intake.CMDsetIndexVelocity(2350),
      // new CMD_GroundIntakeSetPower(m_intake, .7),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-42.7)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(13)),
      new CMD_ElbowCheck(m_arm, 2)
    ));

    NamedCommands.registerCommand("Dunk", new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2200),
      new CMD_ShooterOn(m_shooter),
      // m_intake.CMDsetIndexVelocity(2350),
      // new CMD_GroundIntakeSetPower(m_intake, .7),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(0)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(56.9)),
      new CMD_ElbowCheck(m_arm, 2)
    ));

    NamedCommands.registerCommand("HoldShooter", new SequentialCommandGroup(
      new CMD_ShooterOff(m_shooter),
      m_intake.CMDsetIndexVelocity(0),
      new CMD_GroundIntakeSetPower(m_intake, 0),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-47)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(13))

    ));

    NamedCommands.registerCommand("Fire", new SequentialCommandGroup( 
      // new CMD_AutoShoulder(m_arm, m_drivetrain, m_intake, m_variables),
      m_intake.CMDsetIndexPower(2350),
      new CMD_GroundIntakeSetPower(m_intake, .7)
    ));

    NamedCommands.registerCommand("AdjustShoulder", new SequentialCommandGroup(
      m_variables.CMDsetAutoaim(true),
      new CMD_AutoShoulder(m_arm, m_drivetrain, m_intake, m_variables),
      m_intake.CMDsetIntakeVelocity(2350),
      new CMD_GroundIntakeSetPower(m_intake, .7)
    ));

    NamedCommands.registerCommand("Intake3", new SequentialCommandGroup(
      // m_intake.CMDsetIndexVelocity(2750),     
      m_intake.CMDsetIndexPower(.7),
       new CMD_GroundIntakeSetPower(m_intake, .4)
      // new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-47.5)),
      // new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(10))
      // new CMD_IndexerIndex(m_intake).withTimeout(3)
      // m_intake.CMDsetIndexPower(0)
    ));

    NamedCommands.registerCommand("PickUp", new SequentialCommandGroup(
      m_intake.CMDsetIndexVelocity(2650),     
       new CMD_GroundIntakeSetPower(m_intake, .5),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-47.5)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(13)),
      new CMD_IndexerIndex(m_intake).withTimeout(3)
      // m_intake.CMDsetIndexPower(0)
    ));

    NamedCommands.registerCommand("PickUp5", new SequentialCommandGroup(
      m_intake.CMDsetIndexVelocity(2650),     
       new CMD_GroundIntakeSetPower(m_intake, .5),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-47.5)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(13)),
      new CMD_IndexerIndex(m_intake).withTimeout(5)
    ));

    NamedCommands.registerCommand("ShooterCheck", new SequentialCommandGroup(
      new CMD_ShooterCheck(m_shooter).withTimeout(2)
    ));

    NamedCommands.registerCommand("ShoulderCheck", new SequentialCommandGroup(
      new CMD_ShoulderCheck(m_arm, 2)    
    ));

    NamedCommands.registerCommand("End", new SequentialCommandGroup(
      m_variables.CMDsetAutoaim(false)
    ));

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure default commands
    m_drivetrain.setDefaultCommand(new CMD_Drive(m_drivetrain, m_driverController, m_variables));
    m_intake.setDefaultCommand(new CMD_PickUpRumble(m_intake, m_driverXController));
    m_led.setDefaultCommand(new CMD_LedHandler(m_led, m_shooter, m_variables));
    // m_shooter.setDefaultCommand(new CMD_IdleShooter(m_shooter));
    
  
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
  
    // m_driverController.leftBumper().onTrue(new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-4.8)));
    // m_driverController.rightBumper().onTrue(new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-44)));

    m_driverController.leftBumper().onTrue(new CMD_FloorIntake(m_variables, m_arm, m_intake));
    m_driverController.rightBumper().onTrue(new CMD_ShootSpeaker(m_arm, m_shooter, m_intake, m_variables, m_drivetrain));

    m_driverController.y().onTrue(new CMD_StockFire(m_arm, m_drivetrain, m_intake, m_shooter, m_variables));
    // eject
    m_driverController.x().onTrue(new ParallelCommandGroup(
      
      new SequentialCommandGroup(
        // new CMD_setShooterSetpoint(m_shooter, 2200),
        new CMD_setBotSetpoint(m_shooter, 2500),
        new CMD_setTopSetpoint(m_shooter, 2000),
        new CMD_ShooterOn(m_shooter),
        new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-49)),
        new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(16)),
        new CMD_ElbowCheck(m_arm, 2),
        new CMD_ShoulderCheck(m_arm, 2),
        new WaitCommand(1),
        new CMD_IndexerSetPower(m_intake, .7)
      )
    ));
    
    m_driverController.b().onTrue(new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, -1000),
      m_intake.CMDsetIndexPower(-3000),
      new CMD_GroundIntakeSetVelocity(m_intake, -1000),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-45)),
      new CMD_ElbowSetPosition(m_arm, Math.toRadians(16))
    )); 

    m_driverController.back().onTrue(
      m_drivetrain.CMDzeroHeading()
    );

    m_driverController.pov(0).onTrue(new SequentialCommandGroup(
      // m_arm.CMDsetShoulderConstrainst(ShoulderConstants.kClimbConstraints),
      m_arm.CMDsetLHookPWM(HookConstants.LHookOpen),
      m_arm.CMDsetRHookPWM(HookConstants.RHookOpen),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(45)),//45
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(48)),
      new CMD_ShoulderCheck(m_arm, 2)
      // new CMD_setShooterTrap(m_shooter, 1500),
      // new CMD_ShooterOn(m_shooter)
      // new WaitCommand(1),
      // m_intake.setIndexVelocity(1000),
      // new WaitCommand(6),
      // new CMD_ShooterOff(m_shooter),
      // m_intake.setIndexVelocity(0)
    ));

    m_driverController.pov(90).onTrue(new SequentialCommandGroup(

      new CMD_setShooterTrap(m_shooter, 2000),
      new CMD_ShooterOn(m_shooter),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(17)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(128)),
      new CMD_ShoulderCheck(m_arm, 2),
      new WaitCommand(0.25),
      m_intake.CMDsetIndexVelocity( 2000),
      new WaitCommand(1),
      m_intake.CMDsetIndexVelocity(0),
      new CMD_ShooterOff(m_shooter),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(70)),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-5))
      
    ));

    m_driverController.pov(180).onTrue(new SequentialCommandGroup(
      m_arm.CMDsetShoulderConstraints(ShoulderConstants.kClimbConstraints),
      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-47)),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(50)),//50
      new CMD_ShoulderCheck(m_arm, 4.00),
      m_arm.CMDsetLHookPWM(HookConstants.LHookClose),
      m_arm.CMDsetRHookPWM(HookConstants.RHookClose),
      new WaitCommand(0.2),

      new CMD_ShoulderSetPosition(m_arm, Math.toRadians(-10)),
      new CMD_ShoulderCheck(m_arm, 4),
      new CMD_ElbowSetPositionRelative(m_arm, Math.toRadians(80)),//60
      new CMD_ElbowCheck(m_arm, 4)

    ));

    m_driverController.start().onTrue(
      new CMD_Home(m_arm, m_intake, m_shooter, m_variables)
    );
    
    m_driverController.rightTrigger(.5).onTrue(
      new CMD_placeBackAmp(m_arm, m_shooter, m_intake)
      // new PrintCommand("PP")
    );
    m_driverController.rightTrigger(.5).onFalse(new SequentialCommandGroup(
      new CMD_dropBackAmp(m_arm, m_shooter, m_intake),
      m_variables.CMDsetHasItem(false),
      new CMD_Home(m_arm, m_intake, m_shooter, m_variables)
      // new PrintCommand("Poopoo")
    ));

    m_driverController.leftTrigger().whileTrue(m_drivetrain.teleopPathfindTo(TeleopPath.AMP));

    m_operatorController.a().onTrue(new CMD_ManFire(m_arm, m_drivetrain, m_intake, m_shooter, m_variables));
    
    m_operatorController.y().onTrue(new CMD_CycleSyncLocation(m_variables));

    m_operatorController.b().onTrue(m_intake.CMDsetIndexVelocity(4000));
    
    m_operatorController.x().onTrue(new CMD_SyncOdometry(m_drivetrain, m_variables));
    
    m_operatorController.rightBumper().onTrue(new CMD_CycleOutputType(m_variables));

    m_operatorController.leftBumper().onTrue(new ConditionalCommand(
      m_variables.CMDsetContShooting(false)
      ,m_variables.CMDsetContShooting(true)
      ,ContShoot)
    );

    m_operatorController.rightTrigger(.5).onTrue(new SequentialCommandGroup(
      new CMD_setShooterSetpoint(m_shooter, 2050),
      new CMD_ShooterOn(m_shooter)
    ));

    m_operatorController.povUp().onTrue(new SequentialCommandGroup (
      m_arm.CMDsetLHookPWM( HookConstants.LHookOpen ),
      m_arm.CMDsetRHookPWM( HookConstants.RHookOpen )
    ));
    
    m_operatorController.povDown().onTrue(new SequentialCommandGroup (
      m_arm.CMDsetLHookPWM( HookConstants.LHookClose ),
      m_arm.CMDsetRHookPWM( HookConstants.RHookClose )
    ));
  }

  private int getIntakeType() {
    return m_variables.getIntakeType();
  }
  private int getOutputType() {
    return m_variables.getOutputType();
  }


  public final Command getOutputCommand =
  new SelectCommand<>(
    Map.ofEntries(
      Map.entry(VariablesConstants.kSpeakerOutput, new CMD_ShootSpeaker(m_arm, m_shooter, m_intake, m_variables, m_drivetrain)),
      Map.entry(VariablesConstants.kFrontAmpOutput, new SequentialCommandGroup (
        new CMD_placeFrontAmp(m_arm, m_shooter, m_intake),
        m_variables.CMDsetReadyDrop(true),
        m_variables.CMDsetHasItem(false) 
      )),
      Map.entry(VariablesConstants.kBackAmpOutput, new SequentialCommandGroup (
        new CMD_placeBackAmp(m_arm, m_shooter, m_intake),
        m_variables.CMDsetReadyDrop(true),  
        m_variables.CMDsetHasItem(false) 
      )),
      Map.entry(VariablesConstants.kTallOutput, new CMD_ShootSpeakerTall(m_arm, m_shooter, m_intake, m_variables, m_drivetrain)
      )
    ), 
    this::getOutputType
  );
  
  public final Command getDropCommand =
  new SelectCommand<>(
    Map.ofEntries(
      Map.entry(VariablesConstants.kSpeakerOutput, new CMD_ShootSpeaker(m_arm, m_shooter, m_intake, m_variables, m_drivetrain)),
      Map.entry(VariablesConstants.kFrontAmpOutput, new SequentialCommandGroup (
      new CMD_dropFrontAmp(m_arm, m_shooter, m_intake),
      m_variables.CMDsetReadyDrop(false)
      )),
      Map.entry(VariablesConstants.kBackAmpOutput, new SequentialCommandGroup (
      new CMD_dropBackAmp(m_arm, m_shooter, m_intake),
      m_variables.CMDsetReadyDrop(false)
      )),
      Map.entry(VariablesConstants.kTallOutput,  
      new CMD_ShootSpeakerTall(m_arm, m_shooter, m_intake, m_variables, m_drivetrain)
      )
    ), 
    this::getOutputType
  );

  // public final Command getDropCommand =
  // new SelectCommand<>(
  //   Map.ofEntries(
  //     Map.entry(VariablesConstants.kSpeakerOutput, new CMD_ShootSpeaker(m_arm, m_shooter, m_intake, m_variables, m_drivetrain)),
  //     Map.entry(VariablesConstants.kFrontAmpOutput, new SequentialCommandGroup (
  //     new CMD_dropFrontAmp(m_arm, m_shooter, m_intake),
  //     m_variables.CMDsetReadyDrop(false)
  //     )),
  //     Map.entry(VariablesConstants.kTallOutput,  
  //     new CMD_ShootSpeakerTall(m_arm, m_shooter, m_intake, m_variables, m_drivetrain)
  //     )
  //   ), 
  //   this::getOutputType
  // );
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return autoChooser.getSelected();
    // return new PathPlannerAuto("4SlamBlue");
    return new PathPlannerAuto("4SlamRed");
  }

  public Command get4SlamRed() {
    return new PathPlannerAuto("4SlamRed");
  }

  public Command get4SlamBlue() {
    return new PathPlannerAuto("4SlamBlue");
  }

  public Command get4ShootBlue() {
    return new PathPlannerAuto("4ShootBlue");
  }

  public Command get4ShootBlueSafeV2() {
    return new PathPlannerAuto("4ShootBlueSafe_V2");
  }

  public Command get4ShootRedV3() {
    return new PathPlannerAuto("4ShootRed_V3");
  }
  
  public Command get4ShootRed() {
    return new PathPlannerAuto("4ShootRed");
  }

  public Command get4ShootRedSafeV2() {
    return new PathPlannerAuto("4ShootRedSafe_V2");
  }
  
  public Command get5ShootBlue() {
    return new PathPlannerAuto("5ShootBlue");
  }

  public Command get5ShootBlueV3() {
    return new PathPlannerAuto("5ShootBlue_V3");
  }
  
  public Command get5ShootRed() {
    return new PathPlannerAuto("5ShootRed");
  }

  public Command get5ShootRedV3() {
    return new PathPlannerAuto("5ShootRed_V3");
  }
  
  public Command get3InnerBlue() {
    return new PathPlannerAuto("3InnerBlue");
  }
  
  public Command get3InnerRed() {
    return new PathPlannerAuto("3InnerRed");
  }

  public Command get3OuterBlue() {
    return new PathPlannerAuto("3OuterBlue");
  }
  
  public Command get3OuterRed() {
    return new PathPlannerAuto("3OuterRed");
  }
  
  //   // return new PathPlannerAuto("3OuterBlue");
  //   // return new PathPlannerAuto("BOX");
  
  // public Command getAutonomousCommand(){
  //   // return new AUTO_StraightTuning(m_trajectories);
  //   // return new AUTO_DistanceTunning(m_trajectories);
  //   return new AUTO_TurnTuning(m_trajectories);
  //   // return new AUTO_BoxTuning(m_trajectories);
  //   // return new PathPlannerAuto("StraightTuning");
  // }

  public void resetHeading(){
    m_drivetrain.zeroHeading();
    // m_drivetrain.zeroOdometry();
  }
  public void setOdometry(Pose2d p_pose){
    m_drivetrain.resetOdometry(p_pose);
  }

  public void subsystemInit(){
    m_arm.setLHookPWM(HookConstants.LHookOpen);
    m_arm.setRHookPWM(HookConstants.RHookOpen);
    m_arm.useShoulder();
    m_arm.useElbow();
    m_arm.resetElbow();
    m_arm.setShoulderGoalWithoutElbow(m_arm.getShoulderPosition());
    m_shooter.disableShooter();
    m_intake.setIndexerVelocity(0);
    m_intake.setIntakeVelocity(0);
    // m_drivetrain.setShooterTarget();
    
  }
  
  public void LED(){
    m_led.LEDShootingMode();
  }
  public void getShooterTarget(){
    m_drivetrain.setShooterTarget();
  }
  public Command startRumbleTimer(){
    return new CMD_OperatorRumble(m_operatorXController);
  }
}
