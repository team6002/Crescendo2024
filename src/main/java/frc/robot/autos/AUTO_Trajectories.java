// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SUB_Drivetrain;

import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class AUTO_Trajectories extends SubsystemBase {
  /** Creates a new Trajectories. */
  SUB_Drivetrain m_drivetrain;

  PIDController m_xPIDController;
  PIDController m_yPIDController;
  ProfiledPIDController m_thetaPIDController;

  HolonomicDriveController m_holonomicDriveController;

  public AUTO_Trajectories(SUB_Drivetrain p_drivetrain) {
    m_drivetrain = p_drivetrain;


    m_xPIDController = new PIDController(AutoConstants.kPXController, 0, 0);
    m_yPIDController = new PIDController(AutoConstants.kPYController, 0, 0);
    m_thetaPIDController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    m_thetaPIDController.enableContinuousInput(-Math.PI, Math.PI);

    m_holonomicDriveController = new HolonomicDriveController(m_xPIDController, m_yPIDController, m_thetaPIDController);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics)
        .setReversed(false);
    
        
    // Create config for trajectory
    TrajectoryConfig reverseConfig = new TrajectoryConfig(
      AutoConstants.kMaxSpeedMetersPerSecond,
      AutoConstants.kMaxAccelerationMetersPerSecondSquared)
      // Add kinematics to ensure max speed is actually obeyed
      .setKinematics(DriveConstants.kDriveKinematics)
      .setReversed(true);

    Trajectory forwardTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(160), 0, new Rotation2d(0)),
      config);
    
    Trajectory backwardsTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(160), 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
      reverseConfig);

    Trajectory forward1ftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
       List.of(),
      new Pose2d(Units.inchesToMeters(12), 0, new Rotation2d(0)),
      config);
    
    Trajectory forward2ftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(24), 0, new Rotation2d(0)),
      config);

    Trajectory forward4ftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(48), 0, new Rotation2d(0)),
      config);

    Trajectory forward8ftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(96), 0, new Rotation2d(0)),
      config);

    Trajectory backwards1ftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(12), 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
      reverseConfig);
  
    Trajectory backwards2ftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(24), 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
      reverseConfig);

    Trajectory backwards4ftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(48), 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
      reverseConfig);

    Trajectory backwards8ftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(96), 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
      reverseConfig);

    Trajectory forwardTurnTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(0, 0, new Rotation2d(0)),
      // List.of(new Translation2d(Units.inchesToMeters(48), 0)),
      // new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(48), Rotation2d.fromDegrees(0)),
      // config);
      List.of(),
      new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(0), Rotation2d.fromDegrees(90)),
      config);

    Trajectory backwardTurnTrajectory = TrajectoryGenerator.generateTrajectory(
      // new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(48), Rotation2d.fromDegrees(0)),
      // List.of(new Translation2d(Units.inchesToMeters(48), 0)),
      // new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
      // reverseConfig); 
      new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(0), Rotation2d.fromDegrees(96)),
      List.of(),
      new Pose2d(Units.inchesToMeters(0), 0, Rotation2d.fromDegrees(0)),
      reverseConfig); 
    
    Trajectory boxForwardTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(48), 0, new Rotation2d(0)),
      config);

    Trajectory boxLeftTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(48), 0, new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(48), new Rotation2d(0)),
      config);

    Trajectory boxBackTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(48), Units.inchesToMeters(48), new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(48), new Rotation2d(0)),
      reverseConfig);

    Trajectory boxRightTrajectory = TrajectoryGenerator.generateTrajectory(
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(48), new Rotation2d(0)),
      List.of(),
      new Pose2d(Units.inchesToMeters(0),Units.inchesToMeters(0), new Rotation2d(0)),
      config);
    
    Trajectory splineTrajectory = TrajectoryGenerator.generateTrajectory(
     
      new Pose2d(0, 0, new Rotation2d(0)),
      
      List.of(),
      
      new Pose2d(Units.inchesToMeters(20), Units.inchesToMeters(20), new Rotation2d(0)),
      config);

    Trajectory reverseSplineTrajectory = TrajectoryGenerator.generateTrajectory(
     
      new Pose2d(Units.inchesToMeters(20), Units.inchesToMeters(20), new Rotation2d(0)),
      
      List.of(),
      
      new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(0)),
      reverseConfig);

  public Command driveTrajectory(Trajectory p_trajectory) {

    
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        p_trajectory,
        m_drivetrain::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
        // Position controllers
        m_holonomicDriveController,
        m_drivetrain::setModuleStates,
        m_drivetrain);

    // Reset odometry to the starting pose of the trajectory.
    // m_drivetrain.resetOdometry(p_trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return swerveControllerCommand.andThen(() -> m_drivetrain.drive(0, 0, 0, true, false));
  }
}
