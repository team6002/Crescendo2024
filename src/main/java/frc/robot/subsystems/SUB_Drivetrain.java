// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
// import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LocationConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.HardwareConstants;
import frc.utils.SwerveUtils;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class SUB_Drivetrain extends SubsystemBase {
  SUB_Vision m_vision;
  // Create MAXSwerveModules
  SwerveModule[] SwerveModules;
  private final SwerveModule m_frontLeft = new SwerveModule(
      HardwareConstants.kFrontLeftDrivingCanId,
      HardwareConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset,
      "FrontLeft"
      );

  private final SwerveModule m_frontRight = new SwerveModule(
      HardwareConstants.kFrontRightDrivingCanId,
      HardwareConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset,
      "FrontRight"
      );

  private final SwerveModule m_rearLeft = new SwerveModule(
      HardwareConstants.kRearLeftDrivingCanId,
      HardwareConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset,
      "BackLeft"
      );

  private final SwerveModule m_rearRight = new SwerveModule(
      HardwareConstants.kRearRightDrivingCanId,
      HardwareConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset,
      "BackRight"
      );
  
  private final SwerveDrivePoseEstimator m_odometry;
  
  private ChassisSpeeds targetChassisSpeeds = new ChassisSpeeds();

  private boolean onTarget = false;
  // The gyro sensor
  private final AHRS m_gyro = new AHRS(Port.kMXP);
  // Slew rate filter variables for controlling lateral acceleration
  private double m_currentRotation = 0.0;
  private double m_currentTranslationDir = 0.0;
  private double m_currentTranslationMag = 0.0;

  private SlewRateLimiter m_magLimiter = new SlewRateLimiter(DriveConstants.kMagnitudeSlewRate);
  private SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.kRotationalSlewRate);
  private double m_prevTime = WPIUtilJNI.now() * 1e-6;
  
  private double m_TargetAngle = 0;
  private double m_angleOffset = 0;

  private double m_prevXPos = 0;
  private double m_prevYPos = 0;

  //the trapzoid motion profile for the autoalign function
  private TrapezoidProfile m_AutoAlignTrapProfile;
  private TrapezoidProfile.Constraints m_AutoAlignConstraints;
  private ProfiledPIDController m_AutoAlignProfile;
  private SimpleMotorFeedforward m_AutoAlignFF;

  private Translation2d m_currentTarget = LocationConstants.SpeakerBlue;
  // Odometry class for tracking robot pose
  // SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
  //     DriveConstants.kDriveKinematics,
  //     Rotation2d.fromDegrees(getAngle()),
  //     getModulePositions()
      // );

  // Available paths in teleop.  Will select path based on alliance color.
  public enum TeleopPath {
    AMP,
    SOURCE
  }

  Field2d field;
  Field2d fieldEst;
  /** Creates a new DriveSubsystem. */

  public SUB_Drivetrain(SUB_Vision p_vision) {
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    var visionStdDevs = VecBuilder.fill(1, 1, 1);    
        m_odometry =
    new SwerveDrivePoseEstimator(
            DriveConstants.kDriveKinematics,
            Rotation2d.fromDegrees(getAngle()),
            getModulePositions(),
            new Pose2d(),
            stateStdDevs,
            visionStdDevs);

    SwerveModules = new SwerveModule[]{
      m_frontLeft,
      m_frontRight,
      m_rearLeft,
      m_rearRight
    };
    m_vision = p_vision;
    field = new Field2d();
    fieldEst = new Field2d();
    // m_ChassisSpeed = new ChassisSpeeds(0, 0, 0);
    // SmartDashboard.putNumber("SwerveP", m_SwerveP);
    // SmartDashboard.putNumber("SwerveI", m_SwerveI);
    // SmartDashboard.putNumber("SwerveD", m_SwerveD);
    // SmartDashboard.putNumber("SwerveFF", m_SwerveFF);
    // Configure AutoBuilder last
      AutoBuilder.configureHolonomic(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getChasisSpeed, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              this::driveAutoBuilder, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
              new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                      new PIDConstants(AutoConstants.kPXController, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(AutoConstants.kPThetaController, 0.0, 0.0), // Rotation PID constants
                      4, // Max module speed, in m/s
                      Units.inchesToMeters(11), // Drive base radius in meters. Distance from robot center to furthest module.
                      new ReplanningConfig() // Default path replanning config. See the API for the options here
              ),
              () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                  return false;
              },
              this // Reference to this subsystem to set requirements
      );
      m_AutoAlignConstraints = new TrapezoidProfile.Constraints(DriveConstants.kAutoAlignMaxVelo, DriveConstants.kAutoAlignMaxAccel);
      m_AutoAlignTrapProfile = new TrapezoidProfile(m_AutoAlignConstraints);
      m_AutoAlignProfile = new ProfiledPIDController(DriveConstants.kAutoAlignP, DriveConstants.kAutoAlignI, DriveConstants.kAutoAlignD, m_AutoAlignConstraints);
      m_AutoAlignFF = new SimpleMotorFeedforward(DriveConstants.kAutoAlignS, DriveConstants.kAutoAlignV, DriveConstants.kAutoAlignA);
  }

  
  // private double m_SwerveP = m_frontLeft.getSwerveP();
  // private double m_SwerveI = m_frontLeft.getSwerveI();
  // private double m_SwerveD = m_frontLeft.getSwerveD();
  // private double m_S,werveFF = m_frontLeft.getSwerveFF();

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    
    var visionEst = m_vision.getEstimatedGlobalPose();
    // SmartDashboard.putData("Field", field);
    // SmartDashboard.putData("FieldEst", fieldEst);
    field.setRobotPose(getPose());
    m_odometry.update(
        Rotation2d.fromDegrees(getAngle()),
        getModulePositions()
        );
    SmartDashboard.putNumber("X",Units.metersToInches(getPose().getX()));
    SmartDashboard.putNumber("Y", Units.metersToInches(getPose().getY()));
    SmartDashboard.putNumber("Angle", getAngle());
    // SmartDashboard.putNumber("Target Angle", angleToCurrentTarget().getDegrees());
    // SmartDashboard.putNumber("Velocity?", Units.metersToInches(getVelocity()));
    // SmartDashboard.putNumber("TargetXError", Units.metersToInches(calculateTargetXError()));
    SmartDashboard.putNumber("TargetDistance", Units.metersToInches(calculateTargetDistance()));
    // SmartDashboard.putNumber("XVelocity", getXVelocity());
    // SmartDashboard.putNumber("YVelocity", getYVelocity());
    // SmartDashboard.putBoolean("SeeTarget", visionEst.isPresent());
    // SmartDashboard.putNumber("TargetError", autoAlignTurn());
    SmartDashboard.putBoolean("OnTarget", onTarget);
    // m_frontLeft.telemetry(); 
    // m_frontRight.telemetry();
    // m_rearLeft.telemetry();
    // m_rearRight.telemetry();
    
    // m_frontLeft.showPID();
  
    // double m_SwerveP_ = SmartDashboard.getNumber("SwerveP", m_SwerveP);
    // double m_SwerveI_ = SmartDashboard.getNumber("SwerveI", m_SwerveI);
    // double m_SwerveD_ = SmartDashboard.getNumber("SwerveD", m_SwerveD);
    // double m_SwerveFF_ = SmartDashboard.getNumber("SwerveFF", m_SwerveFF);
    // // noice coding :D
    // if (m_SwerveP_ != m_SwerveP || m_SwerveI_ != m_SwerveI || m_SwerveD_ != m_SwerveD || m_SwerveFF_ != m_SwerveFF){
    //   m_SwerveP = m_SwerveP_;
    //   m_SwerveI = m_SwerveI_;
    //   m_SwerveD = m_SwerveD_;
    //   m_SwerveFF = m_SwerveFF_;  
    //   m_frontLeft.updatePID(m_SwerveP, m_SwerveI, m_SwerveD, m_SwerveFF);
    //   m_frontRight.updatePID(m_SwerveP, m_SwerveI, m_SwerveD, m_SwerveFF);
    //   m_rearLeft.updatePID(m_SwerveP, m_SwerveI, m_SwerveD, m_SwerveFF);
    //   m_rearRight.updatePID(m_SwerveP, m_SwerveI, m_SwerveD, m_SwerveFF);
    //   SmartDashboard.putNumber("SwerveP", m_SwerveP);
    //   SmartDashboard.putNumber("SwerveI", m_SwerveI);
    //   SmartDashboard.putNumber("SwerveD", m_SwerveD);
    //   SmartDashboard.putNumber("SwerveFF", m_SwerveFF);
    // }
    // else {
    //   // lol how long does it take for justin to notice this
    //   //saw it immedialtly lmao
    // }
    
    visionEst.ifPresent(
      est -> {
          var estPose = est.estimatedPose.toPose2d();
          // Change our trust in the measurement based on the tags we can see
          var estStdDevs = m_vision.getEstimationStdDevs(estPose);

          addVisionMeasurement(
                  est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
      }
      
    );
    
    if (visionEst.isPresent()){
      SmartDashboard.putNumber("targetYaw", m_vision.getTargetYaw());
      // SmartDashboard.putNumber("EstX", Units.metersToInches(visionEst.get().estimatedPose.getX()));
      // SmartDashboard.putNumber("EstY", Units.metersToInches(visionEst.get().estimatedPose.getY()));
      // SmartDashboard.putNumber("EstDeg", Math.toDegrees(visionEst.get().estimatedPose.getRotation().getAngle()));
      // SmartDashboard.putNumber("targetAng", m_vision.getTargetYaw(7));
      // fieldEst.setRobotPose(visionEst.get().estimatedPose.toPose2d());
    }
  }



  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[SwerveModules.length];
    for (int i = 0; i < SwerveModules.length; i++) {
      states[i] = SwerveModules[i].getState();
    }
    return states;
  }

  public ChassisSpeeds getChasisSpeed() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
  
  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    // return m_odometry.getPoseMeters();
    Pose2d p_decompPose = m_odometry.getEstimatedPosition();
    Pose2d p_Pose2d = new Pose2d(p_decompPose.getX(), p_decompPose.getY(), p_decompPose.getRotation());
    return p_Pose2d;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    setHeading(pose.getRotation().getDegrees());
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(getAngle()),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   * @param rateLimit     Whether to enable rate limiting for smoother control.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
    
    double xSpeedCommanded;
    double ySpeedCommanded;

    if (rateLimit) {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));

      // Calculate the direction slew rate based on an estimate of the lateral acceleration
      double directionSlewRate;
      if (m_currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.kDirectionSlewRate / m_currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //some high number that means the slew rate is effectively instantaneous
      }
      

      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, m_currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (m_currentTranslationMag > 1e-4) { //some small number to avoid floating-point errors with equality checking
          // keep currentTranslationDir unchanged
          m_currentTranslationMag = m_magLimiter.calculate(0.0);
        }
        else {
          m_currentTranslationDir = SwerveUtils.WrapAngle(m_currentTranslationDir + Math.PI);
          m_currentTranslationMag = m_magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        m_currentTranslationDir = SwerveUtils.StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        m_currentTranslationMag = m_magLimiter.calculate(0.0);
      }
      m_prevTime = currentTime;
      
      xSpeedCommanded = m_currentTranslationMag * Math.cos(m_currentTranslationDir);
      ySpeedCommanded = m_currentTranslationMag * Math.sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.calculate(rot);


    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      m_currentRotation = rot;
    }

    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = m_currentRotation * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    setModuleStates(swerveModuleStates);
  }
  
  public void driveAutoBuilder(ChassisSpeeds p_ChassisSpeed){
    // ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(p_ChassisSpeed, 0.02);
    
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(new ChassisSpeeds(p_ChassisSpeed.vxMetersPerSecond, p_ChassisSpeed.vyMetersPerSecond, -p_ChassisSpeed.omegaRadiansPerSecond), 0.02);
    SwerveModuleState[] targetStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(targetSpeeds);
    
    setModuleStates(targetStates);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Using the PathPlanner pathfinding algorithm, pathfind from our current position to a path. Used
   * in teleop to pathfind to the start of a known path location.  Requires AutoPathBuilder to be
   * configured before use.  
   * @param wanted_path Path we want to pathfind to.  Known location in TeleopPath.
   * @return Command to follow the path that it found.
   */
  public Command teleopPathfindTo(TeleopPath wanted_path){
    PathPlannerPath path;
    if (DriverStation.getAlliance().isPresent()){
      switch (wanted_path) {
        case AMP:
          if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            path = PathPlannerPath.fromPathFile("RedAmp");
          }
          else {
            path = PathPlannerPath.fromPathFile("BlueAmp");
          }
          break;
        case SOURCE:
          if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
            path = PathPlannerPath.fromPathFile("RedSource");
          }
          else {
            path = PathPlannerPath.fromPathFile("BlueSource");
          }
          break;
        
        default:
          // no valid path to select.  Do nothing
          return new InstantCommand();
      }
    }else {
      // Driver alliance not selected
      return new InstantCommand();
    }
     
    
    // Create the constraints to use while pathfinding. The constraints defined in the path will only be used for the path.
    PathConstraints constraints = new PathConstraints(
            3.0, 2.0,
            Units.degreesToRadians(360), Units.degreesToRadians(180));
    
    Command pathfindingCommand = AutoBuilder.pathfindThenFollowPath(
      path, 
      constraints,
      3.0 // Rotation delay in meters.  How far robot will travel before rotating.
      );
    return pathfindingCommand;
  }

  public double getXVelocity(){
    return getChasisSpeed().vxMetersPerSecond;
  }
  public double getYVelocity(){
    return getChasisSpeed().vyMetersPerSecond;
  }
  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. LOL*/
  public void zeroHeading() {
    // m_gyro.resetDisplacement();
    m_gyro.reset();
    m_angleOffset = 0;
  }

  public Command CMDzeroHeading() {
    return Commands.runOnce(()->zeroHeading(),this);
  }

  public void setHeading(double p_DegAngle){
    m_gyro.reset();
    m_angleOffset = p_DegAngle;
  }

  public void zeroOdometry(){
    resetOdometry(new Pose2d(0,0, Rotation2d.fromDegrees(0)));
    zeroHeading();
    // resetEncoders();
  }
  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  // public double getHeading() {
  //   return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  // }

  public double getVelocity(){
    double p_velocity =
    (m_frontLeft.getVelocity() + m_frontRight.getVelocity()
      + m_rearLeft.getVelocity() + m_rearRight.getVelocity())/4;
    return p_velocity;
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
  
  public void selectTarget(Translation2d p_target){
    m_currentTarget = p_target;
  }

  public void setShooterTarget(){
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      m_currentTarget = LocationConstants.SpeakerShootingRed;
    }else{
      m_currentTarget = LocationConstants.SpeakerShootingBlue;
    }
  }
  
  public double calculateTargetDistance(){
    return (getPose().getTranslation().getDistance(m_currentTarget));
  }

  public double calculateTargetXError(){
    return  Math.abs(getPose().getX() - Units.inchesToMeters(DriveConstants.kTrackWidth/2) - m_currentTarget.getX());
  }
  
  public double calculateTargetYError(){
    return  (getPose().getY()) - Units.inchesToMeters(DriveConstants.kTrackWidth/2) - m_currentTarget.getY();
  }
  /**
   * Calculate the angle between current pose to current target
   * @return Relative angle to the current target from robot pose
   */
  public Rotation2d angleToCurrentTarget() {
    Translation2d currentTranslation = getPose().getTranslation();
    // Translation2d yAdjustment = new Translation2d(0, calculateTargetYError() *0.1);
    Translation2d delta = currentTranslation.minus(m_currentTarget);
    Rotation2d angleTo = new Rotation2d(Math.atan2(delta.getY(), delta.getX()));
    return angleTo;
  }

  /*
   * Using the relative angle from angleToCurrentTarget(), determine the rotation speed for the 
   * robot to turn towards the target
   * @param targetAng: angle to the target
   * @return: rotation power for drivetrain from -0.5 to 0.5
   */
  public double autoAlignTurn(){
    double CameraError = 0;

    //Angle to target
    // Rotation2d yAdjustment = new Rotation2d(calculateTargetYError() *0.1);
    Rotation2d globalTargetAng = angleToCurrentTarget();
    // adjusts the target angle to better shoot from the side
    // Rotation2d adjustedTargetAng = new Rotation2d(globalTargetAng.getCos(), globalTargetAng.getSin() + (calculateTargetYError()*0.1));

    // Calculate difference between target angle and our current heading 
    Rotation2d wantedTurnAngle = getPose().getRotation().minus(globalTargetAng);
    
    //The difference between our current pose and target in radians
    double targetError = wantedTurnAngle.getRadians();
    // SmartDashboard.putNumber("targetError", targetError);
    
    //target angle as detect by the Camera
    var visionEst = m_vision.getEstimatedGlobalPose();
    if (visionEst.isPresent()){
      // CameraError = m_vision.getTargetYaw(0);
    }
    // double f = Math.copySign(DriveConstants.kAutoAlignF, targetError);

    // m_AutoAlignProfile.calculate(, null, m_AutoAlignConstraints)
    if (Math.abs(targetError) <= Math.toRadians(2) && Math.abs(CameraError) <= 1){
      return 0;
    }
    // variable tolerance for different distances
    if (targetError <= MathUtil.clamp((250 / calculateTargetXError()) - ( 0.1 * calculateTargetYError()), 2, 4)){
      onTarget = true;
    } else{
      onTarget = false;
    }
    // return MathUtil.clamp(p + f, -0.5, 0.5);
    // SmartDashboard.putNumber("TargetAngle", globalTargetAng.getDegrees());
    // m_AutoAlignTrapProfile.calculate(0.02, new TrapezoidProfile.State(Math.toRadians(getAngle()), Math.toRadians(getTurnRate())), new TrapezoidProfile.State(globalTargetAng.getRadians(), 0));
    return -m_AutoAlignProfile.calculate(Math.toRadians(getAngle()), globalTargetAng.getRadians());
  }

  public boolean getOnTarget(){
    return onTarget;
  }
  /*
   * Returns the angle of the robot in FRC coordinate system
   * @return heading of the robot in degrees
   */
  public double getAngle() {
    return Math.toDegrees(MathUtil.angleModulus(-Rotation2d.fromDegrees(m_gyro.getAngle()).getRadians())) + m_angleOffset;
  }

  /**
  * Get the SwerveModulePosition of each swerve module (position, angle). The returned array order
  * matches the kinematics module order.
  */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
      m_odometry.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
  public void addVisionMeasurement(
          Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
      //uses navx instead of camera vision.
      Pose2d p_angledPose = new Pose2d(visionMeasurement.getTranslation(), Rotation2d.fromDegrees(getAngle())); 
      m_odometry.addVisionMeasurement(p_angledPose, timestampSeconds, stdDevs);
  }

  
}
