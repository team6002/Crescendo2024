// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class HardwareConstants{
    // SPARK MAX CAN IDs 
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kFrontLeftTurningCanId = 1;
    public static final int kFrontRightDrivingCanId =  4;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kRearRightDrivingCanId = 8;
    public static final int kRearRightTurningCanId = 7;

    public static final int kShoulderMotorCANID = 9;
    public static final int kShoulderFollowerMotorCANID = 10;

    public static final int kIntakeMotorCANID = 11;
    public static final int kIntakeFollowerMotorCANID = 12;

    public static final int kIndexerMotorCANID = 13;

    public static final int kElbowMotorCANID = 14;

    public static final int kShooterBotMotorCANID = 22;
    public static final int kShooterBotFollowerCANID = 21;
    public static final int kShooterTopMotorCANID = 23;

    public static final int kHookLeft = 0;
    public static final int kHookRight = 1;

    public static final int kLEDStrip = 9;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.6;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 1.2; // radians per second
    public static final double kMagnitudeSlewRate = 1.8; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 2.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(18.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(25.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;   

    public static final boolean kGyroReversed = false;

    public static final double kAutoAlignMaxAccel = 1.25;
    public static final double kAutoAlignMaxVelo = 1;
    
    public static final double kAutoAlignP = 0.5;
    public static final double kAutoAlignI = 0.0;
    public static final double kAutoAlignD = 0.0;

    public static final double kAutoAlignS = 0.2;
    public static final double kAutoAlignV = 0.2;
    public static final double kAutoAlignA = 0.00;

    public static final double kAlignVelocityMod = 0.018;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 15;

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 20) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    // This accounts for stuff such as wheel wear
    public static final double kXFactor = .9714;  // if actual is smaller than odo go down  

    public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) * kXFactor; // meters
    public static final double kDrivingEncoderVelocityFactor = (((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0) * kXFactor; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.17;//0.004;
    public static final double kDrivingI = 0.0;
    public static final double kDrivingD = 0.0;
    public static final double kDrivingFF = 0.21;

    public static final double kDrivingA = .4;
    public static final double kDrivingS = 0.17;//0.2;
    public static final double kDrivingV = 2.28;
    
    // public static final double kDrivingA = 0.44218;
    // public static final double kDrivingS = 0.17491;
    // public static final double kDrivingV = 2.7538;

    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1.5;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps

    public static final double kPAcceleration = 0.005;

  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 2;//4.5
    public static final double kMaxAccelerationMetersPerSecondSquared = 2;//5
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    // public static final double kPXController = 1;
    // public static final double kPYController = 1;
    // public static final double kPThetaController = 1;

    public static final double kPXController = 4;//0.2;//0.5;// perfect distance test
    public static final double kPYController = 0;//0.5; //perfect distance test
    public static final double kPThetaController = 4;//1.5;//2; perfect distance test
    
    
    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class IntakeConstants{
    // public static final int kShelfIntakeMotorCANID = 12;

    public static final double kIntakeP = 0.00005;
    public static final double kIntakeI = 0;
    public static final double kIntakeD = 0.00001;
    public static final double kIntakeFF = 0.00022;

    public static final double kIntakeS = 0;
    public static final double kIntakeV = 0;


    public static final int kIntakeCurrentLimit = 40;

    public static final double kIntakeOff = 0;
    public static final double kIntakeForward = 0.5;
    public static final double kIntakeBackward = -0.5;
  }

  public static final class IndexerConstants{

    public static final int kIndexerCurrentLimit = 50;

    public static final double kIndexerP = 0.0002;
    public static final double kIndexerI = 0;
    public static final double kIndexerD = 0.00001;
    public static final double kIndexerFF = 0.0001;

    public static final double kIndexerS = 0;
    public static final double kIndexerV = 0;

    public static final double kIndexerOff = 0;
    public static final double kIndexerForward = 0.4;
    public static final double kIndexerBackward = -0.4;

    public static final double kIndexerShoot = .2;
  }

  public static final class ShooterConstants{

    public static final double kShooterEncoderResolution = 42;
    public static final int kShooterCurrentLimit = 50;

    public static final double kShooterMaxVelocity = 6000;
    public static final double kShooterMaxAcceleration = 6000;
    public static final double kShooterMaxOutput = 1;
    public static final double kShooterMinOutput = -1;

    // public static final double kShooterBotP = 0.01;//0.005;
    // public static final double kShooterBotP = 0.0079;//0.005;
    public static final double kShooterBotP = 0.0075;//0.0085;//0.005;
    public static final double kShooterBotI = 0.0;
    public static final double kShooterBotD = 0.0009;//0.0005
    public static final double kShooterBotFF = 0.0000;

    public static final double kShooterBotSVolts = 0.00002;
    public static final double kBotVVoltSecondsPerRotation = 0.0022;
    public static final double kShooterBotA = 0.1;

    public static final double kShooterTopP = 0.009;//0.0077;//0.005;
    public static final double kShooterTopI = 0.0;
    public static final double kShooterTopD = 0.0009;//0.0005
    public static final double kShooterTopFF = 0.0;

    public static final double kShooterTopSVolts = 0.00002;//0.7;
    public static final double kTopVVoltSecondsPerRotation = 0.0022;//0.0022;
    public static final double kShooterTopA = 0.1;

    public static final double kShooterFreeSpeed = 5675;
    public static final double kShooterTargetSpeed = -500;
    public static final double kShooterTolerance = 50;

    
    public static final double kShootVelocity = 4000;
    public static final double kShooterIntakeVelocity = -500;

    public static final double[][] kShooterArray = {
    //RPMS for shooting at the Speaker for ft 
      {84, 2250}, 
      {120, 2400}, 
      {180, 2600},
      {240, 3000},
      {300, 3250},
    };
    public static final double[][] kStockShooterArray = {
    //RPMS for shooting to store notes at the AMP
      {36, 1300}, 
      {84, 1400}, 
      {120, 1500}, 
      {180, 1600},
      {240, 1700},
      {300, 1800},
    };
  }

  public static final class ShoulderConstants{
    //240 : 1
    // 0.47699999809265137

    public static final int kShoulderCurrentLimit = 50;

    public static final double kPositionConversionFactor = Math.PI * 2; // radians
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60; // radians per second

    public static final double kShoulderP = 2.1;
    public static final double kShoulderI = 0;
    public static final double kShoulderD = 0;
    public static final double kShoulderFF = 0.0;


    public static final double kShoulderClimbP = 1.1;
    public static final double kShoulderClimbI = 0;
    public static final double kShoulderClimbD = 0;
    public static final double kShoulderClimbFF = 0.0;

    // public static final double kShoulderS = 0.001;
    // public static final double kShoulderV = 0.01;
    
    public static final double kSVolts = 0.1;
    public static final double kGVolts = 0.2394;//0.12
    public static final double kVVoltSecondPerRad = 0;
    public static final double kAVoltSecondSquaredPerRad = 0;

    // public static final double kShoulderMaxVelocity = 1000;
    // public static final double kShoulderMaxAcceleration = 1000;

    public static final double kMaxVelocityRadPerSecond = 6.1;
    public static final double kMaxAccelerationRadPerSecSquared = 8.1;

    public static final double kMaxClimbVelocityRadPerSecond = .6;
    public static final double kMaxClimbAccelerationRadPerSecSquared = 1;

    public static final double kShoulderHome = Math.toRadians(-45);

    public static final TrapezoidProfile.Constraints kClimbConstraints = new TrapezoidProfile.Constraints(ShoulderConstants.kMaxClimbVelocityRadPerSecond
        , ShoulderConstants.kMaxClimbAccelerationRadPerSecSquared);
    
    public static final TrapezoidProfile.Constraints kNormalConstaints = new TrapezoidProfile.Constraints(ShoulderConstants.kMaxVelocityRadPerSecond
        , ShoulderConstants.kMaxAccelerationRadPerSecSquared);
  

    public static final double[][] kShoulderArray = {
      {36, Math.toRadians(-45)},
      {50, Math.toRadians(-45)},  
      {84, Math.toRadians(-40.5)}, 
      {100, Math.toRadians(-39)}, 
      {120, Math.toRadians(-37.5)}, 
      {150, Math.toRadians(-31.3)},
      {180, Math.toRadians(-30.9)},
      {240, Math.toRadians(-26.0)},
      {300, Math.toRadians(-26.)},
    };

  }
  
  public static final class HookConstants{
    public static final double LHookClose = .8;
    public static final double LHookOpen = 0.4;

    public static final double RHookClose = 0.2;
    public static final double RHookOpen = 0.4;

  }

  public static final class ElbowConstants{
    
    public static final int kElbowCurrentLimit = 50;

    public static final double kPositionConversionFactor = (Math.PI * 2); 
    public static final double kVelocityConversionFactor = kPositionConversionFactor / 60;

    public static final double kElbowP = .85;
    public static final double kElbowI = 0;
    public static final double kElbowD = 0.0;
    public static final double kElbowFF = 0.0;
    
    public static final double kSVolts = 0;
    public static final double kGVolts = 0.21;
    public static final double kVVoltSecondPerRad = 0;
    public static final double kAVoltSecondSquaredPerRad = 0;

    // public static final double kElbowMaxVelocity = 1000;
    // public static final double kElbowMaxAcceleration = 1000;
    public static final double kMaxVelocityRadPerSecond = 6.1;
    public static final double kMaxAccelerationRadPerSecSquared = 6.1;

    public static final double kElbowHome = Math.toRadians(16); 
    //only for da front one
    public static final double kElbowAmp = Math.toRadians(23); 
    public static final double kElbowShelfIntake = Math.toRadians(95);
    public static final double kElbowGroundIntake = Math.toRadians(45);
    //intake position is 90 to ground

    public static final double[][] kElbowArray = {
    //Elbow Position for shooting at the Speaker for ft 
      {36, Math.toRadians(60)},
      {50, Math.toRadians(47)},  
      {84, Math.toRadians(43)}, 
      {100, Math.toRadians(41)}, 
      {120, Math.toRadians(38)}, 
      {180, Math.toRadians(34)},
      {240, Math.toRadians(23)},
      {300, Math.toRadians(15)},
    };

    public static final double[][] kElbowShortArray = {
    //Elbow Position for shooting at the Speaker for ft 
      {36, Math.toRadians(22)},
      {48, Math.toRadians(18)},
      {50, Math.toRadians(14)},  
      {84, Math.toRadians(16)}, 
      {100, Math.toRadians(16)},
      {120, Math.toRadians(16)},
    };
  }

  public static final class VariablesConstants{
    public static final int kgroundIntakeType = 0;
    public static final int kshelfIntakeType = 1;

    public static final int kSpeakerOutput = 0;
    public static final int kFrontAmpOutput = 1;
    public static final int kBackAmpOutput = 3;
    public static final int kTallOutput = 2;
  }

  public static final class LocationConstants{
    public static final Translation2d SpeakerBlue = new Translation2d(Units.inchesToMeters(0), Units.inchesToMeters(215));
    public static final Translation2d SpeakerRed = new Translation2d(16.54, Units.inchesToMeters(215));
  
    public static final Translation2d SpeakerShootingBlue = new Translation2d(Units.inchesToMeters(4), Units.inchesToMeters(218));
    public static final Translation2d SpeakerShootingRed = new Translation2d(16.54 - Units.inchesToMeters(4), Units.inchesToMeters(218));
    // the location in which we shoot at to stage it.
    public static final Pose2d StageBlue = new Pose2d( 0.9,6.5, Rotation2d.fromDegrees(-90));
    public static final Pose2d StageRed = new Pose2d( 15.8,6.5, Rotation2d.fromDegrees(-90));
    
    public static final Pose2d SubwooferBlue = new Pose2d( 1.2,5.4, Rotation2d.fromDegrees(0));
    public static final Pose2d AmpBlue = new Pose2d( 1.9,7.8, Rotation2d.fromDegrees(-90));
    public static final Pose2d LSourceBlue = new Pose2d( 15,.5, Rotation2d.fromDegrees(125));// the source closest to blue side
    
    public static final Pose2d SubwooferRed = new Pose2d( 15.4,5.4, Rotation2d.fromDegrees(-180));
    public static final Pose2d AmpRed = new Pose2d( 14.8,7.8, Rotation2d.fromDegrees(-90));
    public static final Pose2d LSourceRed = new Pose2d( 1.6,.5, Rotation2d.fromDegrees(60));// the source closest to blue side
    
  }

  public static final class VisionConstants{
           public static final String kPhoton = "ShooterCam";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(Units.inchesToMeters(10.75), 0, Units.inchesToMeters(21)), new Rotation3d(0, Math.toRadians(-10), 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFields.kDefaultField.loadAprilTagLayoutField();

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(3, 3, 6);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.06, 0.06, .12);
 
  }
}
