package frc.robot;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {

    public static final class ModuleConstants {
        
        //Setting the wheel diameter in meters (4"=.01016m)
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4); // Almost sure, physical measurement is hard to measure
       
        //Setting the gear ratio from drive motor to wheel
        public static final double kDriveMotorGearRatio = 6.75; // Sure
       
        //Setting the gear ratio for turning motor to wheel
        public static final double kTurningMotorGearRatio = 150.0 / 7.0;// Sure:
       
        //Calculating rotations to radians (gear ratio of the turning motor * 2 * Pi)
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI; // Sure

        //Calculating rotations to meters (gear ratio of the turning motor * Pi * wheel diameter in meters)
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; // Sure

        //Calculating rotations to meters/sec, basically taking the last calculation and dividing by 60
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60; // Sure

        //For every this many rotations of the motor, the wheel turns one full rotation
        public static final double kTurningEncoderRotPerRot = 1 / kTurningMotorGearRatio; // Sure

        //Calculating revolutions/minute to radians/sec ((2*Pi*(1/6.75))/60)
        public static final double kTurningEncoderRPM2RadPerSec = 2*Math.PI*kTurningEncoderRotPerRot/60; // Almost sure


        // Tune:
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        //Distance between right and left wheels
        public static final double kTrackWidth = -Units.inchesToMeters(21); // Sure

        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(21); // Sure
    
    
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));


        // Can ID's for the swerve drive motors
        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackLeftDriveMotorPort = 6;
        public static final int kBackRightDriveMotorPort = 5;

        // Can ID's for the swerve turning motors
        public static final int kFrontLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 1;
        public static final int kBackRightTurningMotorPort = 3;

        // Determines direction of the swerve turning motors
        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        // Determines direction of the swerve drive motors
        public static final boolean kFrontLeftDriveEncoderReversed = false;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = false;
        public static final boolean kBackRightDriveEncoderReversed = false;

        //Can ID's for the swerve encoders
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 10; // Sure
        public static final int kFrontRightDriveAbsoluteEncoderPort = 11; // Sure
        public static final int kBackLeftDriveAbsoluteEncoderPort = 9; // Sure
        public static final int kBackRightDriveAbsoluteEncoderPort = 12; // Sure

        // Almost Sure: Determines the direction of sensing for the swerve encoders
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;


        // Sure: Offset to basically make the drive wheels straight when the encoders read zero
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = .408 + Math.PI; //.408
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 2.1859; //2.1859
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 4.6617744;// 4.6617744
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 3.980677145; //3.980677145

        // Maybe check:
        // public static final double kPhysicalMaxSpeedMetersPerSecond = 5*(2.0/0.25); // max speed, roughly 0.6 full speed, was 5, changing this didn't change anything it seemed, maybe slowed down the turning motors.

        public static double kPhysicalMaxSpeedMetersPerSecond = 7;

        // Multiplying this by 2 divided the actual max speed the drive motors were going by 2.

        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;

        public static double kTeleDriveMaxAccelerationUnitsPerSecond = 3; // originally 3
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = //
                DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; // maybe change this
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 0;
        public static final double kPYController = 0;
        public static final double kPThetaController = 0;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kSecondaryDriverControllerPort = 1;

        public static final int kDriverYAxis = 1;
        public static final int kDriverXAxis = 0;
        public static final int kDriverRotAxis = 4;
        public static final int kDriverFieldOrientedButtonIdx = 1;

        public static final int kDriverSlowButton = 1;
        public static final int kDriverlbumper = 5;
        public static final int kDriverrbumper = 6;

        public static final double kDeadband = 0.05;
    }

    public static final class ArmConstants {
        public static final int kLeftSliderMotor = 13;
        public static final int kRightSliderMotor = 14;
        public static final int kArmRotateMotor = 16;
        
        public static final int kManipulatorWristMotor = 17;
        public static final int kManipulatorIntakeMotor = 18;

        // uncertain
        public static final int gArmSliderBottom = 1;
        public static final int gArmSliderTop = 41;
        public static final int gArmSliderLow = 22;
        public static final int gArmSliderHumanPlayer = 41;

        public static final double gArmOffset = 1;
        public static final double gRotateoffset = 0.25;

        public static final double gSliderSpeed = -0.6;
        public static final double gSliderDown = -0.4;
        public static final double gRotateSpeed = 0.85;
        public static final double gOutputSpeed = 0.10;
        public static final double gIntakeSpeed = 0.50;

        public static final double rotateoffset = 2.5;

        // manipulator rotations
        // human player
        public static final double posDoubleHuman = 100;
        // placing
        public static final double posPlace = 117;
        // up
        public static final double posDrive = 150;
        // intake
        public static final double posIntake = 291;
        // Single human player station
        public static final double posSingularHuman = 125; //Match 24, was at 130
        //Hybrid Node on front
        public static final double posHybrid = 70;
        
        public static final double posDoubleHumanGravity = 0.03;
        public static final double posPlaceGravity = 0.03;
        public static final double posDriveGravity = 0.01;
        public static final double posIntakeGravity = -0.04;
        public static final double posSingularHumanGravity = 0.02;
        public static final double posHybridGravity = 0.04;
        public static final double restriction1 = 70;
        public static final double restriction2 = 291;
        public static final double rotateSpeed = 1;

        public static boolean manipulatorOn = false;
        public static boolean manipulatorManual = false;
        public static int rightYPort = 5;
    }

    public static final class SensorConstants {
        public static PIDController PIDspeed = new PIDController(0.20, 0, 0);
        public static PIDController PIDside = new PIDController(0.06, 0, 0);
        public static PIDController PIDturn = new PIDController(0.005, 0, 0);
        public static PIDController PIDcharging = new PIDController(0.05, 0, 0);
    }

    public static CANSparkFlex outtake1 = new CANSparkFlex(20, MotorType.kBrushless);
    public static CANSparkFlex outtake2 = new CANSparkFlex(21, MotorType.kBrushless);
}