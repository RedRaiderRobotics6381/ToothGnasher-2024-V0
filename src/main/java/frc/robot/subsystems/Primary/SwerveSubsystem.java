package frc.robot.subsystems.Primary;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveSubsystem extends SubsystemBase {

    // Lists all of the components that make up the front left wheel. The components are all determined in Constants.java
    private final SwerveModule frontLeft = new SwerveModule(
            DriveConstants.kFrontLeftDriveMotorPort, // Can ID of the drive motor
            DriveConstants.kFrontLeftTurningMotorPort, // Can ID of the turning motor
            DriveConstants.kFrontLeftDriveEncoderReversed, // Direction of the drive motor
            DriveConstants.kFrontLeftTurningEncoderReversed, // Direction of the turning motor
            DriveConstants.kFrontLeftDriveAbsoluteEncoderPort, // Can ID of the encoder
            DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad, // Radian offset to make the wheel straight
            DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed); // Direction of the encoder

    // Lists all of the components that make up the front right wheel. The components are all determined in Constants/java
            private final SwerveModule frontRight = new SwerveModule(
            DriveConstants.kFrontRightDriveMotorPort, // Can ID of the drive motor
            DriveConstants.kFrontRightTurningMotorPort, // Can ID of the turning motor
            DriveConstants.kFrontRightDriveEncoderReversed, // Direction of the drive motor
            DriveConstants.kFrontRightTurningEncoderReversed, // Direction of the turning motor
            DriveConstants.kFrontRightDriveAbsoluteEncoderPort, // Can ID of the encoder
            DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad, // Raidain offset to the wheel straight
            DriveConstants.kFrontRightDriveAbsoluteEncoderReversed); // Direction of the encoder

    // Lists all of the components that make up the back left wheel. The components are all determined in Contants.java
    private final SwerveModule backLeft = new SwerveModule(
            DriveConstants.kBackLeftDriveMotorPort, // Can ID of the drive motor
            DriveConstants.kBackLeftTurningMotorPort, // Can ID of the turning motor
            DriveConstants.kBackLeftDriveEncoderReversed, // Direction of the drive motor
            DriveConstants.kBackLeftTurningEncoderReversed, // Direction of the turning motor
            DriveConstants.kBackLeftDriveAbsoluteEncoderPort, // Can ID of the encoder
            DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad, // Radian offset to make the wheel straight
            DriveConstants.kBackLeftDriveAbsoluteEncoderReversed); // Direction of the encoder

    // Lists all of the components that make up the back right wheel. The components are all determined in the Constants.java        
    private final SwerveModule backRight = new SwerveModule(
            DriveConstants.kBackRightDriveMotorPort, // Can ID of the drive motor
            DriveConstants.kBackRightTurningMotorPort, // Can ID of the turning motor
            DriveConstants.kBackRightDriveEncoderReversed, // Direction of the drive motor
            DriveConstants.kBackRightTurningEncoderReversed, // Direction of the turning motor
            DriveConstants.kBackRightDriveAbsoluteEncoderPort, // Can ID of the encoder
            DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad, // Radian offset to make the wheel straight
            DriveConstants.kBackRightDriveAbsoluteEncoderReversed); // Direction of the encoder

    // NAVX........
    private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    // private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
    //         new Rotation2d(0));
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(DriveConstants.kDriveKinematics,
            new Rotation2d(0), getSwerveModulePositions());

    public SwerveSubsystem() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }

    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return -Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public void setHeading(double value){
        gyro.setAngleAdjustment(value);
    }

    public double getPitch(){
        return gyro.getPitch();
    }

    public double getYaw(){
        return gyro.getYaw();
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), getSwerveModulePositions(), pose);
    }

    public SwerveModulePosition[] getSwerveModulePositions(){
        SwerveModulePosition[] position = {frontLeft.getState(), frontRight.getState(), backLeft.getState(),
            backRight.getState()};
        return position;
    }

    @Override
    public void periodic() {

        // odometer.update(getRotation2d(), frontLeft.getState(), frontRight.getState(), backLeft.getState(),
                // backRight.getState());
        odometer.update(getRotation2d(), getSwerveModulePositions());

        // SmartDashboard.putNumber("Robot Heading", getHeading());
        // SmartDashboard.putString("Robot Location", getPose().getTranslation().toString());
        
        SmartDashboard.putString("Special Turn Encoder [" + 9 + "] current angle:",""+frontLeft.getTurningPosition());

        SmartDashboard.putString("Special Turn Encoder [" + 10 + "] current angle:",""+frontRight.getTurningPosition());

        SmartDashboard.putString("Special Turn Encoder [" + 12 + "] current angle:",""+backLeft.getTurningPosition());

        SmartDashboard.putString("Special Turn Encoder [" + 11 + "] current angle:",""+backRight.getTurningPosition());

        SmartDashboard.putString("Wheel [frontLeft] ", " " + frontLeft.getDriveMotorVelocity());

        // SmartDashboard.putString("Gyro Roll:", "" + gyro.getRoll());
        // SmartDashboard.putString("Gyro Pitch:", "" + gyro.getPitch());
        // SmartDashboard.putString("Gyro Yaw:", "" + gyro.getYaw());

        // SmartDashboard.putNumber("Max Drive Speed: ", DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // SmartDashboard.putNumber("Max Turn Speed: ", DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond);
    }

    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        System.out.println(DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // SmartDashboard.putNumber("Max Speed: ", DriveConstants.kPhysicalMaxSpeedMetersPerSecond);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }
}