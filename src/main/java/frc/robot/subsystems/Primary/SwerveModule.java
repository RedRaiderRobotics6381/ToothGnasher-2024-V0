package frc.robot.subsystems.Primary;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    // private final CANEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    // private final AnalogInput absoluteEncoder;

    // private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        // this.absoluteEncoderReversed = absoluteEncoderReversed;

        //Important:
        // absoluteEncoder = new AnalogInput(absoluteEncoderId);
        absoluteEncoder = new CANcoder(absoluteEncoderId); 

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        // turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        // turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        // turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        //absoluteEncoder.setPositionToAbsolute();
        
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }

    public double getTurningPosition() { //returns in radians
        return Math.PI/180 * absoluteEncoder.getAbsolutePosition().refresh().getValueAsDouble() - absoluteEncoderOffsetRad;
        // return turningEncoder.getPosition();
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getDriveMotorVelocity(){
        return driveMotor.get();
    }

    public double getTurningVelocity() {
        return absoluteEncoder.getVelocity().refresh().getValueAsDouble();
        // return turningEncoder.getVelocity();
    }

    // public double getAbsoluteEncoderRad() { // Important:
    //     // double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
    //     // angle *= 2.0 * Math.PI;
    //     // angle -= absoluteEncoderOffsetRad;
    //     // return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    // }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        // turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModulePosition getState() {
        return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        // made change
        // state.angle = new Rotation2d(-state.angle.getRadians());

        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));

        // SmartDashboard.putString("Swerve Module [" + absoluteEncoder.getDeviceID() + "] going to state", state.toString());
        // SmartDashboard.putString("Turn Encoder [" + absoluteEncoder.getDeviceID() + "] current angle:",""+ getTurningPosition());
        // SmartDashboard.putString("Drive Encoder [" + absoluteEncoder.getDeviceID() + "] current speeed:",""+ getDriveVelocity());

        // SmartDashboard.putString("Swerve Module [" + absoluteEncoder.getDeviceID() + "] setting the drive motor to:",""+state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        // SmartDashboard.putString("Swerve Module [" + absoluteEncoder.getDeviceID() + "] setting the turn motor to:",""+ turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}