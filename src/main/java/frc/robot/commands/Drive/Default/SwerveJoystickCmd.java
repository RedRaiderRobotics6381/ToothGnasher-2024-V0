package frc.robot.commands.Drive.Default;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Primary.SwerveSubsystem;

public class SwerveJoystickCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
    private final Supplier<Boolean> lbumper, rbumper, slowbutton;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    public double turningChange;
    public double driveChange;

    // This file is the default command that drives the entire drivetrain. It takes joystick values and sets the swerve module states.

    /**
     * Gets the joystick & controller values and sets the module states.
     * @param swerveSubsystem *Subsystem* SwerveSubsystem
     * @param xSpdFunction *Joystick Port* x value from the controllers
     * @param ySpdFunction *Joystick Port* y value from the controllers
     * @param turningSpdFunction *Joystick Port* turning value from the controllers
     * @param lbumper *Button Port* left bumper that increases speed
     * @param rbumper *Button Port* right bumper that increases speed
     * @param slowbutton *Button Port* button that descreases speed
     * @return *Void* Sets the swerve module states.
     */
    public SwerveJoystickCmd(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xSpdFunction, Supplier<Double> ySpdFunction, Supplier<Double> turningSpdFunction, 
            Supplier<Boolean> lbumper, Supplier<Boolean> rbumper, Supplier<Boolean> slowbutton) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpdFunction = xSpdFunction;
        this.ySpdFunction = ySpdFunction;
        this.turningSpdFunction = turningSpdFunction;
        this.lbumper = lbumper;
        this.rbumper = rbumper;
        this.slowbutton = slowbutton;
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // dC = 3
        // tC = 2

        // dC = 2
        // tC = 2
        
        // 1. Spencer buttons
        if(lbumper.get() && rbumper.get()){
            driveChange = 3;
            turningChange = 4;
        } else if(lbumper.get() || rbumper.get()){
            driveChange = 2;
            turningChange = 3;
        } else if(slowbutton.get()){
            driveChange = 0.5;
            turningChange = 0.5;
        } else{
            driveChange = 1;
            turningChange = 1;
        }

        // 2. Get real-time joystick inputs
        double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 3. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 4. Make the driving smoother w/ a limiter
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 5. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (!slowbutton.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed * driveChange, ySpeed * driveChange, turningSpeed * turningChange, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 6. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 7. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}