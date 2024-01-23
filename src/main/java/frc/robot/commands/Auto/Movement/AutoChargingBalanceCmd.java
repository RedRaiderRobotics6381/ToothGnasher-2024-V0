package frc.robot.commands.Auto.Movement;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.Primary.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoChargingBalanceCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;

    // This file balances on the charging station w/ PID.

    /**
     * Gets the joystick & controller values and sets the module states.
     * @param swerveSubsystem *Subsystem* SwerveSubsystem
     * @return *Void* Sets the swerve module states.
     */
    public AutoChargingBalanceCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        // 1. Creates a field oriented chassis speed
        ChassisSpeeds chassisSpeeds;
 
        // 1.5 This PID has a goal of 0 (first param) with an actual result of the gyro (second param), so it spits out a value to set the motor to
        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(SensorConstants.PIDcharging.calculate(0, -swerveSubsystem.getPitch()), 0, 0, swerveSubsystem.getRotation2d());

        // 2. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 3. Output each module states to wheels
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