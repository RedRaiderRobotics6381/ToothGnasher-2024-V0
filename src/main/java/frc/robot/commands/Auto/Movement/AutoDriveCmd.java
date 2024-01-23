package frc.robot.commands.Auto.Movement;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Primary.SwerveSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AutoDriveCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    double value;

    // This file is used in auto to get up the charging station.
    // I just wanted to be able to set the motors for a period of time w/ out using trajectories

    /**
     * Moves on the y-axis
     * @param swerveSubsystem *Subsystem* SwerveSubsystem
     * @param value *Double* Speed that the vyMetersPerSecond is set to
     * @return *Void* Sets the swerve module states.
     */
    public AutoDriveCmd(SwerveSubsystem swerveSubsystem, double value) {
        this.value = value;
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // 1. Creates a robot oriented chassis speed
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = new ChassisSpeeds(0, value, 0);

        // 1. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 2. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
    }

    @Override
    public void end(boolean interrupted) {
        // swerveSubsystem.setModuleStates(states);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}