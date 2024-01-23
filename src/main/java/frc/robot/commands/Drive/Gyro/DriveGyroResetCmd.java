package frc.robot.commands.Drive.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Primary.SwerveSubsystem;

public class DriveGyroResetCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;

    // A lot of times, we need to reset the gyro, so we use this file to do so.

    /**
     * Resets the gyro
     * @param swerveSubsystem *Subsystem* SwerveSubsystem
     * @return *Void* resets the heading.
     */
    public DriveGyroResetCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveSubsystem.zeroHeading();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}