package frc.robot.commands.Drive.Gyro;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Primary.SwerveSubsystem;

public class DriveGyro180Cmd extends Command {

    private final SwerveSubsystem swerveSubsystem;

    // We need to "reset" the gyro at the end of auto, so we used this command for a period of time.

    /**
     * Gives the gyro heading a 180 degree offset
     * @param swerveSubsystem *Subsystem* SwerveSubsystem
     * @return *Void* sets the heading.
     */
    public DriveGyro180Cmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        swerveSubsystem.setHeading(180);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}