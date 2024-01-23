package frc.robot.commands.Auto;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoWaitCmd extends Command {

    int goal;
    Boolean finished;

    // This is a timer command for auto. We need to know how long to output a cube / cone, move the motors, et. ceters so we build one timer command we can use for all auto commands.

    /**
     * Pauses
     * @param goal *Integer* The milliseconds it pauses
     * @return *Void* Sets the swerve module states.
     */
    public AutoWaitCmd(int goal) {
        this.goal = goal;
    }

    @Override
    public void initialize() {
        finished = false;
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        try {
            Thread.sleep(goal);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        return true;
    }
}