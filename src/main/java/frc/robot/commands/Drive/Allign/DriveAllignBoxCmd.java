package frc.robot.commands.Drive.Allign;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SensorConstants;
import frc.robot.subsystems.Primary.SwerveSubsystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class DriveAllignBoxCmd extends Command {

    private final SwerveSubsystem swerveSubsystem;
    static ChassisSpeeds chassisSpeeds;

    // We allign with the box using PID.

    /**
     * Alligns with the box
     * @param swerveSubsystem *Subsystem* SwerveSubsystem
     * @return *Void* Sets the module states.
     */
    @Deprecated
    public DriveAllignBoxCmd(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() { // turns on the limelight
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    @Override
    public void execute() { // if the limeight sees something, move
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
            swerveSubsystem.setModuleStates(move());
        }
    }

    @Override
    public void end(boolean interrupted) { // turns off the limelight
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    // Moves the robot based on the limelight position
    public static SwerveModuleState[] move(){
        // The first parameter is the number we want to reach, and the second is the value we are currently at. It spits out a number we set the motors to.
        double speed = SensorConstants.PIDspeed.calculate(3.5, getDistance(getVerticle()));
        double side = -SensorConstants.PIDside.calculate(0, getHorizontal());
        double turn = -SensorConstants.PIDturn.calculate(0, getHorizontal());

        System.out.println("Side : " + side); // 0.3
        System.out.println("Turn : " + turn); // 0.03

        if(Math.abs(side) < 0.3 && Math.abs(turn) < 0.03){
            chassisSpeeds = new ChassisSpeeds(speed * 2.5, side, turn);
        } else{
            chassisSpeeds = new ChassisSpeeds(0, side, turn);
        }

        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        return moduleStates;
    }

    // Gets the distace away from the box w/ trig
    public static double getDistance(double verticle) {
        double bottom = Math.tan(verticle * Math.PI / 180);
        double total = 2.83 / bottom; // originaly 3
        return total;
    }

    // Gets the verticle angle from the box w/ limelight
    public static double getVerticle(){
        double verticle1 = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double reading = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        if (reading == 1) {
            return verticle1 + 45; // mount angle
        } else {
            return 0;
        }
    }

    // Gets the horizontal angle w/ limeligt
    public static double getHorizontal(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    // Sets the chassis motors to 0
    public static ChassisSpeeds stop() {
        return new ChassisSpeeds(0, 0, 0);
    }
}