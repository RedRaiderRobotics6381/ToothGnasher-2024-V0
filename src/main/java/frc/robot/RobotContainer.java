package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.Auto.Movement.AutoChargingBalanceCmd;
import frc.robot.commands.Drive.Allign.DriveAllignPoleCmd;
import frc.robot.commands.Drive.Default.SwerveJoystickCmd;
import frc.robot.commands.Drive.Gyro.DriveGyroResetCmd;
import frc.robot.subsystems.Primary.SwerveSubsystem;


public class RobotContainer {




        public final static SwerveSubsystem swerveSubsystem = new SwerveSubsystem();

        public final XboxController driverJoytick = new XboxController(OIConstants.kDriverControllerPort);
        public final static XboxController secondaryJoystick = new XboxController(
                        OIConstants.kSecondaryDriverControllerPort);

        public RobotContainer() {

                swerveSubsystem.setDefaultCommand(new SwerveJoystickCmd(
                                swerveSubsystem,
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
                                () -> -driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
                                () -> driverJoytick.getRawButton(OIConstants.kDriverlbumper),
                                () -> driverJoytick.getRawButton(OIConstants.kDriverrbumper),
                                () -> driverJoytick.getRawButton(OIConstants.kDriverSlowButton)));

                configureButtonBindings();
        }

        private void configureButtonBindings() {

                // Secondary

                // Primary

                new JoystickButton(driverJoytick, 3)
                                .whileTrue(new DriveAllignPoleCmd(swerveSubsystem));

                new JoystickButton(driverJoytick, 4).onTrue(new DriveGyroResetCmd(swerveSubsystem));

                new JoystickButton(driverJoytick, 2).whileTrue(new AutoChargingBalanceCmd(swerveSubsystem));

        }

        public Command getAutonomousCommand() {
                return null;
        }
}