package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.SwerveCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class RobotContainer {

    private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();;
    private final XboxController driverOne = new XboxController(0);

    public RobotContainer() {
        swerveSubsystem.setDefaultCommand(new SwerveCommand(swerveSubsystem, driverOne));
        configureButtonBindings();
        // System.out.println(DriveConstants.kTeleDriveMaxSpeedMetersPerSecond);
    }

    private void configureButtonBindings() {
        
    }

    public SwerveSubsystem getSwerve() {
        return swerveSubsystem;
    }

    public Command getAutonomousCommand() {
        return null;
    }
}