package frc.robot.commands;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveCommand extends CommandBase {

    private final SwerveSubsystem swerveSubsystem;
    private final XboxController controller;
    private CANcoder FrontLeftCoder;
    private CANSparkMax FrontLeftTurnSpark;

    public SwerveCommand(SwerveSubsystem swerveSubsystem, XboxController controller) {
        this.swerveSubsystem = swerveSubsystem;
        this.controller = controller;
        addRequirements(swerveSubsystem);
        this.FrontLeftCoder = swerveSubsystem.frontLeft.getCANcoder();
        this.FrontLeftTurnSpark = swerveSubsystem.frontLeft.turningMotor;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {

        // :3

        //
        System.out.println("ANGLE = " + swerveSubsystem.frontLeft.getAbsoluteEncoderDeg() + ", SPEED = " + FrontLeftTurnSpark.get());

        // \:3

        
        //this.swerveSubsystem.getpo
        // double xSpeed = -controller.getLeftY() * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;

        // double ySpeed = controller.getLeftX() * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        // ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        
        // double turningSpeed = controller.getRightX() * DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond;
        // turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        
        // ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, Rotation2d.fromDegrees(swerveSubsystem.getHeading()));
        // swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
       // this.swerveSubsystem.setModuleStates(new SwerveModuleState[] {new SwerveModuleState(0.5, new Rotation2d(0.1)), new SwerveModuleState(0.5, new Rotation2d(0.1)), new SwerveModuleState(0.5, new Rotation2d(0.1)), new SwerveModuleState(.5, new Rotation2d( 0.1))});
        //for (int i = 0; i < this.swerveSubsystem.getModuleStates().length; i ++) {
           // System.out.print(this.swerveSubsystem.getModuleStates()[i] + ",  " );
       // }
       // System.out.print("\n");
       // 

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