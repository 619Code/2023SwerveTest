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
import frc.robot.helpers.Crashboard;
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
        
        double xSpeed = Math.abs(controller.getLeftY()) > OIConstants.kDeadband ? controller.getLeftY() : 0.0;
        xSpeed = xSpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        
        double ySpeed = Math.abs(controller.getLeftX()) > OIConstants.kDeadband ? controller.getLeftX() : 0.0;        
        ySpeed = ySpeed * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;        
        
        double turningSpeed = Math.abs(controller.getRightX()) > OIConstants.kDeadband ? controller.getRightX() : 0.0; 
        turningSpeed = turningSpeed * DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond;
        Crashboard.toDashboard("kTeleDriveMaxAngularSpeedDegreesPerSecond", DriveConstants.kTeleDriveMaxAngularSpeedDegreesPerSecond, "navx");
        //turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;
        //turningSpeed = 0;
        System.out.println("xSpeed: " + xSpeed + " ySpeed: " + ySpeed + " turningSpeed: " + turningSpeed);

        double turningSpeedRadiansPerSecond = Rotation2d.fromDegrees(turningSpeed).getRadians();
        Rotation2d currentHeading = Rotation2d.fromDegrees(-swerveSubsystem.getHeading()); //inverted
        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeedRadiansPerSecond, currentHeading);
        swerveSubsystem.setModuleStates(DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds));
        Crashboard.toDashboard("turningSpeedRadiansPerSecond", turningSpeedRadiansPerSecond, "navx");
        Crashboard.toDashboard("currentHeading", currentHeading.getRadians(), "navx");
        //Crashboard.toDashboard("desired states", DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds)[0].toString(), "navx");
        // this.swerveSubsystem.setModuleStates(new SwerveModuleState[] {new SwerveModuleState(0.5, new Rotation2d(0.1)), new SwerveModuleState(0.5, new Rotation2d(0.1)), new SwerveModuleState(0.5, new Rotation2d(0.1)), new SwerveModuleState(.5, new Rotation2d( 0.1))});
        //for (int i = 0; i < this.swerveSubsystem.getModuleStates().length; i ++) {
           // System.out.print(this.swerveSubsystem.getModuleStates()[i] + ",  " );
        // }
        // System.out.print("\n");
        // 
        

    }

    
    public void getAngle()
    {
        double angle = Math.atan(controller.getRightY()/controller.getRightX());
        if(controller.getRightY() > 0)
        {
            return angle;
        }
        else{
            return angle + 90;
        }
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