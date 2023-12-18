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
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
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

        //testing
        Crashboard.toDashboard("updatedControllerAngle", getHeadingFromController(controller), "controllerAngle");
        Crashboard.toDashboard("x", controller.getRightX(), "controllerAngle");
        Crashboard.toDashboard("y", controller.getRightY(), "controllerAngle");
        // :3

        //
        //System.out.println("ANGLE = " + swerveSubsystem.frontLeft.getAbsoluteEncoderDeg() + ", SPEED = " + FrontLeftTurnSpark.get());

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

        
        Rotation2d currentHeading = Rotation2d.fromDegrees(swerveSubsystem.getHeading()); //inverted
        Rotation2d targetHeading = Rotation2d.fromRadians(getHeadingFromController(controller));
        double turningSpeedRadiansPerSecond;

        if (RobotContainer.ABSOLUTE_TURNING_MODE) {
            turningSpeedRadiansPerSecond = 0;
            double deadzoneRadians = 0.1;

            if (currentHeading.getDegrees() < targetHeading.getDegrees()) turningSpeedRadiansPerSecond = 0.2;

            if (currentHeading.getDegrees() > targetHeading.getDegrees()) turningSpeedRadiansPerSecond = -0.2;

            double offset = currentHeading.getDegrees() - targetHeading.getDegrees();

            if (Math.abs(offset) <= deadzoneRadians) {
                turningSpeedRadiansPerSecond = 0;
            }

        } else {
            turningSpeedRadiansPerSecond = Rotation2d.fromDegrees(turningSpeed).getRadians();
        }

        

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

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    private double getHeadingFromController(XboxController c) {

        double deadzone = 0.05;

        double x = c.getRightX();
        double y = -c.getRightY();

        if (Math.abs(x) < deadzone) {
            x = 0;
        }
        if (Math.abs(y) < deadzone) {
            y = 0;
        }

        if (x == 0 && y == 0) return Math.toRadians(swerveSubsystem.getHeading());

        
        int holder = 1;
        
        double angle = -1 * Math.atan(x/y);
        if (y < 0) {
            if (x < 0)
            {
                holder = -1;
            }
            angle -= holder * Math.PI;
      }
      return -angle;
    }
}