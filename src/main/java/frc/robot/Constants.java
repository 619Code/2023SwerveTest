package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class ModuleConstants {
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 1/21.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;


        // We may need to tune this for the PID turning
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        // we need to update this
        public static final double kTrackWidth = 0.5969; 
        public static final double kWheelBase = 0.5969;

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //front right
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //back left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); //backright

        //We need to update these motors
        // frontLeft Module
        public static final int kFrontLeftDriveMotorPort = 4;
        public static final int kFrontLeftTurningMotorPort = 3;
        public static final boolean kFrontLeftDriveMotorReversed = false;
        public static final boolean kFrontLeftTurningMotorReversed = true;
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 3;
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = true;
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = -106.5;

        // frontRight Module
        public static final int kFrontRightDriveMotorPort = 2;
        public static final int kFrontRightTurningMotorPort = 1;
        public static final boolean kFrontRightDriveMotorReversed = false;
        public static final boolean kFrontRightTurningMotorReversed = true;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = true;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = -128;

        // backLeft Module
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final boolean kBackLeftDriveMotorReversed = true;
        public static final boolean kBackLeftTurningMotorReversed = true;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 7;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = true;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = -3;

        // backRight Module
        public static final int kBackRightDriveMotorPort = 6;
        public static final int kBackRightTurningMotorPort = 5;
        public static final boolean kBackRightDriveMotorReversed = true;
        public static final boolean kBackRightTurningMotorReversed = true;
        public static final int kBackRightDriveAbsoluteEncoderPort = 5;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = true;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = -64;

        
        
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * Math.PI * 2;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0 * 3);

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
        
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        public static final double kDeadband = 0.05;
    }
}

 /**
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    //these numbers may be incorrect 
  }

    public static final int kDriverControllerPort = 1;

    public static final double kTrackWidth = Units.inchesToMeters(21);
        // Distance between right and left wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5);
        // Distance between front and back wheels

    
    //please update these with the accurate dimensions for robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
    new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
    new Translation2d(kWheelBase / 2, kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
    new Translation2d(-kWheelBase / 2, kTrackWidth / 2));
    


    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kPturning = 0.5;
    public static final double kMaxSpeed = 5;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 5;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 5;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

    public static final int kDriverYAxis = 1;
    public static final int kDriverXAxis = 0;
    public static final int kDriverRotAxis = 4;
    public static final int kDriverFieldOrientedButtonIdx = 1;
    public static final double kDeadband = 0.05;
    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = //
                kPhysicalMaxAngularSpeedRadiansPerSecond / 4;
}
*/
 