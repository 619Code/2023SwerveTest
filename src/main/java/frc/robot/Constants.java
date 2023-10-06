package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class ModuleConstants {
        // these constants should be correct for the current 4ki module we are using 
        public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
        public static final double kDriveMotorGearRatio = 1/6.75;
        public static final double kTurningMotorGearRatio = 1/21.0;
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        public static final double kTurningEncoderRot2Deg = kTurningMotorGearRatio * 360;
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        public static final double kTurningEncoderRPM2DegPerSec = kTurningEncoderRot2Deg / 60;


        // We may need to tune this for the PID turning
        
        // we may need to tune it so this is not set in stone
        public static final double kPTurning = 0.5;
    }

    public static final class DriveConstants {

        // we need to update this // no longer needs to be updated: I measured from center of axle to center of axle
        //thse seem to be based off of the base dimensions
        public static final double kTrackWidth = Units.inchesToMeters(21); 
        public static final double kWheelBase = Units.inchesToMeters(21);

        //This should be relative to the center, but still check documentation about how the grid is set up for swerve kinetics
            //Ive changed it. It looked wrong and I fixed it based on the coordinate system at https://hhs-team670.github.io/MustangLib/frc/team670/robot/utils/math/Translation2d.html
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2), //front left
                new Translation2d(kWheelBase / 2, kTrackWidth / 2), //front right
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), //back left
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2)); //backright

        //We need to update these motors
        // frontLeft Module
        public static final int kFrontLeftDriveMotorPort = 57; //motors updated
        public static final int kFrontLeftTurningMotorPort = 56; //motors updated
        public static final boolean kFrontLeftDriveMotorReversed = false; //updated
        public static final boolean kFrontLeftTurningMotorReversed = true; //updated
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 30; //updated
        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false; //updated
        public static final double kFrontLeftDriveAbsoluteEncoderOffsetDeg = -360.0 * 0.134033; //updated, in degrees // they want this to be the negative of the reported values?

        //we need to updats these motors
        // frontRight Module
        public static final int kFrontRightDriveMotorPort = 51; //motors updated
        public static final int kFrontRightTurningMotorPort = 50; //motors updated
        public static final boolean kFrontRightDriveMotorReversed = true; //updated
        public static final boolean kFrontRightTurningMotorReversed = true; //updated
        public static final int kFrontRightDriveAbsoluteEncoderPort = 31; //updated
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false; //updated
        public static final double kFrontRightDriveAbsoluteEncoderOffsetDeg = -360.0 * 0.279297; //updated, in degrees

        //we need to update these motors
        // backLeft Module
        public static final int kBackLeftDriveMotorPort = 55; //motors updated
        public static final int kBackLeftTurningMotorPort = 54; //motors updated
        public static final boolean kBackLeftDriveMotorReversed = false; //updated
        public static final boolean kBackLeftTurningMotorReversed = true; //updated
        public static final int kBackLeftDriveAbsoluteEncoderPort = 33; //updated
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false; //updated
        public static final double kBackLeftDriveAbsoluteEncoderOffsetDeg = -360.0 * 0.029297; //updated, in degrees

        //we need to update these motors
        // backRight Module
        public static final int kBackRightDriveMotorPort = 52; //motors updated
        public static final int kBackRightTurningMotorPort = 53; //motors updated
        public static final boolean kBackRightDriveMotorReversed = true; //updated
        public static final boolean kBackRightTurningMotorReversed = true; //updated
        public static final int kBackRightDriveAbsoluteEncoderPort = 32; //updated
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false; //updated
        public static final double kBackRightDriveAbsoluteEncoderOffsetDeg = -360.0 * 0.607910; //updated, in degrees

        
        //NOTE: these are not used in actual code they are just used to define max based on physical contraints
        //If you want to ignore these then change the limit on the max speeds manually 
        //these seem to be mostly fine but we may need change some things
        // also we need to change the physical dimensions of our base if we are going to use this 
        public static final double kPhysicalMaxSpeedMetersPerSecond = 6380.0 / 60.0 * (ModuleConstants.kDriveMotorGearRatio) * ModuleConstants.kWheelDiameterMeters * 360;
        public static final double kPhysicalMaxAngularSpeedDegreesPerSecond = kPhysicalMaxSpeedMetersPerSecond / Math.hypot(DriveConstants.kTrackWidth / 2.0, DriveConstants.kWheelBase / 2.0 * 3);

        //These are the variables that determine the max speeds of our swerve drive
        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kTeleDriveMaxAngularSpeedDegreesPerSecond = kPhysicalMaxAngularSpeedDegreesPerSecond / 4;
        
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;
    }

    //these we can ignore for now since they are only for autonmous 
    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 4;
        public static final double kMaxAngularSpeedDegreesPerSecond = DriveConstants.kPhysicalMaxAngularSpeedDegreesPerSecond / 10;
        
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularAccelerationDegreesPerSecondSquared = 45.0;
        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedDegreesPerSecond,
                        kMaxAngularAccelerationDegreesPerSecondSquared);
    }

    //this is just to set a deadzone so we don't need to move the stick
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
 