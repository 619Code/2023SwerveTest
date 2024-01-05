package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.helpers.Crashboard;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder; //BAD CRINGE
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenixpro.configs.CANcoderConfiguration;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenixpro.signals.SensorDirectionValue;

public class SwerveModule {
    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    //private final RelativeEncoder turningEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(DriveConstants.Ks, DriveConstants.Kv);

    private final PIDController drivePIDController = new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD);

    private final PIDController turningPidController;
    private final PIDController drivePidController; 


    private final CANcoder absoluteEncoder;

    private final String ModuleName;

    public SwerveModule(String ModuleName, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, SensorDirectionValue positiveDirection) {

        this.ModuleName = ModuleName;
        absoluteEncoder = new CANcoder(absoluteEncoderId);
        CANcoderConfiguration canCoderConfiguration = new CANcoderConfiguration();
        canCoderConfiguration.MagnetSensor.MagnetOffset = absoluteEncoderOffset;
        canCoderConfiguration.MagnetSensor.SensorDirection = positiveDirection;
        canCoderConfiguration.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
        //absoluteEncoder.configAllSettings(canCoderConfiguration);
        absoluteEncoder.getConfigurator().apply(canCoderConfiguration);

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        configureMotor(driveMotor, driveMotorReversed);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        configureMotor(turningMotor, turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        //turningEncoder = turningMotor.getEncoder();

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Deg);
        //turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2DegPerSec);
 
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-180, 180);

        this.drivePidController = new PIDController(ModuleConstants.kPModuleDriveController, 0, ModuleConstants.kDModuleDriveController);

        resetEncoders();
    }

    private void configureMotor(CANSparkMax motor, Boolean inverted) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kCoast);
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(30);
        motor.burnFlash();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity() * ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    public double getAbsoluteEncoderDeg() {
        return (absoluteEncoder.getAbsolutePosition().getValue() * 360);// - absoluteEncoderOffset;

        //should be in degrees???
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        //turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public void setDesiredState(SwerveModuleState state) {

        state.speedMetersPerSecond *= state.angle.minus(Rotation2d.fromDegrees(getAbsoluteEncoderDeg())).getCos();

        Crashboard.toDashboard("driveVelocity", getDriveVelocity(), this.ModuleName + " Swerve");
        Crashboard.toDashboard("driveposition", this.driveEncoder.getPosition(), this.ModuleName + " Swerve");
        
        double driveOutput = drivePIDController.calculate(driveEncoder.getVelocity(), state.speedMetersPerSecond); // Convert to m/s
        //Crashboard.toDashboard(this.ModuleName + " driveSpeed", driveSpeed, "Swerve");
        
        final double driveFeedForwardOutput = driveFeedForward.calculate(state.speedMetersPerSecond);
        
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        
        // Calculate the drive output from the drive PID controller. ;}
        double driveSpeed = MathUtil.clamp(state.speedMetersPerSecond  / Constants.DriveConstants.kTeleDriveMaxSpeedMetersPerSecond/* / 360*/, -.5 ,.5);
  
        //driveMotor.set(driveSpeed);
        Crashboard.toDashboard(this.ModuleName + " driveSpeed", driveSpeed, "Swerve");
        Crashboard.toDashboard(this.ModuleName + " speed in mps", state.speedMetersPerSecond, "Swerve");
        
                
        double turnSpeed = (turningPidController.calculate(getAbsoluteEncoderDeg(), state.angle.getDegrees()));
        //System.out.println("Turn Speed Calculated " + this.ModuleName + ": " + turnSpeed);
        if (turnSpeed > 0)
            turnSpeed = Math.min(turnSpeed, .2);
        else
            turnSpeed = Math.max(turnSpeed, -.2);
        
        //System.out.println("Turn Speed Final " + this.ModuleName + ": " + turnSpeed);
        Crashboard.toDashboard(ModuleName + "Turn Speed Final", turnSpeed, "Swerve");

        turningMotor.set(turnSpeed);

        double outVoltage = driveOutput;// + driveFeedForwardOutput;
        outVoltage = MathUtil.clamp(outVoltage, -4.0 ,4.0);
        Crashboard.toDashboard("drive voltage PID", driveOutput, this.ModuleName + " Swerve");
        Crashboard.toDashboard("drive voltage ff", driveFeedForwardOutput, this.ModuleName + " Swerve");
        Crashboard.toDashboard(" output voltage", outVoltage, this.ModuleName + " Swerve");
        

        //outVoltage = MathUtil.clamp(outVoltage, -5, 5);

        Crashboard.toDashboard(this.ModuleName + " clamp voltage", outVoltage, "Swerve");
        
        driveMotor.setVoltage(outVoltage);
        //System.out.println(ModuleName + "- DriveMotorCommand: " + driveSpeed + " - True Angle: " + getAbsoluteEncoderRad() + " AngleSetPoint: " + state.angle.getDegrees() + " AngleMotorCommand: " + turnSpeed);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public CANcoder getCANcoder() {
        return this.absoluteEncoder;
    }

    public void logIt() {
      Crashboard.toDashboard(ModuleName + " Wheel Angle", this.getAbsoluteEncoderDeg(), "swerve");

      

    }
}