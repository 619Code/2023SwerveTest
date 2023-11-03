package frc.robot.subsystems;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.ModuleConstants;

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
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final double absoluteEncoderOffset;

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

        this.absoluteEncoderOffset = absoluteEncoderOffset;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        configureMotor(driveMotor, driveMotorReversed);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        configureMotor(turningMotor, turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Deg);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2DegPerSec);
 
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-180, 180);

        resetEncoders();
    }

    private void configureMotor(CANSparkMax motor, Boolean inverted) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(30);
        motor.burnFlash();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderDeg() {
        return (absoluteEncoder.getAbsolutePosition().getValue() * 360);// - absoluteEncoderOffset;

        //should be in degrees???
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderDeg()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        double driveSpeed = state.speedMetersPerSecond*0.2;
        driveMotor.set(driveSpeed);
        double turnSpeed = (turningPidController.calculate(getAbsoluteEncoderDeg(), state.angle.getDegrees()))/20;
        turningMotor.set(turnSpeed);
        //System.out.println(ModuleName + "- DriveMotorCommand: " + driveSpeed + " - True Angle: " + getAbsoluteEncoderRad() + " AngleSetPoint: " + state.angle.getDegrees() + " AngleMotorCommand: " + turnSpeed);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public CANcoder getCANcoder() {
        return this.absoluteEncoder;
    }
}