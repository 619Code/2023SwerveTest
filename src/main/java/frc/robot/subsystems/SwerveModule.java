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
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

public class SwerveModule {
    public final CANSparkMax driveMotor;
    public final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANCoder absoluteEncoder;
    private final double absoluteEncoderOffset;

    private final String ModuleName;

    public SwerveModule(String ModuleName, int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.ModuleName = ModuleName;
        absoluteEncoder = new CANCoder(absoluteEncoderId);
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.magnetOffsetDegrees = absoluteEncoderOffset;
        canCoderConfiguration.sensorDirection = absoluteEncoderReversed;
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        absoluteEncoder.configAllSettings(canCoderConfiguration);

        this.absoluteEncoderOffset = absoluteEncoderOffset;

        driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
        configureMotor(driveMotor, driveMotorReversed);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);
        configureMotor(turningMotor, turningMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);
 
        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-180, 180);

        resetEncoders();
    }

    private void configureMotor(CANSparkMax motor, Boolean inverted) {
        motor.restoreFactoryDefaults();
        motor.setIdleMode(IdleMode.kBrake);
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(40);
        motor.burnFlash();
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(getAbsoluteEncoderRad()));
    }

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    public double getAbsoluteEncoderRad() {
        return absoluteEncoder.getAbsolutePosition();
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(0);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getAbsoluteEncoderRad()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        double driveSpeed = state.speedMetersPerSecond*0.2;
        driveMotor.set(driveSpeed);
        double turnSpeed = (turningPidController.calculate(getAbsoluteEncoderRad(), state.angle.getDegrees()))/20;
        turningMotor.set(turnSpeed);
        //System.out.println(ModuleName + "- DriveMotorCommand: " + driveSpeed + " - True Angle: " + getAbsoluteEncoderRad() + " AngleSetPoint: " + state.angle.getDegrees() + " AngleMotorCommand: " + turnSpeed);
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}