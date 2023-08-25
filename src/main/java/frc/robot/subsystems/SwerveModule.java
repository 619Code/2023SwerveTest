package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPIDController;

    private final AnalogInput absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffSetRad;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveMotorReversed, boolean turningMotorReversed, 
                        int absoluteEncoderID, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
            
            this.absoluteEncoderOffSetRad = absoluteEncoderOffset;
            this.absoluteEncoderReversed = absoluteEncoderReversed;
            absoluteEncoder = new AnalogInput(absoluteEncoderID);

            driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
            turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

            driveMotor.setInverted(driveMotorReversed);
            turningMotor.setInverted(turningMotorReversed);

            driveEncoder = driveMotor.getEncoder();
            turningEncoder = turningMotor.getEncoder();

            driveEncoder.setPositionConversionFactor(Constants.kDriveEncoderRot2Meter);
            driveEncoder.setVelocityConversionFactor(Constants.kDriveEncoderRPM2MeterPerSec);
            turningEncoder.setPositionConversionFactor(Constants.kTurningEncoderRot2Rad);
            turningEncoder.setVelocityConversionFactor(Constants.kTurningEncoderRPM2RadPerSec);

            turningPIDController = new PIDController(Constants.kPturning,0,0);
            turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

            resetEncoders();

        }

    public double getDrivePosition() {
        return 0.0; //driveEncoder.getPosition();
    }
    public double getTurningPosition(){
        return turningEncoder.getPosition();
    }
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }
    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getCurrent5V();
        angle *= 2 * Math.PI;
        angle -= absoluteEncoderOffSetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }
    
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
        }

        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / Constants.kMaxSpeed);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
    }


}
