package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

//import edu.wpi.first.math.kinematics.SwerveModuleState; //prebuilt swerve is cringe and im not using it.
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveSub extends SubsystemBase {
    CANSparkMax[] driveMotors = new CANSparkMax[4];
    CANSparkMax[] rotateMotors = new CANSparkMax[4];

    public SwerveDriveSub() {

        //if you get mad at me. i will fight you over this.
        for (int i = 0; i < 4; i ++) {
            driveMotors[i] = new CANSparkMax(Constants.DRIVE_MOTORS[i], MotorType.kBrushless);
            rotateMotors[i] = new CANSparkMax(Constants.ROTATION_MOTORS[i], MotorType.kBrushless);
        }
        
    }
}
