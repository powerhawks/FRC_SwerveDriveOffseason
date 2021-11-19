package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

public class Motors {
    public static CANSparkMax driveMotorBottomRight     = new CANSparkMax(1,  CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax turnMotorBottomRight     = new CANSparkMax(2,  CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax driveMotorTopRight = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax turnMotorTopRight = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);
}