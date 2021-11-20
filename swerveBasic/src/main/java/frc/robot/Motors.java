package frc.robot;

import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;

public class Motors {
    public static CANSparkMax driveMotor0    = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax turnMotor0     = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    public static CANSparkMax driveMotor1    = new CANSparkMax(3,  CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax turnMotor1     = new CANSparkMax(4,  CANSparkMaxLowLevel.MotorType.kBrushless);

    public static CANSparkMax driveMotor2    = new CANSparkMax(5,  CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax turnMotor2     = new CANSparkMax(6,  CANSparkMaxLowLevel.MotorType.kBrushless);

    public static CANSparkMax driveMotor3    = new CANSparkMax(7,  CANSparkMaxLowLevel.MotorType.kBrushless);
    public static CANSparkMax turnMotor3     = new CANSparkMax(8,  CANSparkMaxLowLevel.MotorType.kBrushless);
}