package frc.robot;
import frc.robot.swerveDrive;
import com.kauailabs.navx.frc.AHRS; 

public class Objects {
    swerveDrive motorCalculator = new swerveDrive();
    Motors motors = new Motors();
    public static AHRS navx = new AHRS();
}