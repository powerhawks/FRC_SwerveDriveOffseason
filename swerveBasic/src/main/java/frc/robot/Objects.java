package frc.robot;
import frc.robot.Calculations;
import com.kauailabs.navx.frc.AHRS; 

public class Objects {
    Calculations motorCalculator = new Calculations();
    Motors motors = new Motors();
    public static AHRS navx = new AHRS();
}