package frc.robot;
import frc.robot.SwerveDrive;
import com.kauailabs.navx.frc.AHRS; 

public class Objects {
    SwerveDrive swerveDrive = new SwerveDrive();
    Motors motors = new Motors();
    public static AHRS navx = new AHRS();
}