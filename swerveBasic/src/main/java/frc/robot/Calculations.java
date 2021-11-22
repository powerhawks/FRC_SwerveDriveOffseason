package frc.robot;

import frc.robot.Objects;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import java.math.*;

import com.revrobotics.CANSparkMax;
public class Calculations {
    //constructor lmao
    private final Encoder m_encoderModule0 = new Encoder(0,1);
    private final Encoder m_encoderModule1 = new Encoder(2,3);
    private final Encoder m_encoderModule2 = new Encoder(4,5);
    private final Encoder m_encoderModule3 = new Encoder(6,7);

    public Calculations () {
    
    }

    /**
     * Takes in three joystick values and drives all four swerve units <br><br>
     * Motors are numbered 0 through 3, starting in the top right (first quadrant) <br><br>
     * This math works by adding up the component x and y vals of the translation and rotation, then puts the vectors
     * back together to get the final wheel speed and direction <br><br>
     * @param xVal
     * <ul><li> Translation X value from -1 to 1 </ul></li>
     * @param yVal
     * <ul><li> Translation Y value from -1 to 1 </ul></li>
     * @param rotVal
     * <ul><li> Rotation Value from -1 to 1 </ul></li>
     * @param velocityScalerDrive
     * <ul><li> Decimal percentage to run the drive motor (usually 0 to 1 but can be more or less) </ul></li>
     * @param velocityScalerRotate
     * <ul><li> Decimal percentage to run the rotation motor (usually 0 to 1 but can be more or less) </ul></li>
     */
    public void allWheelDrive(double xVal, double yVal, double rotVal, double velocityScalerDrive, double velocityScalerRotate) {
        
        /**
         * Sets the deadband for the controller
         */
        xVal = Math.abs(xVal) > 0.03 ? xVal : 0;
        yVal = Math.abs(yVal) > 0.03 ? yVal : 0;
        rotVal = Math.abs(rotVal) > 0.03 ? rotVal : 0;

        /**
         * Determines the speed and direction for each motor
         */

        // top right wheel 0
        double module0Angle = (3 * Math.PI) / 4; // angle from the center of the robot to the module
        double m0x = xVal - (rotVal * Math.cos(module0Angle)); // add the x components of the translation and rotation (negative because the encoders go backwards)
        double m0y = yVal + (rotVal * Math.sin(module0Angle)); // add the y components of the translation and rotation
        double m0Speed = Math.sqrt( (Math.pow(m0x, 2)) + (Math.pow(m0y, 2))); // find the magnitude of the hypotenuse between the x and y components
        double m0Angle = Math.atan2(m0y, m0x); // find the angle that the x and y vals make (IMPORTANT! This range is -pi to pi, could break something by not being 0 to 2pi)

        // top left wheel 1
        double module1Angle = (5 * Math.PI) / 4; // angle from the center of the robot to the module
        double m1x = xVal - (rotVal * Math.cos(module1Angle)); // add the x components of the translation and rotation (negative because the encoders go backwards)
        double m1y = yVal + (rotVal * Math.sin(module1Angle)); // add the y components of the translation and rotation
        double m1Speed = Math.sqrt( (Math.pow(m1x, 2)) + (Math.pow(m1y, 2))); // find the magnitude of the hypotenuse between the x and y components
        double m1Angle = Math.atan2(m1y, m1x); // find the angle that the x and y vals make (IMPORTANT! This range is -pi to pi, could break something by not being 0 to 2pi)

        // bottom left wheel 2
        double module2Angle = (7 * Math.PI) / 4; // angle from the center of the robot to the module
        double m2x = xVal - (rotVal * Math.cos(module2Angle)); // add the x components of the translation and rotation (negative because the encoders go backwards)
        double m2y = yVal + (rotVal * Math.sin(module2Angle)); // add the y components of the translation and rotation
        double m2Speed = Math.sqrt( (Math.pow(m2x, 2)) + (Math.pow(m2y, 2))); // find the magnitude of the hypotenuse between the x and y components
        double m2Angle = Math.atan2(m2y, m2x); // find the angle that the x and y vals make (IMPORTANT! This range is -pi to pi, could break something by not being 0 to 2pi)

        //bottom right wheel 3
        double module3Angle = (1 * Math.PI) / 4; // angle from the center of the robot to the module
        double m3x = xVal - (rotVal * Math.cos(module3Angle)); // add the x components of the translation and rotation (negative because the encoders go backwards)
        double m3y = yVal + (rotVal * Math.sin(module3Angle)); // add the y components of the translation and rotation
        double m3Speed = Math.sqrt( (Math.pow(m3x, 2)) + (Math.pow(m3y, 2))); // find the magnitude of the hypotenuse between the x and y components
        double m3Angle = Math.atan2(m3y, m3x); // find the angle that the x and y vals make (IMPORTANT! This range is -pi to pi, could break something by not being 0 to 2pi
        
        
        /**
         * Normalizes wheel speeds so no wheel is going over 100 percent
         */

        double maxWheelSpeed = m0Speed;
        maxWheelSpeed = Math.max(maxWheelSpeed, m1Speed);
        maxWheelSpeed = Math.max(maxWheelSpeed, m2Speed);
        maxWheelSpeed = Math.max(maxWheelSpeed, m3Speed);
        if (maxWheelSpeed > 1) {
            m0Speed /= maxWheelSpeed;
            m1Speed /= maxWheelSpeed;
            m2Speed /= maxWheelSpeed;
            m3Speed /= maxWheelSpeed;
        }

        /**
         * Sends the direction and speed of each module to the motor driver function
         */
        
        setSwerveUnitState(Motors.driveMotor0, Motors.turnMotor0, m_encoderModule0, m0Speed, m0Angle, velocityScalerDrive, velocityScalerRotate);
        setSwerveUnitState(Motors.driveMotor1, Motors.turnMotor1, m_encoderModule1, m1Speed, m1Angle, velocityScalerDrive, velocityScalerRotate);
        setSwerveUnitState(Motors.driveMotor2, Motors.turnMotor2, m_encoderModule2, m2Speed, m2Angle, velocityScalerDrive, velocityScalerRotate);
        setSwerveUnitState(Motors.driveMotor3, Motors.turnMotor3, m_encoderModule3, m3Speed, m3Angle, velocityScalerDrive, velocityScalerRotate);

    }

    public void setSwerveUnitState(CANSparkMax driveMotor, CANSparkMax rotateMotor, Encoder swerveUnitEncoder, double driveMotorSpeed, double moduleAngle, double driveMotorSpeedScale, double rotateMotorSpeedScale) {
        
        /**
         * The following section is the rotation direction optimizer, and finds the shortest path to rotate the motor
         */
        double encoderRotation = ( (swerveUnitEncoder.getRaw() / 4096.0) % 1) * ( 2 * Math.PI); //convert the encoder val to radians. the mod 1 is because the rotation can go above and below a full rotation, so mod 1 removes extras
        double signedDiffNormal = closestAngleCalculator(encoderRotation, moduleAngle);
        double signedDiffReverse = closestAngleCalculator(encoderRotation, moduleAngle - Math.PI);
        double signedDiff = 0.0;
        if (Math.abs(signedDiffNormal) <= Math.abs(signedDiffReverse)) {
            signedDiff = signedDiffNormal;
        }
        else {
            driveMotorSpeedScale = -driveMotorSpeedScale; //run the drive wheel backwards
            signedDiff = signedDiffReverse;
        }

        driveMotor.set(Math.cos(signedDiff) * driveMotorSpeed * driveMotorSpeedScale); //drive motor power is cosine of error delta so the wheels don't fight each other
        double percentError = signedDiff / (Math.PI); //proportion error control
        rotateMotor.set(-percentError * rotateMotorSpeedScale); //proportional error control
    }

    public double closestAngleCalculator(double encoderRotation, double moduleAngle) {
        double signedDiff = 0.0;
        double rawDiff = encoderRotation > moduleAngle ? encoderRotation - moduleAngle : moduleAngle - encoderRotation;
        double modDiff = rawDiff % (2 * Math.PI);

        if (modDiff > Math.PI) {
            signedDiff = ((2 * Math.PI) - modDiff);
            if (moduleAngle > encoderRotation) signedDiff = signedDiff * -1;
        }
        else {
            signedDiff = modDiff;
            if (encoderRotation > moduleAngle) signedDiff = signedDiff * -1;
        }
        return signedDiff;
    }


}


