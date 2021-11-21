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
    
    public void calculations (double xVal, double yVal, double rotation, double maxVelocity) {
        SmartDashboard.putNumber("X value", xVal);
        SmartDashboard.putNumber("Y value", yVal);
        SmartDashboard.putNumber("Rotation value", rotation);
        SmartDashboard.putNumber("Encoder", m_encoderModule3.getRaw());
        bottomRightMotorCalc(xVal, yVal, rotation, maxVelocity);
        TopRightMotorCalc(xVal, yVal, rotation, maxVelocity);
        

    }

    public void TopRightMotorCalc (double xVal, double yVal, double rotation, double maxVelocity) {
        double maxWheelVeolcity = maxVelocity; //feet per second at full power
        double maxRotationalVelocity = Math.PI*2*Math.sqrt(2);

        double errValue = 0; //rotation optimizer
        boolean rotationDirection; //rotation optimizer cw=true ccw = false
        double xInputVelocity = xVal * maxWheelVeolcity;
        double yInputVelocity = yVal * maxWheelVeolcity;
        double rotationalV = (rotation * maxRotationalVelocity); //TODO UNDERSTAND ROTATIONAL VELOCITY POSITIVE?NEGATIVE
        double resultantX = xInputVelocity + (-(Math.sqrt(2)/2)*(rotationalV));
        double resultantY = yInputVelocity + ((Math.sqrt(2)/2)*(rotationalV));
        double resultantVectors = Math.sqrt(Math.pow(resultantX, 2)+Math.pow(resultantY, 2));

        double resultantAngle = Math.atan(resultantY/resultantX);
        if (xVal <= 0+.01) {
            resultantAngle += Math.PI;
         } else if (xVal > 0 && yVal <=0 )
        {

            resultantAngle += (2*Math.PI);
        }
        double motorOutput = resultantVectors/85;
        SmartDashboard.putNumber("Resultant Value", resultantVectors);
        SmartDashboard.putNumber("Resultant Angle", resultantAngle);
        SmartDashboard.putNumber("Motor Output", motorOutput);
        double encoderRotation = (((m_encoderModule0.getRaw()/4096.000000)%1)*(2*Math.PI));
        SmartDashboard.putNumber("RawValue", encoderRotation);
        if (encoderRotation < 0 ) {
            encoderRotation *= -1;
        } else {
            encoderRotation = (2 * Math.PI) - encoderRotation;
        }
        SmartDashboard.putNumber("Encoder", encoderRotation);
        Motors.driveMotor0.set(-motorOutput);
        
        double rawDistanceFromSetpoint = encoderRotation - resultantAngle;
        double limitedDistance = rawDistanceFromSetpoint % (2 * Math.PI);

        if (limitedDistance < Math.PI) {
            rotationDirection = limitedDistance > 0 ? true : false;
        }
        else {
            rotationDirection = limitedDistance > 0 ? false : true;
        }
        if (Math.abs(limitedDistance) > (Math.PI / 32)){
            Motors.turnMotor0.set(rotationDirection ? -0.05 : 0.05);
        }
        else {
            Motors.turnMotor0.set(0);
        }

    } 
    public void bottomRightMotorCalc (double xVal, double yVal, double rotation, double maxVelocity) {
        double maxWheelVeolcity = maxVelocity; //feet per second at full power
        double maxRotationalVelocity = Math.PI*2*Math.sqrt(2);

        double errValue = 0; //rotation optimizer
        boolean rotationDirection; //rotation optimizer cw=true ccw = false
        double xInputVelocity = xVal * maxWheelVeolcity;
        double yInputVelocity = yVal * maxWheelVeolcity;
        double rotationalV = (-rotation * maxRotationalVelocity);
        double resultantX = xInputVelocity + ((Math.sqrt(2)/2)*(rotationalV));
        double resultantY = yInputVelocity + ((Math.sqrt(2)/2)*(rotationalV));
        double resultantVectors = Math.sqrt(Math.pow(resultantX, 2)+Math.pow(resultantY, 2));

        double resultantAngle = Math.atan(resultantY/resultantX);
        if (xVal <= 0+.01) {
            resultantAngle += Math.PI;
         } else if (xVal > 0 && yVal <=0 )
        {
            
            resultantAngle += (2*Math.PI);
        }
        double motorOutput = resultantVectors/85;
        //SmartDashboard.putNumber("Resultant Value", resultantVectors);
        //SmartDashboard.putNumber("Resultant Angle", resultantAngle);
        //SmartDashboard.putNumber("Motor Output", motorOutput);
        double encoderRotation = (((m_encoderModule3.getRaw()/4096.000000)%1)*(2*Math.PI));
        SmartDashboard.putNumber("RawValue", encoderRotation);
        if (encoderRotation < 0 ) {
            encoderRotation *= -1;
        } else {
            encoderRotation = (2 * Math.PI) - encoderRotation;
        }
        //SmartDashboard.putNumber("Encoder", encoderRotation);
        Motors.driveMotor3.set(-motorOutput);
        
        double rawDistanceFromSetpoint = encoderRotation - resultantAngle;
        double limitedDistance = rawDistanceFromSetpoint % (2 * Math.PI);

        if (limitedDistance < Math.PI) {
            rotationDirection = limitedDistance > 0 ? true : false;
        }
        else {
            rotationDirection = limitedDistance > 0 ? false : true;
        }
        if (Math.abs(limitedDistance) > (Math.PI / 32)){
            Motors.turnMotor3.set(rotationDirection ? -0.05 : 0.05);
        }
        else {
            Motors.turnMotor3.set(0);
        }

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
         * Determines the speed and direction for each motor
         */
        xVal = Math.abs(xVal) > 0.03 ? xVal : 0;
        yVal = Math.abs(yVal) > 0.03 ? yVal : 0;
        rotVal = Math.abs(rotVal) > 0.03 ? rotVal : 0;
        // top right wheel 0
        double module0Angle = (1 * Math.PI) / 4; // angle from the center of the robot to the module
        double m0x = xVal + (rotVal * Math.cos(module0Angle)); // add the x components of the translation and rotation
        double m0y = yVal + (rotVal * Math.sin(module0Angle)); // add the y components of the translation and rotation
        double m0Speed = Math.sqrt( (Math.pow(m0x, 2)) + (Math.pow(m0y, 2))); // find the magnitude of the hypotenuse between the x and y components
        double m0Angle = Math.atan2(m0y, m0x); // find the angle that the x and y vals make (IMPORTANT! This range is -pi to pi, could break something by not being 0 to 2pi)
        m0Angle = m0Angle < 0 ? m0Angle + (2*Math.PI) : m0Angle;
        SmartDashboard.putNumber("m0Angle", m0Angle);
        SmartDashboard.putNumber("m0Acual", ((m_encoderModule0.getRaw()/4096.0)%1)*(2*Math.PI));

        // top left wheel 1
        double module1Angle = (7 * Math.PI) / 4; // angle from the center of the robot to the module
        double m1x = xVal + (rotVal * Math.cos(module1Angle)); // add the x components of the translation and rotation
        double m1y = yVal + (rotVal * Math.sin(module1Angle)); // add the y components of the translation and rotation
        double m1Speed = Math.sqrt( (Math.pow(m1x, 2)) + (Math.pow(m1y, 2))); // find the magnitude of the hypotenuse between the x and y components
        double m1Angle = Math.atan2(m1y, m1x); // find the angle that the x and y vals make (IMPORTANT! This range is -pi to pi, could break something by not being 0 to 2pi)
        m1Angle = m1Angle < 0 ? m1Angle + (2 * Math.PI) : m1Angle;
        SmartDashboard.putNumber("m1Angle", m1Angle);
        SmartDashboard.putNumber("m1Acual", ((m_encoderModule1.getRaw()/4096.0)%1)*(2*Math.PI));

        // bottom left wheel 2
        double module2Angle = (5 * Math.PI) / 4; // angle from the center of the robot to the module
        double m2x = xVal + (rotVal * Math.cos(module2Angle)); // add the x components of the translation and rotation
        double m2y = yVal + (rotVal * Math.sin(module2Angle)); // add the y components of the translation and rotation
        double m2Speed = Math.sqrt( (Math.pow(m2x, 2)) + (Math.pow(m2y, 2))); // find the magnitude of the hypotenuse between the x and y components
        double m2Angle = Math.atan2(m2y, m2x); // find the angle that the x and y vals make (IMPORTANT! This range is -pi to pi, could break something by not being 0 to 2pi)
        m2Angle = m2Angle < 0 ? m2Angle + (2 * Math.PI) : m2Angle;
        SmartDashboard.putNumber("m2Angle", m2Angle);
        SmartDashboard.putNumber("m2Acual", ((m_encoderModule2.getRaw()/4096.0)%1)*(2*Math.PI));

        //bottom right wheel 3
        double module3Angle = (3 * Math.PI) / 4; // angle from the center of the robot to the module
        double m3x = xVal + (rotVal * Math.cos(module3Angle)); // add the x components of the translation and rotation
        double m3y = yVal + (rotVal * Math.sin(module3Angle)); // add the y components of the translation and rotation
        double m3Speed = Math.sqrt( (Math.pow(m3x, 2)) + (Math.pow(m3y, 2))); // find the magnitude of the hypotenuse between the x and y components
        double m3Angle = Math.atan2(m3y, m3x); // find the angle that the x and y vals make (IMPORTANT! This range is -pi to pi, could break something by not being 0 to 2pi
        m3Angle = m3Angle < 0 ? m3Angle + (2 * Math.PI) : m3Angle;
        SmartDashboard.putNumber("m3Angle", m3Angle);
        SmartDashboard.putNumber("m3Acual", ((m_encoderModule3.getRaw()/4096.0)%1)*(2*Math.PI));

        
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
        SmartDashboard.putNumber("RawValue", m3Angle);
        setSwerveUnitState(Motors.driveMotor0, Motors.turnMotor0, m_encoderModule0, m0Speed, m0Angle, velocityScalerDrive, velocityScalerRotate);
        setSwerveUnitState(Motors.driveMotor1, Motors.turnMotor1, m_encoderModule1, m1Speed, m1Angle, velocityScalerDrive, velocityScalerRotate);
        setSwerveUnitState(Motors.driveMotor2, Motors.turnMotor2, m_encoderModule2, m2Speed, m2Angle, velocityScalerDrive, velocityScalerRotate);
        setSwerveUnitState(Motors.driveMotor3, Motors.turnMotor3, m_encoderModule3, m3Speed, m3Angle, velocityScalerDrive, velocityScalerRotate);

    }

    public void setSwerveUnitState(CANSparkMax driveMotor, CANSparkMax rotateMotor, Encoder swerveUnitEncoder, double driveMotorSpeed, double moduleAngle, double driveMotorSpeedScale, double rotateMotorSpeedScale) {
        driveMotor.set(driveMotorSpeed * driveMotorSpeedScale); //sets the drive motor power
        
        /**
         * The following section is the rotation direction optimizer, and finds the shortest path to rotate the motor
         */
        double encoderRotation = ( (swerveUnitEncoder.getRaw() / 4096.0) % 1) * ( 2 * Math.PI); //convert the encoder val to radians. the mod 1 is because the rotation can go above and below a full rotation, so mod 1 removes extras
        if (encoderRotation < 0 ) {
            encoderRotation *= -1;
            encoderRotation = (2 * Math.PI) - encoderRotation;
        }
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

        driveMotor.set(Math.cos(signedDiff) * driveMotorSpeed * driveMotorSpeedScale); //sets the drive motor power

        double percentError = signedDiff / (Math.PI);
        SmartDashboard.putNumber("encoderRotation", encoderRotation);
        SmartDashboard.putNumber("signedDiff", signedDiff);
        SmartDashboard.putNumber("percentError", percentError);
        if (Math.abs(signedDiff) > (Math.PI / 128)){
            rotateMotor.set(-percentError * rotateMotorSpeedScale); //proportional error control
        }
        else {
            rotateMotor.set(0);
        }
    }

    public double turnAcceleration (double difference) {
        difference = Math.abs(difference);
        double motorspeed = ((1/(10*Math.PI))*Math.pow(difference, 1.4));
        return motorspeed;
    }
    public void abstractMotorCalc (double xVal, double yVal, double rotation, double maxVelocity, double rotationalVangle, CANSparkMax driveMotor, CANSparkMax turnMotor, Encoder turnEncoder ) {
        double maxWheelVeolcity = maxVelocity; //feet per second at full power
        double maxRotationalVelocity = Math.PI*2*Math.sqrt(2);

        double errValue = 0; //rotation optimizer
        boolean rotationDirection; //rotation optimizer cw=true ccw = false
        double xInputVelocity = xVal * maxWheelVeolcity;
        double yInputVelocity = yVal * maxWheelVeolcity;
        double rotationalV = (-rotation * maxRotationalVelocity);
        double resultantX = xInputVelocity + (Math.cos(rotationalVangle)*(rotationalV));
        double resultantY = yInputVelocity + ((Math.sin(rotationalVangle))*(rotationalV));
        double resultantVectors = Math.sqrt(Math.pow(resultantX, 2)+Math.pow(resultantY, 2));

        double resultantAngle = Math.atan(resultantY/resultantX);
        if (resultantX <= 0) {
            resultantAngle += Math.PI;
        } else if (resultantX > 0 && resultantY <=0 )
        {
            
            resultantAngle += (2*Math.PI);
        }
        double motorOutput = resultantVectors/85;
        //SmartDashboard.putNumber("Resultant Value", resultantVectors);
        //SmartDashboard.putNumber("Resultant Angle", resultantAngle);
        //SmartDashboard.putNumber("Motor Output", motorOutput);
        double encoderRotation = (((turnEncoder.getRaw()/4096.000000)%1)*(2*Math.PI));
        SmartDashboard.putNumber("RawValue", encoderRotation);
        if (encoderRotation < 0 ) {
            encoderRotation *= -1;
        } else {
            encoderRotation = (2 * Math.PI) - encoderRotation;
        }
        //SmartDashboard.putNumber("Encoder", encoderRotation);
        driveMotor.set(-motorOutput);
        
        double rawDistanceFromSetpoint = encoderRotation - resultantAngle;
        double limitedDistance = rawDistanceFromSetpoint % (2 * Math.PI);

        if (limitedDistance < Math.PI) {
            rotationDirection = limitedDistance > 0 ? true : false;
        }
        else {
            rotationDirection = limitedDistance > 0 ? false : true;
        }
        if (Math.abs(limitedDistance) > (Math.PI / 32)){
            turnMotor.set(rotationDirection ? -0.05 : 0.05);
        }
        else {
            turnMotor.set(0);
        }

    }

}


