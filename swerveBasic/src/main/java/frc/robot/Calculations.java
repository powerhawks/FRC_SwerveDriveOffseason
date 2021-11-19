package frc.robot;

import frc.robot.Objects;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import java.math.*;
public class Calculations {
    /** TODO FIX ROTATIONAL VELOCITY ISSUE */
    //constructor lmao
    private final Encoder m_turningEncoderBottomRight = new Encoder(0, 1);
    private final Encoder m_turningEncoderTopRight = new Encoder (2,3);
    public Calculations () {
    
    }
    
    public void calculations (double xVal, double yVal, double rotation, double maxVelocity) {
        SmartDashboard.putNumber("X value", xVal);
        SmartDashboard.putNumber("Y value", yVal);
        SmartDashboard.putNumber("Rotation value", rotation);
        SmartDashboard.putNumber("Encoder", m_turningEncoderBottomRight.getRaw());
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
        double encoderRotation = (((m_turningEncoderTopRight.getRaw()/4096.000000)%1)*(2*Math.PI));
        SmartDashboard.putNumber("RawValue", encoderRotation);
        if (encoderRotation < 0 ) {
            encoderRotation *= -1;
        } else {
            encoderRotation = (2 * Math.PI) - encoderRotation;
        }
        SmartDashboard.putNumber("Encoder", encoderRotation);
        Motors.driveMotorTopRight.set(-motorOutput);
        
        double rawDistanceFromSetpoint = encoderRotation - resultantAngle;
        double limitedDistance = rawDistanceFromSetpoint % (2 * Math.PI);

        if (limitedDistance < Math.PI) {
            rotationDirection = limitedDistance > 0 ? true : false;
        }
        else {
            rotationDirection = limitedDistance > 0 ? false : true;
        }
        if (Math.abs(limitedDistance) > (Math.PI / 32)){
            Motors.turnMotorTopRight.set(rotationDirection ? -0.05 : 0.05);
        }
        else {
            Motors.turnMotorTopRight.set(0);
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
        double encoderRotation = (((m_turningEncoderBottomRight.getRaw()/4096.000000)%1)*(2*Math.PI));
        SmartDashboard.putNumber("RawValue", encoderRotation);
        if (encoderRotation < 0 ) {
            encoderRotation *= -1;
        } else {
            encoderRotation = (2 * Math.PI) - encoderRotation;
        }
        //SmartDashboard.putNumber("Encoder", encoderRotation);
        Motors.driveMotorBottomRight.set(-motorOutput);
        
        double rawDistanceFromSetpoint = encoderRotation - resultantAngle;
        double limitedDistance = rawDistanceFromSetpoint % (2 * Math.PI);

        if (limitedDistance < Math.PI) {
            rotationDirection = limitedDistance > 0 ? true : false;
        }
        else {
            rotationDirection = limitedDistance > 0 ? false : true;
        }
        if (Math.abs(limitedDistance) > (Math.PI / 32)){
            Motors.turnMotorBottomRight.set(rotationDirection ? -0.05 : 0.05);
        }
        else {
            Motors.turnMotorBottomRight.set(0);
        }

    } 
}


