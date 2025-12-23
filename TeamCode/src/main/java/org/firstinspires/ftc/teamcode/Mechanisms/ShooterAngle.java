package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.FunctionsAndValues;

//@Configurable

public class ShooterAngle {
    private Servo ServoShooter1;
    //private static double ShooterAngle = FunctionsAndValues.startPoint;

    private static double START_POINT = .15;
    private static double END_POINT = .9;//.7

    private double normalize(double value){
        double newValue = value;
        if (value>END_POINT) {
            newValue = END_POINT;
        }
        if (value<START_POINT) {
            newValue = START_POINT;
        }
        return newValue;
    }

    public void init(HardwareMap hardwareMap){
        ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");
    }

    public void SetPosition(double position){
        position = normalize(position);
        ServoShooter1.setPosition(position);
    }

    public double getPosition(){
        //give position so that when moving manually it can start from the current place.
        return 1984;//position
    }

    public void SetPositionBasedOnDistance(double Distance){

    }





}
