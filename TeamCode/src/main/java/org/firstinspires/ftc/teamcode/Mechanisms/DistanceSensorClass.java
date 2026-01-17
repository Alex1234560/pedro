package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


public class DistanceSensorClass
{
    public static double CHANGE_POINT = 0.16;
    private double distance;
    private OpticalDistanceSensor distanceSensor;

    public void init(HardwareMap hardwareMap) {
        distanceSensor = hardwareMap.get(OpticalDistanceSensor.class, "distanceSensor");
    }

    public void update(){
        distance = GetDistance();
    }

    public double GetDistance(){
        return distanceSensor.getLightDetected();
    }

    public boolean IsBallDetected(){

        if (distance>= CHANGE_POINT){return false;}
        else{return true;}
    }

}
