package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

@Configurable
public class DistanceSensorClass
{
    public static double CHANGE_POINT = .18;
    double ballsShotCount;
    private double distance;
    private OpticalDistanceSensor distanceSensor;
    private boolean wasBallDetected;

    public void init(HardwareMap hardwareMap) {
        ballsShotCount=0;
        wasBallDetected = false;
        distanceSensor = hardwareMap.get(OpticalDistanceSensor.class, "distanceSensor");
    }

    public double GetBallsShotCount(){
        return ballsShotCount;
    }

    public void ResetBallsShotCount(){
        ballsShotCount=0;
    }
    public void update(){
        distance = GetDistance();
        if (!wasBallDetected && IsBallDetected()){
            wasBallDetected=true;
        }
        if (wasBallDetected&&!IsBallDetected()){
            wasBallDetected=false;
            ballsShotCount+=1;
        }


    }

    public double GetDistance(){
        return distanceSensor.getLightDetected();
    }

    public boolean IsBallDetected(){

        if (distance>= CHANGE_POINT){return false;}
        else{return true;}
    }

}
