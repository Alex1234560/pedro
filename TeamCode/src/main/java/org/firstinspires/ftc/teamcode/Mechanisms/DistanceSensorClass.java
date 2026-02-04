package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

import org.firstinspires.ftc.teamcode.Functions.FunctionsAndValues;

@Configurable
public class DistanceSensorClass
{
    public static double CHANGE_POINT = .15;
    public static double TOO_FAR = .15;



    public static double CHANGE_POINT_FOR_COUNTING = .12;
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

    public void SetBallsShotCount(double Balls){
        ballsShotCount=Balls;
    }

    public void update(){
        distance = GetDistance();

        //.15?
        if (!wasBallDetected && IsBallDetectedForCounting()){
            wasBallDetected=true;
        }
        if (wasBallDetected&&!IsBallDetectedForCounting()){
            wasBallDetected=false;
            ballsShotCount+=1;
        }


    }

    public double GetDistance(){
        return distanceSensor.getLightDetected();
    }

    public boolean IsBallDetected(){

        if (distance> CHANGE_POINT){return false;}
        else{return true;}
    }
    public boolean IsBallTooFarIn(){
        if (distance<= TOO_FAR){return true;}
        else{return false;}
    }

    public double ReturnValueForPreload(){
        double power=-0.07;
       if (distance>CHANGE_POINT){
           power = FunctionsAndValues.PowerValueForPreloading;

       }
        return power;
    }

    public boolean IsBallDetectedForCounting(){

        if (distance>= CHANGE_POINT_FOR_COUNTING){return false;}
        else{return true;}
    }

}
