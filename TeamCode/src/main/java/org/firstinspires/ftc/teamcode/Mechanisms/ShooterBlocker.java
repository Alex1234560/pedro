package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable

public class ShooterBlocker {
    private Servo ShooterBlocker;

    //private static double ShooterAngle = FunctionsAndValues.startPoint;

    public static double UNBLOCK_POINT = .3;
    public static double BLOCK_POINT = .2;//.7


    public void init(HardwareMap hardwareMap){
        ShooterBlocker = hardwareMap.get(Servo.class, "ShooterBlocker");

    }

    public void update(){
    }

    private void SetPosition(double position){
        ShooterBlocker.setPosition(position);
    }

    public void Block(){
       SetPosition(BLOCK_POINT);
    }
    public void Unblock(){
        SetPosition(UNBLOCK_POINT);
    }

    public double getPosition(){
        //give position so that when moving manually it can start from the current place.
        return ShooterBlocker.getPosition();//position
    }


}
