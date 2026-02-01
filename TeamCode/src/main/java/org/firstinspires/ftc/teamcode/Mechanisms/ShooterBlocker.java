package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable

public class ShooterBlocker {
    private Servo ShooterBlocker;

    //private static double ShooterAngle = FunctionsAndValues.startPoint;

    public static double UNBLOCK_POINT = 0.30;
    public static double BLOCK_POINT = .1;//.7


    public void init(HardwareMap hardwareMap){
        ShooterBlocker = hardwareMap.get(Servo.class, "ShooterBlocker");

    }

    public void update(){
    }

    private void SetPosition(double position){
        ShooterBlocker.setPosition(position);
    }

    public void block(){
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
