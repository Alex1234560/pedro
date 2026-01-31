package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable

public class ParkingServos {
    private Servo ParkingServo1;
    private Servo ParkingServo2;
    //private static double ShooterAngle = FunctionsAndValues.startPoint;

    public static double START_POINT = .2;
    public static double END_POINT = .8;//.7

    private double position;
    private double goal_position;
    public static double change_amount=.005;

    public double normalize(double value){
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
        ParkingServo1 = hardwareMap.get(Servo.class, "Parking1");
        ParkingServo2 = hardwareMap.get(Servo.class, "Parking2");
        position=getPosition();
        goal_position=getPosition();
    }

    public void update(){
//        if (goal_position>position){
//            position+=change_amount;
//        }
//        if (goal_position<position){
//            position-=change_amount;
//        }
//
//        SetPosition(position);
    }

    private void SetPosition(double position){
        position = normalize(position);
        ParkingServo2.setPosition(position);
        ParkingServo1.setPosition(position);
    }

    public void setToMaxPosition(){
       //goal_position=END_POINT;
        if(getPosition()!=END_POINT){SetPosition(END_POINT);}
    }
    public void setToMinPosition(){
        //goal_position=START_POINT;
        if(getPosition()!=START_POINT){SetPosition(START_POINT);}
    }

    public double getPosition(){
        //give position so that when moving manually it can start from the current place.
        return ParkingServo1.getPosition();//position
    }


}
