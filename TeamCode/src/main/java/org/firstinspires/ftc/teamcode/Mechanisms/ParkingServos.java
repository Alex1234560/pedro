package org.firstinspires.ftc.teamcode.Mechanisms;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable

public class ParkingServos {
    private Servo ParkingServo1;
    private Servo ParkingServo2;
    //private static double ShooterAngle = FunctionsAndValues.startPoint;

    public static double START_POINT = 0;
    public static double END_POINT = 1;//.7

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
    }

    public void SetPosition(double position){
        position = normalize(position);
        ParkingServo2.setPosition(position);
        ParkingServo1.setPosition(position);
    }

    public double getPosition(){
        //give position so that when moving manually it can start from the current place.
        return ParkingServo1.getPosition();//position
    }


}
