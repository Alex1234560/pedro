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
    private static double ShooterAngle = FunctionsAndValues.startPoint;

    private static double START_POINT = .15;
    private static double END_POINT = .9;//.7

    public void init(HardwareMap hardwareMap){
        ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");

    }





}
