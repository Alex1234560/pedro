package org.firstinspires.ftc.teamcode.Mechanisms;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    //private FunctionsAndValues FAndV;
    private DcMotorEx IntakeMotor = null;
    private CRServo IntakeHelperCRServo = null;
    private CRServo IntakeHelperCRServo2 = null;

    public void init(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        IntakeHelperCRServo = hardwareMap.get(CRServo.class, "ServoHelper");
        IntakeHelperCRServo2 = hardwareMap.get(CRServo.class, "ServoHelper2");

        IntakeHelperCRServo.setDirection(CRServo.Direction.REVERSE);
        IntakeHelperCRServo2.setDirection(CRServo.Direction.REVERSE);
    }

    public void start() {

    }

    public void intakeOn(double intake,double helper) {
        IntakeMotor.setPower(-intake);
        IntakeHelperCRServo.setPower(-helper);
        IntakeHelperCRServo2.setPower(-helper);
    }
    public void intakeOff() {
        IntakeMotor.setPower(0);
        IntakeHelperCRServo.setPower(0);
        IntakeHelperCRServo2.setPower(0);
    }


    public void update() {

    }


}
