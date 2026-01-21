package org.firstinspires.ftc.teamcode.Mechanisms;


import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    //private FunctionsAndValues FAndV;
    private DcMotorEx IntakeMotor = null;
    private CRServo IntakeHelperCRServo = null;

    public void init(HardwareMap hardwareMap) {
        IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        IntakeHelperCRServo = hardwareMap.get(CRServo.class, "ServoHelper");

        IntakeHelperCRServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void start() {

    }

    public void intakeOn(double intake,double helper) {
        IntakeMotor.setPower(-intake);
        IntakeHelperCRServo.setPower(-helper);
    }
    public void intakeOff() {
        IntakeMotor.setPower(0);
        IntakeHelperCRServo.setPower(0);
    }

    public void setIntakePower(int intake, int intakeHelper) {
        IntakeMotor.setPower(intake);
        IntakeHelperCRServo.setPower(intakeHelper);
    }

    public void update() {

    }


}
