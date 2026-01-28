package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Disabled
@Configurable
@TeleOp
public class TestingWheelConfiguration extends OpMode
{
    private DcMotor leftBack = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor rightFront = null;


    @Override
    public void init() {
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront.setDirection(DcMotor.Direction.FORWARD);

//       .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
//                .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
//                .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
//                .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {

    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.a){rightBack.setPower(1);}else{rightBack.setPower(0);}
        if (gamepad1.x){leftBack.setPower(1);}else{leftBack.setPower(0);}
        if (gamepad1.y){leftFront.setPower(1);}else{leftFront.setPower(0);}
        if (gamepad1.b){rightFront.setPower(1);}else{rightFront.setPower(0);}


    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
