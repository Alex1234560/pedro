package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.ShooterBlocker;

@Disabled
//@Configurable
@TeleOp
public class BlockerShooterTryTeleop extends OpMode
{

    ShooterBlocker shooterBlocker = new ShooterBlocker();
//    private double position;
//    public static double change_amount=.005;

    @Override
    public void init() {
        shooterBlocker.init(hardwareMap);

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


        if (gamepad1.aWasPressed()){
            shooterBlocker.Unblock();
        }
        if (gamepad1.bWasPressed()){
            shooterBlocker.Unblock();
        }


        //parkingServos.SetPosition(position);
        telemetry.addData("Position", shooterBlocker.getPosition());
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
