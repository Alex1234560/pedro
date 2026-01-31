package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.ParkingServos;


//@Configurable
@TeleOp
public class ParkingTryTeleop extends OpMode
{

    ParkingServos parkingServos = new ParkingServos();
    private double position;
    public static double change_amount=.005;

    @Override
    public void init() {
        position=ParkingServos.START_POINT;
        parkingServos.init(hardwareMap);

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
        double parkingServosGoal = gamepad1.left_trigger;
        if (parkingServosGoal>position){
            position+=change_amount;

        }
        if (parkingServosGoal<position){
            position-=change_amount;

        }

        //parkingServos.SetPosition(position);
        telemetry.addData("Position", parkingServos.getPosition());
        telemetry.update();

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
