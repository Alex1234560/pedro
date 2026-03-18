package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.DistanceSensorClass;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterBlocker;


//@Configurable
@TeleOp
public class DistanceSensorTunerTeleop extends OpMode
{

    private TelemetryManager telemetryM;
    DistanceSensorClass DistanceSensor = new DistanceSensorClass();
//    private double position;
//    public static double change_amount=.005;

    @Override
    public void init() {
        DistanceSensor.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
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


//        if (gamepad1.aWasPressed()){
//            shooterBlocker.Unblock();
//        }
//        if (gamepad1.bWasPressed()){
//            shooterBlocker.Block();
//        }

        DistanceSensor.update();

        telemetryM.addData("Is Ball Detected", DistanceSensor.IsBallDetected());
        telemetryM.addData("Distance", DistanceSensor.GetDistance());
        telemetryM.update(telemetry);



    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

}
