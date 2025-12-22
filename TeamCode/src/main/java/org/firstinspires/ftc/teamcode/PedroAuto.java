package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class PedroAuto extends OpMode {

    private Follower follower;

    //1 == true, 0 == false
    private double isRed = 1;

    private Timer pathTimer, opModeTimer;

    // -------- FLYWHEEL SETUP -------

    private FlywheelLogic shooter = new FlywheelLogic();
    private Intake intake = new Intake();
    private boolean shotsTriggered=false;


    public enum PathState{
        // StartPos - EndPos
        DRIVE_TO_SHOOT_POS,
        DRIVE_TO_INTAKE_POS,
        INTAKE_BALLS,
        FINISHED
    }
    PathState pathState;

    private double Switcher = 144 * isRed; // if red ==144, else == 0
    private double AngleFlip = 180 *isRed;

    private double xFlip(double oPos){
        //return Switcher + oPos * nFin; Levi
        return Switcher-oPos; // Alex
    }

    private double angleFlip(double oAng) {
        //double DtC = oAng + 90;
        //return Math.toRadians((DtC*negSwitch) - 90); LEVI
        return AngleFlip-oAng;
    }

    /*private final Pose startPose = new Pose(18, 121.2, Math.toRadians(144));
    private final Pose shootPos = new Pose(59, 85, Math.toRadians(144));
    private final Pose intakeStart = new Pose(44.147, 59.348, Math.toRadians(180));
    private final Pose intakeEnd = new Pose(20.662,   59.348, Math.toRadians(180));*/

    //Wrap x in xF, this accounts for left to right swapping. Wrap anlges in aF, this accounts for rotational mirroring and also takes care of the toRad

    private final Pose startPose = new Pose(xFlip(18), 121.2, angleFlip(144));
    private final Pose shootPos = new Pose(xFlip(59), 85, angleFlip(144));
    private final Pose intakeStart = new Pose(xFlip(44.147), 59.348, angleFlip(180));
    private final Pose intakeEnd = new Pose(xFlip(20.662),  59.348, angleFlip(180));


    private PathChain driveStartToShootPos, driveShootPosToIntake, driveIntakeForward;


    public void buildPaths(){
        // put in coordinates for starting pos > ending pos
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading())
                .build();
        driveShootPosToIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, intakeStart))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeStart.getHeading())
                .build();

        driveIntakeForward = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart, intakeEnd))
                .setLinearHeadingInterpolation(intakeStart.getHeading(), intakeEnd.getHeading())
                // Start intake as this path starts
                //.addTemporalCallback(0.0, () -> intake.IntakeOn())

                // Stop intake 2.0 seconds after this path starts
                //.addTemporalCallback(2000, () -> intake.IntakeOff())

                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_TO_SHOOT_POS:
                if (!shotsTriggered){
                    shooter.fireShots(3);
                    shotsTriggered=true;
                }

                follower.followPath(driveStartToShootPos, true);
                setPathState(PathState.DRIVE_TO_INTAKE_POS);

                break;

            case DRIVE_TO_INTAKE_POS:



                if (!follower.isBusy()){
                    //requested shots yet?

                    if (!shooter.isBusy()){

                        follower.followPath(driveShootPosToIntake, true);
                        setPathState(PathState.INTAKE_BALLS);

                    }
                }

                break;

            case INTAKE_BALLS:

                follower.followPath(driveIntakeForward, true);
                setPathState(PathState.FINISHED);


                break;

            case FINISHED:
                break;


            default:
                //intake.intakeOff();
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered=false;
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_TO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //We might want follower = new Follower(hardwareMap);, just check this if it doesnt work

        shooter.init(hardwareMap);
        intake.init(hardwareMap);

        //intake = new Intake(hardwareMap);


        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        shooter.start(); // to start spinning up flywheel from the start
        setPathState(pathState);
    }

    @Override
    public void loop(){
        telemetry.addData("Latest Upload dec 20 6:29 pm", true);


        follower.update();
        shooter.update();
        statePathUpdate();


        //Bunch of random telemetry shit he added in, im lazy and it has no function so lets js ignore

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

        telemetry.update();

    }

}
