package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class PedroAuto extends OpMode {

    private Follower follower;

    //1 == true, 0 == false
    public static boolean IsRed = false;

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

    // make the mirroring a class in the future
//    public static class MirrorSideFunctions {
//        public boolean isRed = false;
//
//        public MirrorSideFunctions(boolean isRed) {
//            this.isRed=isRed;
//        }

    public double xFlip(double oPos, boolean Red){
        double switcher;
        if (Red){switcher=144;
            return switcher-oPos; // Alex
        }
        else{switcher=0;
            return oPos;
        }

    }

    public double angleFlip(double oAng, boolean Red) {
        double flipVal;
        if (Red){flipVal = 180;
            return flipVal-oAng;}
        else{flipVal = 0;
        return oAng;
        }

    }

    /*private final Pose startPose = new Pose(18, 121.2, Math.toRadians(144));
    private final Pose shootPos = new Pose(59, 85, Math.toRadians(144));
    private final Pose intakeStart = new Pose(44.147, 59.348, Math.toRadians(180));
    private final Pose intakeEnd = new Pose(20.662,   59.348, Math.toRadians(180));*/

    //Wrap x in xF, this accounts for left to right swapping. Wrap anlges in aF, this accounts for rotational mirroring and also takes care of the toRad

//    private final Pose startPose = new Pose(xFlip(18, IsRed), 121.2, Math.toRadians(angleFlip(144, IsRed)));
//    private final Pose shootPos = new Pose(xFlip(59, IsRed), 85, Math.toRadians(angleFlip(144, IsRed)));
//    private final Pose intakeStart = new Pose(xFlip(44.147, IsRed), 59.348, Math.toRadians(angleFlip(180, IsRed)));
//    private final Pose intakeEnd = new Pose(xFlip(20.662, IsRed),  59.348, Math.toRadians(angleFlip(180, IsRed)));
    private  Pose startPose;
    private  Pose shootPos;
    private  Pose intakeStart;
    private  Pose intakeEnd;


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
                //.addTemporalCallback(0.0, () -> intake.intakeOn(1,1))

                // Stop intake 2.0 seconds after this path starts
                //.addTemporalCallback(2000, () -> intake.intakeOff())

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

                intake.intakeOn(1,1);

                follower.followPath(driveIntakeForward, true);

                if (!follower.isBusy()){
                    setPathState(PathState.FINISHED);
                }



                break;

            case FINISHED:
                intake.intakeOff();
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



    }

    @Override
    public void init_loop(){
        telemetry.addData("Alliance Selection", "X for BLUE, B for RED, Y for FRONT, A for BACK");
        if (IsRed == false) {
            telemetry.addData("Color: BLUE ", "");
        }
        if (IsRed == true) {
            telemetry.addData("Color: RED ", "");
        }

        if (gamepad1.x || gamepad2.x) {IsRed = false;} // blue
        if (gamepad1.b || gamepad2.b) {IsRed = true;} //red

        telemetry.update();


    }

    @Override
    public void start() {
        buildPoses();

        buildPaths();
        follower.setPose(startPose);
        opModeTimer.resetTimer();
        shooter.start(); // to start spinning up flywheel from the start
        setPathState(pathState);
    }

    @Override
    public void loop(){

        follower.update();
        shooter.update();
        statePathUpdate();

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

        telemetry.update();

    }

    private void buildPoses(){
        startPose = new Pose(xFlip(18, IsRed), 121.2, Math.toRadians(angleFlip(144, IsRed)));
        shootPos = new Pose(xFlip(59, IsRed), 85, Math.toRadians(angleFlip(144, IsRed)));
        intakeStart = new Pose(xFlip(44.147, IsRed), 59.348, Math.toRadians(angleFlip(180, IsRed)));
        intakeEnd = new Pose(xFlip(20.662, IsRed),  59.348, Math.toRadians(angleFlip(180, IsRed)));
    }
}
