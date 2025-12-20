package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class SamplePathing extends OpMode {

    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState{
        // StartPos - EndPos
        DRIVE_TO_INTAKE,
        INTAKE_BALLS
    }
    PathState pathState;

    private final Pose startPose = new Pose(17.992, 118.676, Math.toRadians(53));
    private final Pose shootPos = new Pose(59, 85, Math.toRadians(53));

    private PathChain driveStartToShootPos;

    public void buildPaths(){
        // put in coordinates for starting pos > ending pos
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_TO_INTAKE:
                follower.followPath(driveStartToShootPos, true);
                setPathState(PathState.INTAKE_BALLS);
                break;
            case INTAKE_BALLS:
                if (!follower.isBusy()) {

                }
                break;
            default:
                break;
        }
    }

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init(){
        pathState = PathState.DRIVE_TO_INTAKE;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        //Bunch of random telemetry shit he added in, im lazy and it has no function so lets js ignore
    }

}
