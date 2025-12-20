/*package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;

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

    public void biuldPaths(){
        // put in cordinates for starting pos > ending pos
        driveStartToShootPos = follower.pathBuilder()
    }

    @Override
    public void init(){


    }
    @Override
    public void loop(){

    }

}
*/