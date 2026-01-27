package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.Coordinates;
import org.firstinspires.ftc.teamcode.Functions.FunctionsAndValues;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@Autonomous
public class BackAuto extends OpMode {

    private Follower follower;

    //Overarching auto timer
    Timer autoTimer = new Timer();

    //1 == true, 0 == false
    public static boolean IsRed = false;
    public static boolean GrabFromSpikeMark = true;


    //public static boolean DidAutoGoToEnd;

    public static double PARK_TIME_TRIGGER = 27;

    private Timer pathTimer, opModeTimer;

    //private DistanceSensorClass distanceSensor = new DistanceSensorClass();
    private Coordinates Cords = new Coordinates();
    private AutoFunctions autoFunctions = new AutoFunctions();
    private ShooterLogic shooter = new ShooterLogic();
    private Intake intake = new Intake();
    private TurretRotation turretRotation = new TurretRotation();
    private ShooterAngle hood = new ShooterAngle();
    //private AprilTagVision camera;

    private FunctionsAndValues FAndV = new FunctionsAndValues();

    private boolean shotsTriggered=false;


    public enum PathState{
        // StartPos - EndPos
        DRIVE_TO_SHOOT_POS_FROM_START,
        SHOOT,
        DRIVE_TO_INTAKE_POS,
        INTAKE_BALLS,
        DRIVE_BACK_TO_SHOOT,
        FINISHED,

        AUTOPARK
    }
    PathState pathState;




    //BallLines

    private double ball_line_offset;
    private int loop_times;

    // -------- everything Poses ---------



    //public static Pose startPose = new Pose(Coordinates.START_X,Coordinates.START_Y,Math.toRadians(Coordinates.StartingRobotAngleDeg));
    public static Pose startPose;
    private static Pose GoalLocationPose,GoalLocationPoseForDistance;

    //this is to track last pose recorded for TeleOp
    // it is start pose cuz if the code never starts then the last position is the start position :)
    //public static Pose LastPoseRecorded;
    private static Pose driveToPark;

    // ------ these are for use only in this AUTO -------
    private static  Pose shootPos,intakeStart,intakeEnd, intakeSpikeMarkStart, intakeSpikeMarkEnd;
    private PathChain driveFromBallsIntakeSpikeMarkBallsToShootPos, driveIntakeForwardForSpikeMarkBalls, driveShootPosToBallsIntakeSpikeMarkBalls,driveStartToShootPos, driveShootPosToIntakeCornerBalls, driveIntakeForwardForCornerBalls, driveFromIntakeToShootPosCornerBalls;

    PathChain[] IntakeToShootPaths;
    PathChain[] IntakeBallsPaths;
    PathChain[] ShootPosToIntakePosPaths;

    private boolean isStateBusy;
    private boolean AutoParkTriggered;



    public void AutoPark(){
        isStateBusy = false;
        AutoParkTriggered = true;
        setPathState(PathState.AUTOPARK);
    }

    public void statePathUpdate() {
        if (autoTimer.getElapsedTimeSeconds() > PARK_TIME_TRIGGER && !AutoParkTriggered){
            AutoPark();
        }
        if (!AutoParkTriggered) {
            intake.intakeOn(1, 1);
        }
        // only spitting out balls if theres already a ball ready to shoot and is in shoot state.
        else if (pathState==PathState.SHOOT&&shooter.IsBallDetected()){ intake.intakeOn(-1, 1);}
        else{intake.intakeOff();}

        switch(pathState) {
            case DRIVE_TO_SHOOT_POS_FROM_START:
                if(isStateBusy == false){
                    follower.followPath(driveStartToShootPos, true);
                    isStateBusy=true;
                }
                else if (isStateBusy == true && !follower.isBusy()){
                    isStateBusy=false;
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                if(isStateBusy == false&&autoFunctions.isRobotInPosition(shootPos,follower) ){
                    //intake.intakeOn(1,1); // to cycle balls to shooter
                    shooter.fireShots(3);
                    isStateBusy=true;
                }

                if (isStateBusy ==true&&!shooter.isBusy()){
                    setPathState(PathState.DRIVE_TO_INTAKE_POS);
                    //intake.intakeOff();
                    isStateBusy=false;
                }
                break;

            case DRIVE_TO_INTAKE_POS:

                if (!shooter.isBusy() ){
                    follower.followPath(ShootPosToIntakePosPaths[loop_times], true);

                    setPathState(PathState.INTAKE_BALLS);

                }

                break;

            case INTAKE_BALLS:

                if (!follower.isBusy() && !isStateBusy){
                    follower.followPath(IntakeBallsPaths[loop_times],.7, true);

                    isStateBusy = true;
                }
                else if (isStateBusy && !follower.isBusy()){
                    isStateBusy = false;
                    //intake.intakeOff();
                    setPathState(PathState.DRIVE_BACK_TO_SHOOT);
                }


                break;
            case DRIVE_BACK_TO_SHOOT:
                if(isStateBusy == false){
                    follower.followPath(IntakeToShootPaths[loop_times], true);

                    isStateBusy = true;
                }
                else if (isStateBusy == true && !follower.isBusy()){
                    isStateBusy = false;
                    if (loop_times<1){
                        loop_times+=1;}
                    setPathState(PathState.SHOOT);
                }

                break;

            case FINISHED:
                AutoFunctions.DidAutoGoToEnd = true;
                break;

            case AUTOPARK:
                PathChain driveToParkPath = null;
                if (isStateBusy==false) {
                    turretRotation.TurretTo0Deg(true);
                    //intake.intakeOff();
                    shooter.Off();

                    follower.breakFollowing();
                    //follower.setPose(follower.getPose());


                    driveToPark = new Pose(Cords.xFlip(44, IsRed), 29, Math.toRadians(Cords.angleFlip(180, IsRed)));
                    Pose currentPose = follower.getPose();



                    driveToParkPath = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, driveToPark))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), driveToPark.getHeading())
                            .build();

                    follower.followPath(driveToParkPath, true);

                    isStateBusy = true;
                }
                else{
                    if (turretRotation.isTurretFinishedRotating()&&!follower.isBusy()) {
                        isStateBusy = false;
                        setPathState(PathState.FINISHED);
                    }
                }

                break;

            default:
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
        //resseting variables
        isStateBusy=false;
        AutoParkTriggered = false;
        ball_line_offset=0;

        pathState = PathState.DRIVE_TO_SHOOT_POS_FROM_START;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //We might want follower = new Follower(hardwareMap);, just check this if it doesnt work
        //distanceSensor.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        turretRotation.init(hardwareMap);
        hood.init(hardwareMap);
        //camera = new AprilTagVision(hardwareMap);
        //intake = new Intake(hardwareMap);

    }

    @Override
    public void init_loop(){
        telemetry.addData("Alliance Selection", "X for BLUE, B for RED");
        if (IsRed == false) {
            telemetry.addData("Color: BLUE ", "");
        }
        if (IsRed == true) {
            telemetry.addData("Color: RED ", "");
        }



        if (gamepad1.x || gamepad2.x) {IsRed = false;} // blue
        if (gamepad1.b || gamepad2.b) {IsRed = true;} //red

        telemetry.addData("Back Auto Selection", "Y to toggle");
        telemetry.addData(" Grab From Spike Mark: ", GrabFromSpikeMark);

        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {GrabFromSpikeMark = !GrabFromSpikeMark;} // blue


        if (GrabFromSpikeMark){loop_times = 0;}
        else{loop_times = 1;}

        telemetry.update();


    }

    @Override
    public void start() {
        AutoFunctions.IsRed = IsRed;
        AutoFunctions.DidAutoGoToEnd = false;
        autoTimer.resetTimer();
        buildPoses();
        turretRotation.CalibrateTurretToCenter();
        buildPaths();
        follower.setPose(startPose);
        opModeTimer.resetTimer();
        shooter.start(); // to start spinning up flywheel from the start
        setPathState(pathState);
    }

    @Override
    public void loop(){
        AutoFunctions.LastPoseRecorded = follower.getPose();

        double DistanceFromGoal = turretRotation.GetDistanceFromGoal(GoalLocationPoseForDistance);

        //distanceSensor.update();
        //shooter.updateDistanceSensorValueForAuto(distanceSensor.IsBallDetected());
        //camera.update();
        follower.update();
        boolean IsTurretReady = autoFunctions.isRobotInPosition(shootPos,follower) && turretRotation.isTurretFinishedRotating();
        shooter.updateWithStateMachine(IsTurretReady);
        turretRotation.update(follower, GoalLocationPose, startPose,IsRed);;
        //turretRotation.handleBearing(camera.getBearing(),camera.getYaw());
        statePathUpdate();

        double[] turretGoals = FAndV.handleShootingRanges(DistanceFromGoal- FunctionsAndValues.OffsetForShootingAlgorithmRemoveLater);
        hood.SetPosition(turretGoals[0]);
        shooter.setFlywheelTPS(turretGoals[1]);

        //turret.handleBearing(camera.getBearing());
        //telemetry.addData("Target angle: ", turretRotation.GetTargetAngle());
        //telemetry.addData("Turret Offset", TurretRotation.turret_offset);
        telemetry.addData("Is Shooter Busy?", shooter.isBusy());
        telemetry.addData("Balls Shot", shooter.GetBallsShotCount());
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("total heading", Math.toDegrees(follower.getTotalHeading()));
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Total time", autoTimer.getElapsedTimeSeconds());

        telemetry.update();
    }

    public void buildPaths(){
        IntakeToShootPaths = new PathChain[2];
        IntakeBallsPaths = new PathChain[2];
        ShootPosToIntakePosPaths = new PathChain[2];

        // put in coordinates for starting pos > ending pos
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading())
                .build();
        driveShootPosToIntakeCornerBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, intakeStart))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeStart.getHeading())
                .build();

        driveIntakeForwardForCornerBalls = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart, intakeEnd))
                .setLinearHeadingInterpolation(intakeStart.getHeading(), intakeEnd.getHeading())
                .build();

        driveFromIntakeToShootPosCornerBalls = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd, shootPos))
                .setLinearHeadingInterpolation(intakeEnd.getHeading(), shootPos.getHeading())
                .build();

        driveShootPosToBallsIntakeSpikeMarkBalls = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, intakeSpikeMarkStart))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeSpikeMarkStart.getHeading())
                .build();

        driveIntakeForwardForSpikeMarkBalls = follower.pathBuilder()
                .addPath(new BezierLine(intakeSpikeMarkStart, intakeSpikeMarkEnd))
                .setLinearHeadingInterpolation(intakeSpikeMarkStart.getHeading(), intakeSpikeMarkEnd.getHeading())
                .build();

        driveFromBallsIntakeSpikeMarkBallsToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeSpikeMarkEnd, shootPos))
                .setLinearHeadingInterpolation(intakeSpikeMarkEnd.getHeading(), shootPos.getHeading())
                .build();


        IntakeToShootPaths[0] =driveFromBallsIntakeSpikeMarkBallsToShootPos;
        IntakeToShootPaths[1] = driveFromIntakeToShootPosCornerBalls;

        IntakeBallsPaths[0] = driveIntakeForwardForSpikeMarkBalls;
        IntakeBallsPaths[1] = driveIntakeForwardForCornerBalls;

        ShootPosToIntakePosPaths[0] = driveShootPosToBallsIntakeSpikeMarkBalls;
        ShootPosToIntakePosPaths[1] = driveShootPosToIntakeCornerBalls;



    }

    private void buildPoses(){
        startPose = new Pose(Cords.xFlip(62.47408343868521, IsRed), 9.787610619469017, Math.toRadians(Cords.angleFlip(180, IsRed)));
        shootPos = new Pose(Cords.xFlip(57, IsRed), 12, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeStart = new Pose(Cords.xFlip(33.89254108723136, IsRed), 12.336283185840703, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeEnd = new Pose(Cords.xFlip(11, IsRed),  12.336, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeSpikeMarkStart = new Pose(Cords.xFlip(47.5, IsRed), 35, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeSpikeMarkEnd = new Pose(Cords.xFlip(13.530973451327434, IsRed),  35, Math.toRadians(Cords.angleFlip(180, IsRed)));

        GoalLocationPose = new Pose(Cords.xFlip(Coordinates.GOAL_X,IsRed), Coordinates.GOAL_Y, Math.toRadians(0));
        GoalLocationPoseForDistance = new Pose(Cords.xFlip(Coordinates.GOAL_X_FOR_DISTANCE,IsRed), Coordinates.GOAL_Y_FOR_DISTANCE, Math.toRadians(0));
    }
}