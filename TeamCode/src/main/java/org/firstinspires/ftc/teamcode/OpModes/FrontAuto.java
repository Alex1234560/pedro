package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.Coordinates;
import org.firstinspires.ftc.teamcode.Functions.FunctionsAndValues;
import org.firstinspires.ftc.teamcode.Functions.ShootingInterpolation;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelAndFeederLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.HoodAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@Autonomous
public class FrontAuto extends OpMode {

    private Follower follower;

    //Overarching auto timer
    Timer autoTimer = new Timer();

    //1 == true, 0 == false
    public static boolean IsRed = false;
    private static boolean OutakeBallsOnShoot = false;
    //public static double WAIT_TO_SHOOT_TIME = .6;

    //public static boolean DidAutoGoToEnd;

    public static double PARK_TIME_TRIGGER = 27;

    private Timer pathTimer, opModeTimer;

    //private DistanceSensorClass distanceSensor = new DistanceSensorClass();
    private Coordinates Cords = new Coordinates();
    private AutoFunctions autoFunctions = new AutoFunctions();
    private FlywheelAndFeederLogic shooter = new FlywheelAndFeederLogic();
    private Intake intake = new Intake();
    private TurretRotation turretRotation = new TurretRotation();
    private HoodAngle hood = new HoodAngle();
    //private AprilTagVision camera;

    private FunctionsAndValues FAndV = new FunctionsAndValues();

    private boolean shotsTriggered=false;
    private boolean EmptyClassifierOnFirstLine =true;


    public enum PathState{
        // StartPos - EndPos
        DRIVE_TO_SHOOT_POS,
        DRIVE_TO_INTAKE_POS,
        INTAKE_BALLS,
        CLEAR_CLASSIFIER,
        DRIVE_BACK_TO_SHOOT,
        SHOOT,
        FINISHED,

        AUTOPARK
    }
    PathState pathState;




    //BallLines

    //private final double BALL_LINE_DIFFERENCE = -24;
    private double ball_line_offset;
    private double loop_times;




    private double BallIntakePosition1 = 85;
    private double BallIntakePosition2 = 58;
    private double BallIntakePosition3 = 35.5;





    // -------- everything Poses ---------



    //public static Pose startPose = new Pose(Coordinates.START_X,Coordinates.START_Y,Math.toRadians(Coordinates.StartingRobotAngleDeg));
    public static Pose startPose;
    private static Pose GoalLocationPose,GoalLocationPoseForDistance, EmptyClassifierPos,EmptyClassifierControlPoint;

    //this is to track last pose recorded for TeleOp
    // it is start pose cuz if the code never starts then the last position is the start position :)
    //public static Pose LastPoseRecorded;
    private static Pose driveToPark;

    // ------ these are for use only in this AUTO -------
    private static  Pose shootPos,intakeStart,intakeEnd,shootPosControlPoint;
    private PathChain driveStartToShootPos, driveShootPosToIntake, driveIntakeForward, driveFromIntakeToShootPos,driveFromIntakeEndToClassifier,driveFromClassifierToShootPos;



    private boolean isStateBusy;
    private boolean AutoParkTriggered;



    public void AutoPark(){
        isStateBusy = false;
        AutoParkTriggered = true;
        setPathState(PathState.AUTOPARK);
    }

    public void statePathUpdate() {
        boolean HasTimeElapsed= autoTimer.getElapsedTimeSeconds() > PARK_TIME_TRIGGER;
        if (HasTimeElapsed&& !AutoParkTriggered){
            AutoPark();
        }

        if (!HasTimeElapsed && pathState==PathState.SHOOT && OutakeBallsOnShoot) {
            intake.intakeOn(-1,1);
        }
        else if (!HasTimeElapsed) {
            intake.intakeOn(1,1);
        }
        else{intake.intakeOff();}

        switch(pathState) {
            case DRIVE_TO_SHOOT_POS:
                if(!isStateBusy){
                    follower.followPath(driveStartToShootPos, 1,false);
                    isStateBusy=true;
                }

                if (isStateBusy &&!follower.isBusy()) {
                    // to cycle balls to shooter
                    shooter.fireShots(3); //change to three
                    isStateBusy = false;
                    setPathState(PathState.DRIVE_TO_INTAKE_POS);
                }

                break;

            case DRIVE_TO_INTAKE_POS:

                if (!shooter.isBusy()){
                    //intake.intakeOff();// to stop cycling balls to shooter.
                    follower.followPath(driveShootPosToIntake, true);
                    setPathState(PathState.INTAKE_BALLS);
                }
                break;

            case INTAKE_BALLS:

                //intake.intakeOn(1,1);


                if (isStateBusy == false &&!follower.isBusy()&&autoFunctions.isRobotInPosition(intakeStart,follower)){
                    loop_times +=1;
                    follower.followPath(driveIntakeForward, .8,true);
                    isStateBusy = true;
                }


                if (!follower.isBusy() && isStateBusy ==true){
                    isStateBusy = false;
                    //intake.intakeOff();
//                    if (loop_times ==1&& EmptyClassifierOnFirstLine){
//                        setPathState(PathState.CLEAR_CLASSIFIER);
//                    }
//                    if (loop_times ==2 && !EmptyClassifierOnFirstLine){
//                        setPathState(PathState.CLEAR_CLASSIFIER);
//                    }
                    if (loop_times==1 && EmptyClassifierOnFirstLine){setPathState(PathState.CLEAR_CLASSIFIER);}
                    else if (loop_times==2 && !EmptyClassifierOnFirstLine){setPathState(PathState.CLEAR_CLASSIFIER);}
                    else {
                        setPathState(PathState.DRIVE_BACK_TO_SHOOT);
                    }
                }
                break;

            case CLEAR_CLASSIFIER:
                if (isStateBusy == false &&!follower.isBusy()){

                    follower.followPath(driveFromIntakeEndToClassifier, 1,true);
                    isStateBusy = true;
                }

                if (pathTimer.getElapsedTimeSeconds()>2 && isStateBusy ==true ){

                    isStateBusy = false;
                    setPathState(PathState.DRIVE_BACK_TO_SHOOT);

                }
                break;


            case DRIVE_BACK_TO_SHOOT:
                if(isStateBusy == false && !follower.isBusy()){
                    if (loop_times == 1 && EmptyClassifierOnFirstLine){
                        follower.followPath(driveFromClassifierToShootPos, true);
                    }
                    else if (loop_times == 2 && !EmptyClassifierOnFirstLine){
                        follower.followPath(driveFromClassifierToShootPos, true);
                    }
                    else{follower.followPath(driveFromIntakeToShootPos, true);}

                    //intake.intakeOn(1,1);
                    isStateBusy = true;
                }

                if (isStateBusy ==true&&!follower.isBusy()&&autoFunctions.isRobotInPosition(shootPos,follower)){
                    isStateBusy =false;
                    setPathState(PathState.SHOOT);
                }

                break;

            case SHOOT:
                if (isStateBusy==true){}//intake.intakeOn(1,1);}

                //if(isStateBusy == false&&(autoFunctions.isRobotInPosition(shootPos,follower)||pathTimer.getElapsedTimeSeconds()>4)&&!follower.isBusy()){// pathTimer.getElapsedTimeSeconds()>WAIT_TO_SHOOT_TIME){
                if(isStateBusy == false&&!follower.isBusy()){
                    //intake.intakeOn(1,1); // to cycle balls to shooter
                    shooter.fireShots(3);
                    isStateBusy=true;
                }

                else if (isStateBusy ==true&&!shooter.isBusy()&&pathTimer.getElapsedTimeSeconds()>1){
                    isStateBusy =false;

                    if (loop_times >= 3) {
                        AutoPark();
                    }
                    else{
//                        if (ball_line_offset==0) {//if first time running
//                        }
                        if (loop_times == 1){
                            ball_line_offset=BallIntakePosition2;
                        }
                        if (loop_times == 2){
                            ball_line_offset=BallIntakePosition3;
                        }

                        buildPoses();
                        buildPaths();
                        setPathState(PathState.DRIVE_TO_INTAKE_POS);
                    }
                    //intake.intakeOff();
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


                    driveToPark = new Pose(Cords.xFlip(36.83312262958282, IsRed), 74.0973451327433, Math.toRadians(Cords.angleFlip(180, IsRed)));
                    Pose currentPose = follower.getPose();



                    driveToParkPath = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, driveToPark))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), driveToPark.getHeading())
                            .build();

                    follower.followPath(driveToParkPath, true);

                    isStateBusy = true;
                }
                else{
                    if (turretRotation.isTurretFinishedRotating()) {
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
        ball_line_offset=BallIntakePosition1;
        loop_times = 0;


        pathState = PathState.DRIVE_TO_SHOOT_POS;
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
        telemetry.addData("Alliance Selection", "X for BLUE, B for RED, Y for FRONT, A for BACK");
        if (IsRed == false) {
            telemetry.addData("Color: BLUE ", "");
        }
        if (IsRed == true) {
            telemetry.addData("Color: RED ", "");
        }

        if (gamepad1.x || gamepad2.x) {IsRed = false;} // blue
        if (gamepad1.b || gamepad2.b) {IsRed = true;} //red

        telemetry.addData("Ball Line Classifier Activation", "Y to switch");

        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()){EmptyClassifierOnFirstLine=!EmptyClassifierOnFirstLine;}
        if (EmptyClassifierOnFirstLine == false) {
            telemetry.addData("Emptying classifier on line 2", "");
        }
        if (EmptyClassifierOnFirstLine == true) {
            telemetry.addData("Emptying classifier on line 1", "");
        }

        telemetry.addData("Outake When Shooting?", "A to switch");

        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()){OutakeBallsOnShoot=!OutakeBallsOnShoot;}

        telemetry.addData("Outake balls On Shoot: ", OutakeBallsOnShoot);





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

        //distanceSensor.update();
        //shooter.updateDistanceSensorValueForAuto(distanceSensor.IsBallDetected());
        //camera.update();
        follower.update();

        boolean IsTurretReady = turretRotation.isTurretFinishedRotating();//autoFunctions.isRobotInPosition(shootPos,follower) &&
        shooter.updateWithStateMachine(IsTurretReady);
        turretRotation.update(follower, startPose,IsRed);
        //turretRotation.handleBearing(camera.getBearing(),camera.getYaw());
        statePathUpdate();

        double[] turretGoals = ShootingInterpolation.get(turretRotation.GetDistanceFromGoal(IsRed));
        hood.SetPosition(turretGoals[0]);
        shooter.setFlywheelTPS(turretGoals[1]);

        //telemetry.addData("Target angle: ", turretRotation.GetTargetAngle());
        //telemetry.addData("Turret Offset", TurretRotation.turret_offset);

        telemetry.addData("Is Robot In Shoot Position", autoFunctions.isRobotInPosition(shootPos,follower));
        telemetry.addData("PathTimer", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("Is Shooter Busy?", shooter.isBusy());
        telemetry.addData("Balls Shot", shooter.GetBallsShotCount());
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("Shooter Path State", FlywheelAndFeederLogic.flywheelState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("total heading", Math.toDegrees(follower.getTotalHeading()));
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Total time", autoTimer.getElapsedTimeSeconds());


        telemetry.update();
    }

    public void buildPaths(){
        // put in coordinates for starting pos > ending pos
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose,shootPosControlPoint, shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading())
                .build();

        driveShootPosToIntake = follower.pathBuilder()
                .addPath(new BezierLine(shootPos, intakeStart))
                .setLinearHeadingInterpolation(shootPos.getHeading(), intakeStart.getHeading())
                .build();

        driveIntakeForward = follower.pathBuilder()
                .addPath(new BezierLine(intakeStart, intakeEnd))
                .setLinearHeadingInterpolation(intakeStart.getHeading(), intakeEnd.getHeading())
                .build();

        driveFromIntakeToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd, shootPos))
                .setLinearHeadingInterpolation(intakeEnd.getHeading(), shootPos.getHeading())
                .build();

        driveFromClassifierToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(EmptyClassifierPos, shootPos))
                .setLinearHeadingInterpolation(EmptyClassifierPos.getHeading(), EmptyClassifierPos.getHeading())
                .build();

        driveFromIntakeEndToClassifier = follower.pathBuilder()
                .addPath(new BezierCurve(
                        intakeEnd,
                        EmptyClassifierControlPoint,
                        EmptyClassifierPos
                ))

                .setLinearHeadingInterpolation(intakeEnd.getHeading(), EmptyClassifierPos.getHeading())
                .build();
    }

    private void buildPoses(){



        startPose = new Pose(Cords.xFlip(Coordinates.FRONT_START_X, IsRed), Coordinates.FRONT_START_Y, Math.toRadians(Cords.angleFlip(Coordinates.StartingRobotAngleDeg, IsRed)));
        shootPos = new Pose(Cords.xFlip(52, IsRed), 89, Math.toRadians(Cords.angleFlip(180, IsRed)));
        shootPosControlPoint = new Pose(Cords.xFlip(44.28000884955753, IsRed), 109.20315297092289);
        //shootPos180 = new Pose(shootPos.getX(), shootPos.getY(), Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeStart = new Pose(Cords.xFlip(51, IsRed), ball_line_offset, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeEnd = new Pose(Cords.xFlip(17, IsRed),  ball_line_offset, Math.toRadians(Cords.angleFlip(180, IsRed)));

        if (EmptyClassifierOnFirstLine){
            EmptyClassifierPos = new Pose(Cords.xFlip(15, IsRed), 76.4740834386852, Math.toRadians(Cords.angleFlip(90, IsRed)));
            EmptyClassifierControlPoint = new Pose(Cords.xFlip(24.25,IsRed), 80);
        }
        else{
            EmptyClassifierPos = new Pose(Cords.xFlip(15, IsRed), 76.4740834386852, Math.toRadians(Cords.angleFlip(270, IsRed)));
            EmptyClassifierControlPoint = new Pose(Cords.xFlip(22.611567635903917,IsRed), 60.82806573957017);
        }


        GoalLocationPose = new Pose(Cords.xFlip(Coordinates.GOAL_X,IsRed), Coordinates.GOAL_Y, Math.toRadians(0));
        GoalLocationPoseForDistance = new Pose(Cords.xFlip(Coordinates.GOAL_X_FOR_CAMERA,IsRed), Coordinates.GOAL_Y_FOR_CAMERA, Math.toRadians(0));




    }


}