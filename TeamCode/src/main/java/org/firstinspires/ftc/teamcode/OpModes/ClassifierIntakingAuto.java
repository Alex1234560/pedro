package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.Coordinates;
import org.firstinspires.ftc.teamcode.Functions.FunctionsAndValues;
import org.firstinspires.ftc.teamcode.Functions.ShootingInterpolation;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelAndFeederLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.HoodAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous
public class ClassifierIntakingAuto extends OpMode {

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
    //private boolean EmptyClassifierOnFirstLine =true;


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
    private static Pose  EmptyClassifierPos;

    //this is to track last pose recorded for TeleOp
    // it is start pose cuz if the code never starts then the last position is the start position :)
    //public static Pose LastPoseRecorded;
    private static Pose ParkPos;

    // ------ these are for use only in this AUTO -------
    private static  Pose shootPos,intakeStart,intakeEnd,shootPosControlPoint, intakeFromClassifierPos;
    private PathChain driveFromIntakeClassifierToShootPos,shootToIntakeClassifier,driveStartToShootPos, driveShootPosToIntake, driveIntakeForward, driveFromIntakeToShootPos,driveFromIntakeEndToClassifier,driveFromClassifierToShootPos;



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

        if (!HasTimeElapsed && pathState== PathState.SHOOT && OutakeBallsOnShoot) {
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
                    if (loop_times==1){
                        follower.followPath(shootToIntakeClassifier, true);
                        setPathState(PathState.DRIVE_BACK_TO_SHOOT);
                    }
                    else {
                        follower.followPath(driveShootPosToIntake, true);
                        setPathState(PathState.INTAKE_BALLS);
                    }


                }
                break;

            case INTAKE_BALLS:

                if (isStateBusy == false &&!follower.isBusy()&&autoFunctions.isRobotInPosition(intakeStart,follower)){

                    follower.followPath(driveIntakeForward, .8,true);
                    isStateBusy = true;
                }


                if (!follower.isBusy() && isStateBusy ==true){
                    isStateBusy = false;
                    setPathState(PathState.DRIVE_BACK_TO_SHOOT);

                }
                break;


            case DRIVE_BACK_TO_SHOOT:

                if ((loop_times == 1 && pathTimer.getElapsedTimeSeconds()>.5)||(loop_times!=1)) {

                    if (isStateBusy == false && !follower.isBusy()) {
                        if (loop_times == 1) {
                            follower.followPath(driveFromIntakeClassifierToShootPos, true);
                        } else {
                            follower.followPath(driveFromIntakeToShootPos, true);
                        }

                        //intake.intakeOn(1,1);
                        isStateBusy = true;
                    }

                    if (isStateBusy == true && !follower.isBusy() && autoFunctions.isRobotInPosition(shootPos, follower)) {
                        isStateBusy = false;
                        setPathState(PathState.SHOOT);

                    }
                }

                break;

            case SHOOT:
                if (isStateBusy==true){}//intake.intakeOn(1,1);}

                //if(isStateBusy == false&&(autoFunctions.isRobotInPosition(shootPos,follower)||pathTimer.getElapsedTimeSeconds()>4)&&!follower.isBusy()){// pathTimer.getElapsedTimeSeconds()>WAIT_TO_SHOOT_TIME){
                if(isStateBusy == false&&!follower.isBusy()){
                    //intake.intakeOn(1,1); // to cycle balls to shooter
                    shooter.fireShots(3);
                    isStateBusy=true;
                    loop_times +=1;
                }

                else if (isStateBusy ==true&&!shooter.isBusy()&&pathTimer.getElapsedTimeSeconds()>1){
                    isStateBusy =false;

                    if (loop_times >= 4) {
                        AutoPark();
                    }
                    else{

                        if (loop_times == 2){
                            ball_line_offset=BallIntakePosition3;
                        }
                        if (loop_times == 3){
                            ball_line_offset=BallIntakePosition1;
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


                    ParkPos = new Pose(Cords.xFlip(36.83312262958282, IsRed), 74.0973451327433, Math.toRadians(Cords.angleFlip(180, IsRed)));
                    Pose currentPose = follower.getPose();



                    driveToParkPath = follower.pathBuilder()
                            .addPath(new BezierLine(currentPose, ParkPos))
                            .setLinearHeadingInterpolation(currentPose.getHeading(), ParkPos.getHeading())
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
        ball_line_offset=BallIntakePosition2;

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

//        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()){EmptyClassifierOnFirstLine=!EmptyClassifierOnFirstLine;}
//        if (EmptyClassifierOnFirstLine == false) {
//            telemetry.addData("Emptying classifier on line 2", "");
//        }
//        if (EmptyClassifierOnFirstLine == true) {
//            telemetry.addData("Emptying classifier on line 1", "");
//        }

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
        turretRotation.update(follower, startPose,IsRed);;
        //turretRotation.handleBearing(camera.getBearing(),camera.getYaw());
        statePathUpdate();

        double[] turretGoals = ShootingInterpolation.get(turretRotation.GetDistanceFromGoal(IsRed));
        hood.SetPosition(turretGoals[0]);
        shooter.setFlywheelTPS(turretGoals[1]);

        //telemetry.addData("Target angle: ", turretRotation.GetTargetAngle());
        //telemetry.addData("Turret Offset", TurretRotation.turret_offset);

        telemetry.addData("Is Robot In Shoot Position", autoFunctions.isRobotInPosition(shootPos,follower));
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

    public void buildPaths() {
        // put in coordinates for starting pos > ending pos
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, shootPosControlPoint, shootPos))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPos.getHeading())
                .build();

        driveShootPosToIntake = buildPath(shootPos, intakeStart);

        driveIntakeForward = buildPath(intakeStart, intakeEnd);

        driveFromIntakeToShootPos = buildPath(intakeEnd, shootPos);

        driveFromClassifierToShootPos = buildPath(EmptyClassifierPos, shootPos);

        driveFromIntakeClassifierToShootPos = buildPath(intakeFromClassifierPos, shootPos);

        driveFromIntakeEndToClassifier = buildPath(intakeEnd, EmptyClassifierPos);

        shootToIntakeClassifier = buildPath(shootPos, intakeFromClassifierPos);
    }


    public PathChain buildPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), Math.toRadians(end.getHeading()))
                .build();
    }

    private void buildPoses(){



        startPose = new Pose(Cords.xFlip(Coordinates.FRONT_START_X, IsRed), Coordinates.FRONT_START_Y, Math.toRadians(Cords.angleFlip(Coordinates.StartingRobotAngleDeg, IsRed)));
        shootPos = new Pose(Cords.xFlip(52, IsRed), 89, Math.toRadians(Cords.angleFlip(180, IsRed)));
        shootPosControlPoint = new Pose(Cords.xFlip(44.28000884955753, IsRed), 109.20315297092289);
        //shootPos180 = new Pose(shootPos.getX(), shootPos.getY(), Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeStart = new Pose(Cords.xFlip(51, IsRed), ball_line_offset, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeEnd = new Pose(Cords.xFlip(17, IsRed),  ball_line_offset, Math.toRadians(Cords.angleFlip(180, IsRed)));


        EmptyClassifierPos = new Pose(Cords.xFlip(15.8, IsRed), 63, Math.toRadians(Cords.angleFlip(180, IsRed)));
        //EmptyClassifierControlPoint = new Pose(Cords.xFlip(24.25,IsRed), 80);


        intakeFromClassifierPos = new Pose(Cords.xFlip(12, IsRed), 60, Math.toRadians(Cords.angleFlip(151, IsRed)));



    }


}