package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
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
import org.firstinspires.ftc.teamcode.Mechanisms.HoodAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@Autonomous
public class NewAutoBack extends OpMode {

    private Follower follower;

    //Overarching auto timer
    Timer autoTimer = new Timer();


    public static boolean IsRed = false;
    private static boolean OutakeBallsOnShoot;
    private static boolean GrabFromTunnel;
    public static double PARK_TIME_TRIGGER = 27;

    private double OVERRIDE_TIME_IF_INTAKING_FOR_TOO_LONG = 1.5;

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
        PRE_INTAKE_BALLS,
        BACK_TO_INTAKE_START,
        INTAKE_BALLS,
        CLEAR_CLASSIFIER,
        DRIVE_BACK_TO_SHOOT,
        SHOOT,
        FINISHED,

        AUTOPARK
    }
    PathState pathState;

    private double loop_times;
    private boolean GrabFromSpikeMark;
    private boolean IsRobotBusy = false;

    // -------- everything Poses ---------

    //public static Pose startPose = new Pose(Coordinates.START_X,Coordinates.START_Y,Math.toRadians(Coordinates.StartingRobotAngleDeg));
    public static Pose startPose;
    private static Pose  EmptyClassifierPos;

    //this is to track last pose recorded for TeleOp
    // it is start pose cuz if the code never starts then the last position is the start position :)
    //public static Pose LastPoseRecorded;
    private static Pose ParkPos;

    // ------ these are for use only in this AUTO -------
    private static  Pose intakeStart, intakeEnd; //
    private static  Pose intakeCornerStart, intakeCornerEnd,intakeTunnelStart,intakeTunnelEnd; //
    private static  Pose shootPos;
    private static  Pose intakeSpikeMarkStart,intakeSpikeMarkEnd;
    private PathChain driveStartToShootPos, driveShootPosToIntake, driveIntakeForward, driveFromIntakeToShootPos,driveIntakeBackward;

    private boolean isStateBusy;
    private boolean AutoParkTriggered;

    private String PickupLocation;

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

                if (isStateBusy &&!IsRobotBusy) {
                    shooter.fireShots(3); //change to three
                    isStateBusy = false;
                    setPathState(PathState.DRIVE_TO_INTAKE_POS);
                }

                break;

            case DRIVE_TO_INTAKE_POS:

                if (!IsRobotBusy && isStateBusy==false) {
                    follower.followPath(driveShootPosToIntake, true);
                    isStateBusy=true;
                }


                if (!IsRobotBusy && isStateBusy==true) {
                    isStateBusy=false;
                    if(PickupLocation=="CORNER")
                    {setPathState(PathState.PRE_INTAKE_BALLS);}
                    //{setPathState(PathState.PRE_INTAKE_BALLS);}
                    else
                    {setPathState(PathState.INTAKE_BALLS);}
                }

                break;

            case PRE_INTAKE_BALLS:
                if (isStateBusy == false && !IsRobotBusy){
                    follower.followPath(driveIntakeForward, .8,true);
                    isStateBusy = true;
                }

                if (( pathTimer.getElapsedTimeSeconds()>1.5 )&&(!IsRobotBusy||autoFunctions.isRobotInPositionCustomAmounts(intakeEnd,follower,0,10) && isStateBusy ==true)){
                    isStateBusy = false;
                    setPathState(PathState.BACK_TO_INTAKE_START);
                }

                else if (pathTimer.getElapsedTimeSeconds()>OVERRIDE_TIME_IF_INTAKING_FOR_TOO_LONG){
                    follower.breakFollowing();
                    isStateBusy = false;
                    setPathState(PathState.BACK_TO_INTAKE_START);
                }


                break;
            case BACK_TO_INTAKE_START:
                if (isStateBusy == false){
                    follower.followPath(driveIntakeBackward, .8,true);
                    isStateBusy = true;
                }

                if (!IsRobotBusy && isStateBusy ==true){
                    isStateBusy = false;
                    setPathState(PathState.INTAKE_BALLS);
                }
                break;

            case INTAKE_BALLS:

                if (isStateBusy == false && AutoFunctions.isRobotInPositionCustomAmounts(intakeStart,follower,100,6)){
                    follower.followPath(driveIntakeForward, .8,false);
                    isStateBusy = true;
                }

                if ( pathTimer.getElapsedTimeSeconds()>.5&&!IsRobotBusy && isStateBusy ==true&&autoFunctions.isRobotInPositionCustomAmounts(intakeEnd,follower,.2,15)){//pathTimer.getElapsedTimeSeconds()>1.5){
                    isStateBusy = false;
                    setPathState(PathState.DRIVE_BACK_TO_SHOOT);
                }

                else if (pathTimer.getElapsedTimeSeconds()>OVERRIDE_TIME_IF_INTAKING_FOR_TOO_LONG){
                    follower.breakFollowing();
                    isStateBusy = false;
                    setPathState(PathState.DRIVE_BACK_TO_SHOOT);
                }

                break;


            case DRIVE_BACK_TO_SHOOT:

                if (isStateBusy == false) {//&& (shooter.IsBallDetected()||pathTimer.getElapsedTimeSeconds()>3)
                    follower.followPath(driveFromIntakeToShootPos, true);
                    isStateBusy = true;
                }

                if (isStateBusy == true && !IsRobotBusy) {
                    isStateBusy = false;
                    setPathState(PathState.SHOOT);
                }

                break;

            case SHOOT:

                //if(isStateBusy == false&&(autoFunctions.isRobotInPosition(shootPos,follower)||pathTimer.getElapsedTimeSeconds()>4)&&!follower.isBusy()){// pathTimer.getElapsedTimeSeconds()>WAIT_TO_SHOOT_TIME){
                if(isStateBusy == false&&!IsRobotBusy){
                    //intake.intakeOn(1,1); // to cycle balls to shooter
                    shooter.fireShots(3);
                    isStateBusy=true;
                    loop_times +=1;
                }

                else if (isStateBusy ==true&&!IsRobotBusy&&pathTimer.getElapsedTimeSeconds()>1){
                    isStateBusy =false;



                        //buildPoses();

                        if (GrabFromSpikeMark) {

                            if (loop_times == 1) {
                                PickupLocation = "SPIKE_MARK";
                                intakeStart = intakeSpikeMarkStart;
                                intakeEnd = intakeSpikeMarkEnd;
                            }
                            if ((loop_times == 2 || loop_times == 4)&&GrabFromTunnel) {
                                PickupLocation = "TUNNEL";
                                intakeStart = intakeTunnelStart;
                                intakeEnd = intakeTunnelEnd;
//                            intakeStart=intakeSpikeMarkStart;
//                            intakeEnd=intakeSpikeMarkEnd;
                            }
                            if ((loop_times == 3 || loop_times == 5)||(!GrabFromTunnel && loop_times!=1)) {
                                PickupLocation = "CORNER";
                                intakeStart = intakeCornerStart;
                                intakeEnd = intakeCornerEnd;
                                //
                            }
                        }


                        buildPaths();
                        setPathState(PathState.DRIVE_TO_INTAKE_POS);

                }
                break;

            case FINISHED:
                AutoFunctions.DidAutoGoToEnd = true;
                break;

            case AUTOPARK:
                PathChain driveToParkPath = null;
                if (isStateBusy==false) {
                    turretRotation.TurretTo0Deg(true);
                    shooter.Off();

                    follower.breakFollowing();

                    ParkPos = new Pose(Cords.xFlip(44, IsRed), 29, Math.toRadians(Cords.angleFlip(180, IsRed)));
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
        GrabFromSpikeMark=true;
        GrabFromTunnel=true;
        OutakeBallsOnShoot = true;
        isStateBusy=false;
        AutoParkTriggered = false;
        loop_times = 0;

        pathState = PathState.DRIVE_TO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
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

        telemetry.addData(" Grab From Spike Mark (Y to toggle): ", GrabFromSpikeMark);
        if (gamepad1.yWasPressed() || gamepad2.yWasPressed()) {GrabFromSpikeMark = !GrabFromSpikeMark;} // blue

        telemetry.addData(" Grab From Tunnel (A to toggle): ", GrabFromTunnel);
        if (gamepad1.aWasPressed() || gamepad2.aWasPressed()) {GrabFromTunnel = !GrabFromTunnel;} // blue

        //telemetry.addData("Outake When Shooting?", "A to switch");
        //if (gamepad1.aWasPressed() || gamepad2.aWasPressed()){OutakeBallsOnShoot=!OutakeBallsOnShoot;}
        //telemetry.addData("Outake balls On Shoot: ", OutakeBallsOnShoot);

        telemetry.update();

    }

    @Override
    public void start() {
        AutoFunctions.IsRed = IsRed;
        AutoFunctions.DidAutoGoToEnd = false;
        autoTimer.resetTimer();

        buildPoses();
        intakeStart =intakeCornerStart;
        intakeEnd =intakeCornerEnd;
        PickupLocation = "CORNER";

        turretRotation.CalibrateTurretToCenter();
        buildPaths();
        follower.setPose(startPose);
        opModeTimer.resetTimer();
        shooter.start(); // to start spinning up flywheel from the start
        setPathState(pathState);



    }

    @Override
    public void loop() {

        if (follower.isBusy()||shooter.isBusy()){IsRobotBusy = true;}
        else{IsRobotBusy=false;}


        AutoFunctions.LastPoseRecorded = follower.getPose();

        follower.update();

        boolean IsTurretReady = turretRotation.isTurretFinishedRotating();//autoFunctions.isRobotInPosition(shootPos,follower) &&
        shooter.updateWithStateMachine(IsTurretReady);
        turretRotation.update(follower, startPose,IsRed);;
        statePathUpdate();

        double[] turretGoals = ShootingInterpolation.get(turretRotation.GetDistanceFromGoal(IsRed));
        hood.SetPosition(turretGoals[0]);
        shooter.setFlywheelTPS(turretGoals[1]);


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

        driveStartToShootPos = buildPath(intakeEnd, shootPos);

        driveIntakeForward= buildPath(intakeStart, intakeEnd);

        driveIntakeBackward = buildPath(intakeEnd, intakeStart);

        driveShootPosToIntake = buildPath(shootPos, intakeStart);

        driveFromIntakeToShootPos = buildPath(intakeEnd, shootPos);
    }


    public PathChain buildPath(Pose start, Pose end) {
        return follower.pathBuilder()
                .addPath(new BezierLine(start, end))
                .setLinearHeadingInterpolation(start.getHeading(), end.getHeading())
                .build();
    }

    private void buildPoses(){

        startPose = new Pose(Cords.xFlip(62.55, IsRed), 9.4, Math.toRadians(Cords.angleFlip(180, IsRed)));
        shootPos = new Pose(Cords.xFlip(57, IsRed), 12, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeStart = new Pose(Cords.xFlip(20, IsRed), 9.7, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeEnd = new Pose(Cords.xFlip(7.7, IsRed),  9.7, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeCornerStart = new Pose(Cords.xFlip(21, IsRed), 9.7, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeCornerEnd = new Pose(Cords.xFlip(11, IsRed),  9.7, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeSpikeMarkStart = new Pose(Cords.xFlip(47.5, IsRed), 35, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeSpikeMarkEnd = new Pose(Cords.xFlip(13.530973451327434, IsRed),  35, Math.toRadians(Cords.angleFlip(180, IsRed)));

        intakeTunnelStart = new Pose(Cords.xFlip(13.3, IsRed), 14, Math.toRadians(Cords.angleFlip(130, IsRed)));
        intakeTunnelEnd = new Pose(Cords.xFlip(13.3, IsRed),  43, Math.toRadians(Cords.angleFlip(130, IsRed)));

    }


}