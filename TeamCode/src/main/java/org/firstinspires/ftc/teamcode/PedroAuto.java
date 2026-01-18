package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




@Configurable
@Autonomous
public class PedroAuto extends OpMode {

   private Follower follower;

    //Overarching auto timer
    Timer autoTimer = new Timer();

    //1 == true, 0 == false
    public static boolean IsRed = false;

    public static boolean DidAutoGoToEnd;

    public static double PARK_TIME_TRIGGER = 28;

    private Timer pathTimer, opModeTimer;

    // -------- FLYWHEEL SETUP -------
    private Coordinates Cords = new Coordinates();
    private FlywheelLogic shooter = new FlywheelLogic();
    private Intake intake = new Intake();
    private TurretRotation turretRotation = new TurretRotation();
    private ShooterAngle hood = new ShooterAngle();
    private AprilTagVision camera;

    private FunctionsAndValues FAndV = new FunctionsAndValues();

    private boolean shotsTriggered=false;


    public enum PathState{
        // StartPos - EndPos
        DRIVE_TO_SHOOT_POS,
        DRIVE_TO_INTAKE_POS,
        INTAKE_BALLS,
        DRIVE_BACK_TO_SHOOT,
        SHOOT,
        FINISHED,

        AUTOPARK
    }
    PathState pathState;


    //for intaking balls where positiion needs to be precise
    private static double ALLOWED_ERROR_POSITION = 3;
    private static double ALLOWED_ERROR_VELOCITY = 15;

    //BallLines

    private final double BALL_LINE_DIFFERENCE = -24;
    private double ball_line_offset;
    private double loop_times;

    // -------- everything Poses ---------





    //public static Pose startPose = new Pose(Coordinates.START_X,Coordinates.START_Y,Math.toRadians(Coordinates.StartingRobotAngleDeg));
    public static Pose startPose;
    private static Pose GoalLocationPose,GoalLocationPoseForDistance;

    //this is to track last pose recorded for TeleOp
    // it is start pose cuz if the code never starts then the last position is the start position :)
    public static Pose LastPoseRecorded;
    private static Pose driveToPark;

    // ------ these are for use only in this AUTO -------
    private static  Pose shootPos,shootPos180,intakeStart,intakeEnd;
    private PathChain driveStartToShootPos, driveShootPosToIntake, driveIntakeForward, driveFromIntake1ToShootPos;

    private boolean isStateBusy;





    public void statePathUpdate() {
        if (autoTimer.getElapsedTimeSeconds() > PARK_TIME_TRIGGER){
            setPathState(PathState.AUTOPARK);
            isStateBusy = false;
        }

        switch(pathState) {
            case DRIVE_TO_SHOOT_POS:
                if(!isStateBusy){
                    follower.followPath(driveStartToShootPos, 1,false);
                    isStateBusy=true;
                }

                if (isStateBusy &&!follower.isBusy()) {
                    intake.intakeOn(1,1); // to cycle balls to shooter
                    shooter.fireShots(3); //change to three
                    isStateBusy = false;
                    setPathState(PathState.DRIVE_TO_INTAKE_POS);
                }

                break;

            case DRIVE_TO_INTAKE_POS:

                if (!shooter.isBusy()){
                        intake.intakeOff();// to stop cycling balls to shooter.
                        follower.followPath(driveShootPosToIntake, true);
                        setPathState(PathState.INTAKE_BALLS);
                }
                break;

            case INTAKE_BALLS:

                intake.intakeOn(1,1);


                if (isStateBusy == false &&!follower.isBusy()&&isRobotInPosition(intakeStart)){
                    loop_times +=1;
                    follower.followPath(driveIntakeForward, .5,true);
                    isStateBusy = true;
                }


                if (!follower.isBusy() && isStateBusy ==true){
                    isStateBusy = false;
                    //intake.intakeOff();
                    setPathState(PathState.DRIVE_BACK_TO_SHOOT);
                }
                break;

            case DRIVE_BACK_TO_SHOOT:
                if(isStateBusy == false && !follower.isBusy()){
                    follower.followPath(driveFromIntake1ToShootPos, true);
                    isStateBusy = true;
                }

                if (isStateBusy ==true&&!follower.isBusy()){
                    isStateBusy =false;
                    setPathState(PathState.SHOOT);
                }

                break;

            case SHOOT:
                if(isStateBusy == false){
                    intake.intakeOn(1,1); // to cycle balls to shooter
                    shooter.fireShots(3);
                    isStateBusy=true;
                }

                if (isStateBusy ==true&&!shooter.isBusy()&&pathTimer.getElapsedTimeSeconds()>1.5){
                    isStateBusy =false;

                    if (loop_times >= 3) {
                        setPathState(PathState.FINISHED);
                    }
                    else{
                        ball_line_offset+=BALL_LINE_DIFFERENCE;
                        buildPoses();
                        buildPaths();
                        setPathState(PathState.DRIVE_TO_INTAKE_POS);
                    }
                    intake.intakeOff();
                }


                break; // was commented out for some reason

            case FINISHED:
                DidAutoGoToEnd = true;
                break;

            case AUTOPARK:
                if (isStateBusy==false) {
                    turretRotation.TurretTo0Deg(true);
                    intake.intakeOff();
                    shooter.Stop();

                    follower.breakFollowing();
                    //follower.setPose(follower.getPose());


                    driveToPark = new Pose(Cords.xFlip(40, IsRed), 60, Math.toRadians(Cords.angleFlip(180, IsRed)));
                    Pose currentPose = follower.getPose();

                    PathChain driveToParkPath;

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
        ball_line_offset=0;
        loop_times = 0;
        DidAutoGoToEnd = false;


        pathState = PathState.DRIVE_TO_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        //We might want follower = new Follower(hardwareMap);, just check this if it doesnt work

        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        turretRotation.init(hardwareMap);
        hood.init(hardwareMap);
        camera = new AprilTagVision(hardwareMap);

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
        LastPoseRecorded = follower.getPose();

        double DistanceFromGoal = turretRotation.GetDistanceFromGoal(follower.getPose(), GoalLocationPoseForDistance);

        camera.update();
        follower.update();
        shooter.updateWithStateMachine(turretRotation.isTurretFinishedRotating());
        turretRotation.update(Math.toDegrees(follower.getTotalHeading()),follower.getPose(), GoalLocationPose, startPose);;
        turretRotation.handleBearing(camera.getBearing(),camera.getYaw());
        statePathUpdate();

        double[] turretGoals = FAndV.handleShootingRangesForWebcam(DistanceFromGoal- FunctionsAndValues.OffsetForShootingAlgorithmRemoveLater);
        hood.SetPosition(turretGoals[0]);
        shooter.setFlywheelTPS(turretGoals[1]);

        //turret.handleBearing(camera.getBearing());
        //telemetry.addData("Target angle: ", turretRotation.GetTargetAngle());
        //telemetry.addData("Turret Offset", TurretRotation.turret_offset);
        telemetry.addData("Is Shooter Busy?", shooter.isBusy());
        telemetry.addData("Shots Remaining", shooter.GetShotsRemaining());
        telemetry.addData("Path State", pathState.toString());
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
                .build();

        driveFromIntake1ToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(intakeEnd, shootPos180))
                .setLinearHeadingInterpolation(intakeEnd.getHeading(), shootPos180.getHeading())
                .build();

    }

    private void buildPoses(){
        startPose = new Pose(Cords.xFlip(Coordinates.START_X, IsRed), Coordinates.START_Y, Math.toRadians(Cords.angleFlip(Coordinates.StartingRobotAngleDeg, IsRed)));
        shootPos = new Pose(Cords.xFlip(59, IsRed), 85, Math.toRadians(Cords.angleFlip(180, IsRed)));
        shootPos180 = new Pose(shootPos.getX(), shootPos.getY(), Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeStart = new Pose(Cords.xFlip(51, IsRed), 84.5+ball_line_offset, Math.toRadians(Cords.angleFlip(180, IsRed)));
        intakeEnd = new Pose(Cords.xFlip(17, IsRed),  84.5+ball_line_offset, Math.toRadians(Cords.angleFlip(180, IsRed)));

        GoalLocationPose = new Pose(Cords.xFlip(Coordinates.GOAL_X,IsRed), Coordinates.GOAL_Y, Math.toRadians(0));
        GoalLocationPoseForDistance = new Pose(Cords.xFlip(Coordinates.GOAL_X_FOR_DISTANCE,IsRed), Coordinates.GOAL_Y_FOR_DISTANCE, Math.toRadians(0));
    }

    private boolean isRobotInPosition(Pose GoalPose) {
        Vector VelocityVector = follower.getVelocity();
        double vX = VelocityVector.getXComponent();
        double vY = VelocityVector.getYComponent();
        double Velocity = Math.hypot(vX, vY);

        boolean isVelocityAcceptable = Velocity <= ALLOWED_ERROR_VELOCITY;

        double dx = GoalPose.getX() - follower.getPose().getX();
        double dy = GoalPose.getY() - follower.getPose().getY();
        double difference = Math.hypot(dx, dy);

        boolean isPositionAcceptable = difference <= ALLOWED_ERROR_POSITION;

        if (isPositionAcceptable && isVelocityAcceptable) {
            return true;
        } else {
            return false;

        }
    }
}
