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

    //1 == true, 0 == false
    public static boolean IsRed = false;


    private Timer pathTimer, opModeTimer;

    // -------- FLYWHEEL SETUP -------

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
        FINISHED
    }
    PathState pathState;


    //for intaking balls where positiion needs to be precise
    private static double ALLOWED_ERROR_POSITION = 3;
    private static double ALLOWED_ERROR_VELOCITY = 15;

    // -------- everything Poses ---------

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


    // ---- following hard poses are for blue side in case CleanTeleop is started for practice ---
    private static final double StartingRobotAngleDeg = 144;
    private static final double GOAL_X = 7.5;
    private static final double GOAL_Y = 142.122;
//    private static final double GOAL_X = 16;
//    private static final double GOAL_Y = 132;
    private static final double START_X = 17.914;
    private static final double START_Y = 121.168;
    // ----- NOTE: These poses will get rewritten in this file, their only purpose is to serve
    // as a default for the TeleOp File.

    public static Pose startPose = new Pose(START_X,START_Y,Math.toRadians(StartingRobotAngleDeg));
    public static Pose GoalLocationPose = new Pose(GOAL_X, GOAL_Y, Math.toRadians(0));

    //this is to track last pose recorded for TeleOp
    // it is start pose cuz if the code never starts then the last position is the start position :)
    public static Pose LastPoseRecorded = startPose;

    // ------ these are for use only in this AUTO -------
    private static  Pose shootPos,intakeStart,intakeEnd;
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

                if (!follower.isBusy()&&!shooter.isBusy()){
                        follower.followPath(driveShootPosToIntake, true);
                        setPathState(PathState.INTAKE_BALLS);
                }
                break;

            case INTAKE_BALLS:

                intake.intakeOn(1,1);
                //if (!follower.isBusy()){ previous code, replace in case don't work

                //set a timer in case isRobotInPosition never happens
                if (!follower.isBusy()&& isRobotInPosition(intakeStart)){

                    follower.followPath(driveIntakeForward, true);
                    setPathState(PathState.FINISHED);
                }
                break;

            case FINISHED:
                if (!follower.isBusy()) {
                    intake.intakeOff();
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

        camera.update();
        follower.update();
        shooter.update();
        turretRotation.update(Math.toDegrees(follower.getTotalHeading()),follower.getPose(), GoalLocationPose, startPose);;
        turretRotation.handleBearing(camera.getBearing(),camera.getYaw());
        statePathUpdate();

        double DistanceFromGoal = turretRotation.GetDistanceFromGoal(follower.getPose(), GoalLocationPose);
        double[] turretGoals = FAndV.handleShootingRanges(DistanceFromGoal);
        hood.SetPosition(turretGoals[0]);
        shooter.setFlywheelRPM(turretGoals[1]);

        //turret.handleBearing(camera.getBearing());

        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());

        telemetry.update();
    }

    private void buildPoses(){
        startPose = new Pose(xFlip(START_X, IsRed), START_Y, Math.toRadians(angleFlip(StartingRobotAngleDeg, IsRed)));
        shootPos = new Pose(xFlip(59, IsRed), 85, Math.toRadians(angleFlip(144, IsRed)));
        intakeStart = new Pose(xFlip(44.147, IsRed), 60.5, Math.toRadians(angleFlip(180, IsRed)));
        intakeEnd = new Pose(xFlip(20.662, IsRed),  60.5, Math.toRadians(angleFlip(180, IsRed)));

        GoalLocationPose = new Pose(xFlip(GOAL_X,IsRed), GOAL_Y, Math.toRadians(0));
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
