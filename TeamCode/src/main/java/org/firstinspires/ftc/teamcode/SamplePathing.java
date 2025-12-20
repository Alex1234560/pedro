package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
// interface

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class SamplePathing extends OpMode {


    public static boolean ShootingNow = false;
    public static boolean SpinShootingBallFeeder = false;
    private FunctionsAndValues FAndV;

    private Intake intake;


    public class Intake{
        private DcMotorEx intakeMotor;
        private DcMotor StopIntakeMotor;
        private CRServo BallFeederServo;

        public Intake(HardwareMap hardwareMap) {
            BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
            BallFeederServo.setDirection(CRServo.Direction.REVERSE);
            BallFeederServo.setPower(0);

            intakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
            intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeMotor.setDirection(DcMotor.Direction.REVERSE);
            StopIntakeMotor = hardwareMap.get(DcMotor.class, "StopIntake");
            StopIntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        }

        public void IntakeOn() {
            intakeMotor.setPower(1);
            StopIntakeMotor.setPower(1);
            }

        public void IntakeOff() {
            intakeMotor.setPower(0);
            StopIntakeMotor.setPower(0);
        }

        public void PassABallToShooter() {
            intakeMotor.setPower(.7);
            StopIntakeMotor.setPower(.7);
            BallFeederServo.setPower(1);
        }


    public class Shooter {
        private Servo ServoShooter1;
        private Servo ServoShooter2;
        private Servo ShooterRotatorServo;;
        private DcMotorEx ShooterMotor = null;
        private DcMotorEx ShooterMotor2 = null;


        public Shooter(HardwareMap hardwareMap) {
            ShooterRotatorServo = hardwareMap.get(Servo.class, "ShooterRotatorServo");
            ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");
            ServoShooter2 = hardwareMap.get(Servo.class, "ServoShooter2");
            ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
            ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
            ShooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            ServoShooter1.setDirection(Servo.Direction.REVERSE);
            ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }

        public void AimShooter() {

        }
        public void RunShooter() {

            ElapsedTime timer; // To track time

            double duration = 5;   // How long to run in seconds

            double GoalTPS = 0;
            double ServoAngle;

            double ShooterMotorPower=0;
            double shooterTPS=0;
            boolean far=false;


            FAndV = new FunctionsAndValues();

            telemetry.addData("shooterTPS: ", shooterTPS);
            telemetry.addData("GoalTPS: ", GoalTPS);
            telemetry.addData("SpinShootingBallFeeder : ", SpinShootingBallFeeder);
            telemetry.addData("ShooterMotorPower : ", ShooterMotorPower);

            if (Math.abs(shooterTPS-GoalTPS)<=FunctionsAndValues.SpeedToleranceToStartShooting){SpinShootingBallFeeder=true;}
            else{SpinShootingBallFeeder=false;
                telemetry.addData("No SPinning Ball Feeder : ", "");
            }


            shooterTPS = ShooterMotor.getVelocity();

            ShooterMotorPower = FAndV.handleShooter(shooterTPS,true, GoalTPS, ShooterMotorPower);
            ShooterMotor.setPower(ShooterMotorPower);
            ShooterMotor2.setPower(ShooterMotorPower);

            telemetry.update();
        }
    }
}




    private Follower follower;

    private Timer pathTimer, opModeTimer;

    public enum PathState{
        // StartPos - EndPos
        DRIVE_TO_INTAKE,

        SHOOT_PRELOAD,
        INTAKE_BALLS
    }
    PathState pathState;

    private final Pose startPose = new Pose(17.992, 118.676, Math.toRadians(53));
    private final Pose shootPos = new Pose(59, 85, Math.toRadians(53));
    private final Pose intakeStart = new Pose();
    private final Pose intakeEnd = new Pose();

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
                .addTemporalCallback(0.0, () -> intake.IntakeOn())

                // Stop intake 2.0 seconds after this path starts
                .addTemporalCallback(2.0, () -> intake.IntakeOff())

                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_TO_INTAKE:
                follower.followPath(driveStartToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if (!follower.isBusy() && pathTimer.getElapsedTime() > 2) {



                    follower.followPath(driveShootPosToIntake, true);
                    setPathState(PathState.INTAKE_BALLS);
                }
                break;
            case INTAKE_BALLS:
                if (!follower.isBusy()) {

                    //Intook balls, move backwards

                }
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
        //We might want follower = new Follower(hardwareMap);, just check this if it doesnt work

        intake = new Intake(hardwareMap);


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
