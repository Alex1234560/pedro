package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp

//
public class CleanTeleop extends LinearOpMode {
    // Hardware Setup Variables
    //private Servo ServoShooter1;
    //private Servo ReadyToShootServo;
    private static double ShooterAngle = FunctionsAndValues.startPoint;
    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();



    private Intake intake = new Intake();
    private FlywheelLogic shooter = new FlywheelLogic();
    private ShooterAngle hood = new ShooterAngle();


    //pedro stuff
    private Follower follower;
    //public static Pose startingPose; //See ExampleAuto to understand how to use this

    //private TelemetryManager telemetryM;

    private FunctionsAndValues FAndV;

    public static boolean fieldCentricDrive = false;
    public static double side = 1; // 1 == blue, -1==red

    private final Pose GoalLocationPose = new Pose(73, 140, Math.toRadians(0));
    public static Pose StartingPosition = new Pose(73,120,Math.toRadians(0));

    // angle for hooded shooter
    double HoodAngle = 0;// value from 0 to 1.0

    //april tag stuff
    private double AprilTagBearing = 0;


    //Variables for statement printing
    private static double ShooterMotorPower = 0;
    public static double GoalShooterMotorTPS = 1200;// rotation ticks per seond

    //variables avobe for testing
    public double shooterTPS = 0; // Ticks per second
    public double range = 0;

    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off
    private double ShootMechanismPower;

    //declaring button globally
    //private boolean autoAimButton = false;
    public static boolean AutoAim = false;

    private TurretRotation turretRotation = new TurretRotation();


    @Override
    public void runOpMode() {
        //currentAngle=90;//center current angle for shooter
        hood.init(hardwareMap);
        shooter.init(hardwareMap);
        turretRotation.init(hardwareMap);
        intake.init(hardwareMap);

        // remove the following once the turret stuff is integrated into the auto, this will go in the auto

        //--------------- ^^ -------------------------


        //pedro stuff
        follower = Constants.createFollower(hardwareMap);
        //hard set pose for now
        follower.setStartingPose(StartingPosition); // in front of blue goal pos
        // ---- below is for after, when auto starts and then the position is used ----
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        //telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();


        FAndV = new FunctionsAndValues();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses START)
        waitForStart();
        //pedro
        follower.startTeleopDrive();
        runtime.reset();

        //test
        turretRotation.TurretCalibrateToCenter();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //pedro
            turretRotation.update(Math.toDegrees(follower.getHeading()),follower.getPose(),GoalLocationPose);
            follower.update();
            shooter.update();
            //telemetryM.update();

            if (gamepad2.aWasPressed()) {AutoAim = true;}
            if (gamepad2.bWasPressed()) {AutoAim = false;}


            handleDriving();
            handleIntake();
            handleShooterServos();
            TelemetryStatements();

            //telemetryM.debug("position", follower.getPose());
            //telemetryM.debug("velocity", follower.getVelocity());

//

//            if (!shooterMotorOn){
//                shooter.TurnFlywheelOff();
//                shooter.SetIdle();
//            }

            if (gamepad2.xWasPressed()) {shooterMotorOn = true;}
            if (gamepad2.yWasPressed()) {shooterMotorOn = false;}
            //if (!shooterMotorOn){shooter.TurnFlywheelOff();}

            if (shooterMotorOn){shooter.SetMotorPowerToTarget();}
            else{shooter.TurnFlywheelOff();}


            if (ShootMechanismPower == 1 && shooter.IsFlywheelUpToSpeed()){
                shooter.SpinBallFeeder(1);
            }
            else if (ShootMechanismPower==-1){shooter.SpinBallFeeder(-1);}
            else{shooter.SpinBallFeeder(0);}

            telemetry.addData("FlywheelSpeed: " ,shooter.GetFlywheelSpeed());


        }
    }

    private void TelemetryStatements(){
        telemetry.addData("FieldCentricDrive?: ", fieldCentricDrive);
        telemetry.addData("Turret Rotation Ticks/Sec ", turretRotation.GetCurrentVel());
        telemetry.addData("Turret Rotation Ticks ", turretRotation.GetCurrentPos());
        telemetry.addData("Heading", follower.getHeading());
        telemetry.addData("Flywheel Speed: " ,shooter.GetFlywheelSpeed());

        //telemetry.addData("x", follower.getPose().getX());
        //telemetry.addData("y", follower.getPose().getY());
        telemetry.update();
    }


    /*
    private void handleFlywheel(){

        shooterTPS = FAndV.GetSpeedAvgFromTwoMotors(ShooterMotor.getVelocity(),ShooterMotor2.getVelocity());

        //flywheel stuff.

        if (gamepad2.xWasPressed()) {//(isXPressed && !wasXButtonPressed) {
            shooterMotorOn = true;
        }
        if (gamepad2.yWasPressed()) {//(isXPressed && !wasXButtonPressed) {
            shooterMotorOn = false;
        }

        if (gamepad2.dpadDownWasPressed() && GoalShooterMotorTPS > FunctionsAndValues.MinimumSpeed) {
            GoalShooterMotorTPS -= 20;
        }
        else if (gamepad2.dpadUpWasPressed() && GoalShooterMotorTPS < 2400) {
            GoalShooterMotorTPS += 20;

        }

        ShooterMotorPower = FAndV.handleShooter(shooterTPS,shooterMotorOn,GoalShooterMotorTPS, ShooterMotorPower);
        ShooterMotor.setPower(ShooterMotorPower);
        ShooterMotor2.setPower(ShooterMotorPower);

        telemetry.addData("ShooterMotorSpeed= ", ShooterMotorPower);
    }*/

    private void handleIntake() {
        // handle feeding to shooter
         ShootMechanismPower = 0; //Positive value = shooting, negative value = retract balls and spit them out

        if ( gamepad2.right_trigger > 0) {
            ShootMechanismPower=1;
        }

        else if(gamepad2.back) {
            ShootMechanismPower=-1;
        }


        /*//IntakeMotor.setPower(-gamepad2.right_trigger * ShootMechanismPower);
        //StopIntakeMotor.setPower(gamepad2.right_trigger *ShootMechanismPower);
        BallFeederServo.setPower(gamepad2.right_trigger * ShootMechanismPower);
        if (ShootMechanismPower==-1){BallFeederServo.setPower(-1);}
        BallFeederServo2.setPower(BallFeederServo.getPower());

         */


        //handle normal intake stuff

        double IntakePowerValue = -Math.abs(gamepad2.left_trigger);
        if (Math.abs(gamepad1.left_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = -Math.abs(gamepad1.left_trigger);
        }
        if (Math.abs(gamepad2.right_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = -Math.abs(gamepad2.right_trigger);
        }
        if (ShootMechanismPower==-1){
            IntakePowerValue = 1;
        }

        intake.intakeOn(IntakePowerValue);
    }

    private void handleDriving() {
        double speed = .5; //
        //if (gamepad1.right_trigger ==1){speed = 1;}
        speed += gamepad1.right_trigger/2 ; // trigger makes it slower

        double axial = -gamepad1.left_stick_y * speed;
        double lateral = -gamepad1.left_stick_x * speed; // Note: pushing stick forward gives negative value
        double yaw = -gamepad1.right_stick_x * speed;

        //parking precisly

        if (gamepad1.dpad_right){lateral= +.2;}
        if (gamepad1.dpad_left){lateral= -.2;}
        if (gamepad1.dpad_up){axial= +.1;}
        if (gamepad1.dpad_down){axial= -.1;}
        if (gamepad1.x){yaw = -.1;}
        if (gamepad1.b){yaw = +.1;}

        //field centric

        if (gamepad1.yWasPressed()) {
            fieldCentricDrive = true;
        }
        if (gamepad1.aWasPressed()) {
            fieldCentricDrive = false;
        }

        //send commands to pedro.

        if (!fieldCentricDrive){

            follower.setTeleOpDrive(
                    axial,
                    lateral,
                    yaw,
                    true // Robot Centric
            );
        }
        else if (side==1){
            follower.setTeleOpDrive(

                    -lateral,
                    axial,
                    yaw,
                    false // Robot Centric
            );
        }
        else if (side==-1){
            follower.setTeleOpDrive(

                    lateral,
                    -axial,
                    yaw,
                    false // Robot Centric
            );
        }




    }

    private void handleShooterServos() {

        // Initialize shooterAngle with the servo's current position to start.
        // This ensures it always has a value.

        ShooterAngle-=gamepad2.left_stick_y/22;
        hood.SetPosition(ShooterAngle);
        telemetry.addData("ServoAngle ", ShooterAngle );

    }
    private void handleTeleOpShooting(){

        if (gamepad2.xWasPressed()) {shooterMotorOn = true;}
        if (gamepad2.yWasPressed()) {shooterMotorOn = false;}
        //if (!shooterMotorOn){shooter.TurnFlywheelOff();}

        if (shooterMotorOn){shooter.SetMotorPowerToTarget();}
        else{shooter.TurnFlywheelOff();}


        if (ShootMechanismPower == 1 && shooter.IsFlywheelUpToSpeed()){
            shooter.SpinBallFeeder(1);
        }
        else if (ShootMechanismPower==-1){shooter.SpinBallFeeder(-1);}
        else{shooter.SpinBallFeeder(0);}
        
    }

}
