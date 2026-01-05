package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    private FunctionsAndValues FAndV = new FunctionsAndValues();
    private Intake intake = new Intake();
    private FlywheelLogic shooter = new FlywheelLogic();
    private ShooterAngle hood = new ShooterAngle();
    private AprilTagVision camera;


    //pedro stuff
    private Follower follower;
    //public static Pose startingPose; //See ExampleAuto to understand how to use this

    private TelemetryManager telemetryM;

    public static boolean fieldCentricDrive = false;
    public static double GOAL_X = 15;
    public static double GOAL_Y = 131;
    public static double STARTING_ANGLE_ROBOT = 144;

    private boolean IsRed = false;

    private Pose GoalLocationPose, StartingPosition;



    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off

    private boolean Reversing = false;      // Tracks if the motor should be on or off


    //declaring button globally
    //private boolean autoAimButton = false;
    public static boolean AutoAim = true;

    private TurretRotation turretRotation = new TurretRotation();



    @Override
    public void runOpMode() {
        //currentAngle=90;//center current angle for shooter
        IsRed = PedroAuto.IsRed; // defines the side of the field based on what the auto had selected as the side of the field.

        GoalLocationPose = new Pose(GOAL_X, GOAL_Y, Math.toRadians(0));
        StartingPosition = new Pose(22.5,125.5,Math.toRadians(STARTING_ANGLE_ROBOT));

        //rn will only work for blue sicne i need to fix some stuff for translation to other side,
        //GoalLocationPose = new Pose(16, 132, Math.toRadians(0));
        //StartingPosition = new Pose(18, 121.2, Math.toRadians(144));

        camera = new AprilTagVision(hardwareMap,"webcam");
        hood.init(hardwareMap);
        shooter.init(hardwareMap);
        turretRotation.init(hardwareMap);
        intake.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //pedro stuff
        follower = Constants.createFollower(hardwareMap);
        //hard set pose for now
        follower.setStartingPose(StartingPosition); // in front of blue goal pos
        // ---- below is for after, when auto starts and then the position is used ----
        //follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

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
            GoalLocationPose = new Pose(GOAL_X, GOAL_Y, Math.toRadians(0));
            StartingPosition = new Pose(StartingPosition.getX(),StartingPosition.getY(),Math.toRadians(STARTING_ANGLE_ROBOT));
            //pedro
            turretRotation.update(Math.toDegrees(follower.getTotalHeading()),follower.getPose(),GoalLocationPose, StartingPosition);
            camera.update();
            follower.update();
            shooter.update();

            double DistanceFromGoal = turretRotation.GetDistanceFromGoal(follower.getPose(), GoalLocationPose);

            if (AutoAim){
                double[] turretGoals = FAndV.handleShootingRanges(DistanceFromGoal);
                hood.SetPosition(turretGoals[0]);
                shooter.setFlywheelRPM(turretGoals[1]);
            }


            if (gamepad2.aWasPressed()) {AutoAim = true;}
            if (gamepad2.bWasPressed()) {AutoAim = false;}


            handleDriving();
            handleIntake();
            handleShooterServos();
            TelemetryStatements();

            handleTeleOpShooting();

            if (gamepad2.back){
                Reversing=true;
            }
            else{Reversing=false;}


        }
    }

    private void TelemetryStatements(){
        telemetryM.addData("FieldCentricDrive?: ", fieldCentricDrive);
        telemetryM.addData("Turret Rotation Ticks/Sec ", Math.round(turretRotation.GetCurrentVel()));
        telemetryM.addData("Distance From Goal ", turretRotation.GetDistanceFromGoal(follower.getPose(), GoalLocationPose));

        telemetryM.addData("Angle From Goal ", Math.round(turretRotation.GetAngleThatIsbeingReturnedForAutoAiming(follower.getPose(), GoalLocationPose)));
        telemetryM.addData("Hood Angle", hood.getPosition());

        telemetryM.addData("Turret Rotation Deg ", Math.round(turretRotation.GetCurrentPosDeg()));
        //telemetryM.addData("Heading", Math.toDegrees(follower.getTotalHeading()));
        telemetryM.addData("Flywheel Speed" ,shooter.GetFlywheelSpeed());
        //telemetryM.addData("Power Of Ball Feeder" ,shooter.GetBallFeederPowerForDebugging()*100);
        //telemetryM.addData("Debugging angle Compensation" ,Math.round(turretRotation.DebugGetAngleCompensation()));

        telemetryM.addData("Bearing (Camera) " ,camera.getBearing());
        telemetryM.addData("Distance (Camera) " ,camera.getRange());

        //telemetry.addData("x", follower.getPose().getX());
        //telemetry.addData("y", follower.getPose().getY());
        //telemetry.update();
        telemetryM.debug("x:" + Math.round(follower.getPose().getX()));
        telemetryM.debug("y:" + Math.round(follower.getPose().getY()));
        telemetryM.debug("heading:" + Math.round(Math.toDegrees(follower.getPose().getHeading())));
        telemetryM.debug("total heading:" + Math.round(Math.toDegrees(follower.getTotalHeading())));
        telemetryM.update(telemetry);

    }
    private void handleIntake() {
        // handle feeding to shooter



        double IntakePowerValue = -Math.abs(gamepad2.left_trigger);
        if (Math.abs(gamepad1.left_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = -Math.abs(gamepad1.left_trigger);
        }
        if (Math.abs(gamepad2.right_trigger) > Math.abs(gamepad2.left_trigger) && !Reversing){
            IntakePowerValue = -Math.abs(gamepad2.right_trigger);
        }


        if (Reversing){
            intake.intakeOn(-IntakePowerValue,0);
        }
        else if(IntakePowerValue!=0){
            intake.intakeOn(IntakePowerValue,-1);
        }
        else{
            intake.intakeOff();
        }



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
        else if (IsRed==false){
            follower.setTeleOpDrive(

                    -lateral,
                    axial,
                    yaw,
                    false // Robot Centric
            );
        }
        else if (IsRed==true){
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

        if (AutoAim==false) {

            ShooterAngle -= gamepad2.left_stick_y / 22;
            hood.SetPosition(ShooterAngle);
            telemetry.addData("ServoAngle ", ShooterAngle);
        }

    }
    private void handleTeleOpShooting(){

        if (gamepad2.xWasPressed()) {shooterMotorOn = true;}
        if (gamepad2.yWasPressed()) {shooterMotorOn = false;}
        //if (!shooterMotorOn){shooter.TurnFlywheelOff();}

        if (shooterMotorOn){shooter.SetMotorPowerToTarget();}
        else{shooter.TurnFlywheelOff();}

        if (Reversing && gamepad2.right_trigger>0){shooter.SpinBallFeeder(-1);}

        else if (gamepad2.right_trigger > 0 && shooter.IsFlywheelUpToSpeed()){
            shooter.SpinBallFeeder(1);
        }
        else if (gamepad2.right_trigger > 0 && gamepad2.right_bumper){
            shooter.SpinBallFeeder(1);
        }

        else{shooter.SpinBallFeeder(0);}
    }
}