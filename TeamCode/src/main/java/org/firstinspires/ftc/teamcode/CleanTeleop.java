package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp
public class CleanTeleop extends OpMode {
    // Hardware Setup Variables
    //private Servo ServoShooter1;
    //private Servo ReadyToShootServo;

    private static double ShooterAngle = FunctionsAndValues.startPoint;
    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();

    private Coordinates Cords = new Coordinates();
    private FunctionsAndValues FAndV = new FunctionsAndValues();
    private Intake intake = new Intake();
    private FlywheelLogic shooter = new FlywheelLogic();
    private ShooterAngle hood = new ShooterAngle();
    private AprilTagVision camera;


    //pedro stuff
    private Follower follower;
    //public static Pose startingPose; //See ExampleAuto to understand how to use this

    private TelemetryManager telemetryM;

    public static boolean fieldCentricDrive = true;
//    public static double GOAL_X = 15;
//    public static double GOAL_Y = 131;
    //public static double STARTING_ANGLE_ROBOT = 144;

    private boolean IsRed = false;



    private Pose GoalLocationPose, StartingPosition;


    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off

    private boolean Reversing = false;      // Tracks if the motor should be on or off


    //declaring button globally
    //private boolean autoAimButton = false;
    public static boolean AutoAim = true;
    private boolean START_PROGRAM_WITOUTH_AUTO_FIRST = true;

    private TurretRotation turretRotation = new TurretRotation();
    //private PedroAuto PedroAutoFunctions = new PedroAuto();

    @Override
    public void init(){





        camera = new AprilTagVision(hardwareMap);
        hood.init(hardwareMap);
        shooter.init(hardwareMap);
        turretRotation.init(hardwareMap);
        intake.init(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        //------------- FOLLOWER STUFF --------------
        follower = Constants.createFollower(hardwareMap);
        //new try start pose



        follower.update();
    }

    @Override
    public void start(){

        GoalLocationPose = new Pose(Cords.xFlip(Coordinates.GOAL_X,IsRed), Coordinates.GOAL_Y, Math.toRadians(0));



        if (START_PROGRAM_WITOUTH_AUTO_FIRST){
            StartingPosition = new Pose(Cords.xFlip(Coordinates.START_X, IsRed), Coordinates.START_Y, Math.toRadians(Cords.angleFlip(Coordinates.StartingRobotAngleDeg, IsRed)));
        }
        else{
            StartingPosition = PedroAuto.LastPoseRecorded;
        }


        follower.setStartingPose(StartingPosition);

        follower.startTeleopDrive();
        runtime.reset();

        //auto aiming and location sor
         // defines the side of the field b ased on what the auto had selected as the side of the field.


        //should be in center when passed on from auto so that it can calibrate.
        turretRotation.CalibrateTurretToCenter();

    }
@Override
    public void init_loop(){
        telemetry.addData("Status", "Initialized");

        // in case you're starting teleop from fresh just for practice
        telemetry.addData("press 'back' to toggle the start program witouth running auto first.",START_PROGRAM_WITOUTH_AUTO_FIRST);
        if (gamepad1.backWasPressed()){START_PROGRAM_WITOUTH_AUTO_FIRST = !START_PROGRAM_WITOUTH_AUTO_FIRST;}

        if (START_PROGRAM_WITOUTH_AUTO_FIRST) {
            telemetry.addData("Alliance Selection", "X for BLUE, B for RED");
            if (IsRed == false) {
                telemetry.addData("Color: BLUE ", "");
            }
            if (IsRed == true) {
                telemetry.addData("Color: RED ", "");
            }

            if (gamepad1.x || gamepad2.x) {IsRed = false;} // blue
            if (gamepad1.b || gamepad2.b) {IsRed = true;} //red
        }

        else{
            IsRed = PedroAuto.IsRed;
        }

        telemetry.update();
    }
    @Override
    public void loop(){

        turretRotation.update(Math.toDegrees(follower.getTotalHeading()),follower.getPose(),GoalLocationPose, StartingPosition);
        camera.update();
        follower.update();
        shooter.update();

        double DistanceFromGoal = turretRotation.GetDistanceFromGoal(follower.getPose(), GoalLocationPose);

        if (AutoAim){
            double[] turretGoals = FAndV.handleShootingRanges(DistanceFromGoal- FunctionsAndValues.OffsetForShootingAlgorithmRemoveLater);// remove -4 in the future
            hood.SetPosition(turretGoals[0]);
            shooter.setFlywheelTPS(turretGoals[1]);


            turretRotation.handleBearing(camera.getBearing(),camera.getYaw());
        }


        if (gamepad2.bWasPressed()) {AutoAim = true;}
        if (gamepad2.aWasPressed()) {AutoAim = false;}


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

    private void TelemetryStatements(){
        telemetryM.addData("FieldCentricDrive?: ", fieldCentricDrive);
        telemetryM.addData("Turret Rotation Ticks/Sec ", Math.round(turretRotation.GetCurrentVel()));
        telemetryM.addData("Distance From Goal ", turretRotation.GetDistanceFromGoal(follower.getPose(), GoalLocationPose));

        //telemetryM.addData("turret rotation goal degree ", Math.round(turretRotation.GetGoalTrackingAngle()));
        telemetryM.addData("Hood Angle", hood.getPosition());

        telemetryM.addData("-360 turret angle?", turretRotation.IsTurretPastAnglePos());
        telemetryM.addData("Target Angle ", Math.round(turretRotation.GetTargetAngle()));
        telemetryM.addData("Turret Rotation Deg ", Math.round(turretRotation.GetCurrentPosDeg()));
        //telemetryM.addData("Heading", Math.toDegrees(follower.getTotalHeading()));
        telemetryM.addData("Flywheel Speed" ,shooter.GetFlywheelSpeed());
        //telemetryM.addData("Power Of Ball Feeder" ,shooter.GetBallFeederPowerForDebugging()*100);
        //telemetryM.addData("Debugging angle Compensation" ,Math.round(turretRotation.DebugGetAngleCompensation()));

        telemetryM.addData("IsRed?" ,IsRed);
        //for tuning purposes for auto
        telemetryM.addData("Is Turret Finished Rotating " ,turretRotation.isTurretFinishedRotating());

        telemetryM.addData("bearing used in Turret", turretRotation.GetCameraBearingUsedInFile());
        telemetryM.addData("Bearing " ,camera.getBearing());
        telemetryM.addData("Yaw " ,camera.getYaw());
        telemetryM.addData("Distance  " ,camera.getRange());

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



        double IntakePowerValue = Math.abs(gamepad2.left_trigger);
        if (Math.abs(gamepad1.left_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = Math.abs(gamepad1.left_trigger);
        }
        if (Math.abs(gamepad2.right_trigger) > Math.abs(gamepad2.left_trigger) && !Reversing){
            IntakePowerValue = Math.abs(gamepad2.right_trigger);
        }


        if (Reversing){
            intake.intakeOn(-IntakePowerValue,0);
        }
        else if(IntakePowerValue!=0){
            intake.intakeOn(IntakePowerValue,1);
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

        if (gamepad1.dpad_right){lateral= +.1;}
        if (gamepad1.dpad_left){lateral= -.1;}
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
                    //works i think
                    -axial,
                    -lateral,
                    yaw,
                    false // Robot Centric
            );
        }
        else if (IsRed==true){
            follower.setTeleOpDrive(
                    -lateral,
                    axial,
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

        //switching gamepad2.right_trigger fonr a butto

        if (Reversing && gamepad2.right_trigger>0){shooter.SpinBallFeeder(-1);}

        else if (gamepad2.right_trigger > 0 && shooter.IsFlywheelUpToSpeed() && turretRotation.isTurretFinishedRotating()){
            shooter.SpinBallFeeder(1);
        }
        else if (gamepad2.right_trigger > 0 && gamepad2.right_bumper){
            shooter.SpinBallFeeder(1);
        }

        else{shooter.SpinBallFeeder(0);}
    }


}
