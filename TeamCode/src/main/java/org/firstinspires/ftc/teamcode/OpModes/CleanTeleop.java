package org.firstinspires.ftc.teamcode.OpModes;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;


import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.AutoFunctions;
import org.firstinspires.ftc.teamcode.Functions.Coordinates;
import org.firstinspires.ftc.teamcode.Functions.FunctionsAndValues;
import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagVision;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.ShooterAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

// NOTE BALL COUNTING ISNT AMAZING IN THIS OP MODE BEACUSE OF CAMERA LOOP DELAY////

@Configurable
@TeleOp
public class CleanTeleop extends OpMode {
    // Hardware Setup Variables
    //private Servo ServoShooter1;
    //private Servo ReadyToShootServo;



    private static double HoodAngle = ShooterAngle.START_POINT;
    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();



    //private DistanceSensorClass distanceSensor = new DistanceSensorClass();
    private AutoFunctions autoFunctions = new AutoFunctions();
    private Coordinates Cords = new Coordinates();
    private FunctionsAndValues FAndV = new FunctionsAndValues();
    private Intake intake = new Intake();
    private ShooterLogic shooter = new ShooterLogic();
    private ShooterAngle hood = new ShooterAngle();
    private AprilTagVision camera;


    //pedro stuff
    private Follower follower;
    //public static Pose startingPose; //See ExampleAuto to understand how to use this

    private TelemetryManager telemetryM;

    public static boolean fieldCentricDrive = true;

    private boolean automatedDrive = false;
    private boolean tuningTelemetry = false;
//    public static double GOAL_X = 15;
//    public static double GOAL_Y = 131;
    //public static double STARTING_ANGLE_ROBOT = 144;

    private boolean IsRed = false;
    private boolean SlowMode = false;

    private Pose GoalLocationPose, StartingPosition, GoalLocationPoseForDistance, restartPos;


    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off

    private boolean Reversing = false;      // Tracks if the motor should be on or off


    //declaring button globally
    //private boolean autoAimButton = false;
    private static boolean UseOdosForSpeedAndDistance = true;
    public static boolean start_program_witouth_auto_first = true;

    private TurretRotation turretRotation = new TurretRotation();

    private double FlywheelSpeedForTuning = 1000;

    private boolean ManuallyAdjustableValues = false;


    Pose shootFrontPos, parkPos, shootBackPos, TriggerClassifierPos;

    Pose lastPoseTriggered;


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
        shootFrontPos = new Pose(Cords.xFlip(57, IsRed), 95, Math.toRadians(Cords.angleFlip(180, IsRed)));
        shootBackPos = new Pose(Cords.xFlip(57, !IsRed), 18, Math.toRadians(Cords.angleFlip(0, IsRed)));
        parkPos = new Pose(Cords.xFlip(38.71049304677624, !IsRed), 33.29329962073322, Math.toRadians(Cords.angleFlip(180, IsRed)));
        TriggerClassifierPos = new Pose(Cords.xFlip(16.5, IsRed), 70, Math.toRadians(Cords.angleFlip(90, IsRed)));


        GoalLocationPoseForDistance = new Pose(Cords.xFlip(Coordinates.GOAL_X_FOR_DISTANCE,IsRed), Coordinates.GOAL_Y_FOR_DISTANCE, Math.toRadians(0));
        GoalLocationPose = new Pose(Cords.xFlip(Coordinates.GOAL_X,IsRed), Coordinates.GOAL_Y, Math.toRadians(0));

        restartPos = new Pose(Cords.xFlip(Coordinates.RESTART_X,!IsRed), Coordinates.RESTART_Y, Math.toRadians(90));

        if (start_program_witouth_auto_first){
            StartingPosition = new Pose(Cords.xFlip(Coordinates.FRONT_START_X, IsRed), Coordinates.FRONT_START_Y, Math.toRadians(Cords.angleFlip(Coordinates.StartingRobotAngleDeg, IsRed)));
        }
        else{
            StartingPosition = AutoFunctions.LastPoseRecorded;
        }

        if (start_program_witouth_auto_first){//(AutoFunctions.DidAutoGoToEnd || start_program_witouth_auto_first){
            turretRotation.CalibrateTurretToCenter();
        }


        follower.setStartingPose(StartingPosition);

        follower.startTeleopDrive();
        runtime.reset();


    }
    @Override
    public void init_loop(){
        telemetry.addData("Status", "Initialized");

        // in case you're starting teleop from fresh just for practice
        telemetry.addData("press 'back' to toggle the start program witouth running auto first.", start_program_witouth_auto_first);
        if (gamepad1.backWasPressed()){
            start_program_witouth_auto_first = !start_program_witouth_auto_first;}

        if (start_program_witouth_auto_first) {
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
            IsRed = AutoFunctions.IsRed;
        }

        telemetry.update();
    }
    @Override
    public void loop(){



        turretRotation.update(Math.toDegrees(follower.getTotalHeading()),follower.getPose(),GoalLocationPose, StartingPosition, IsRed);

        follower.update();
        shooter.update();

        if (true){
        camera.update();}

        if (gamepad2.dpadLeftWasPressed()){
            tuningTelemetry=!tuningTelemetry;
        }



        double DistanceFromGoal = turretRotation.GetDistanceFromGoal(GoalLocationPoseForDistance );

        // ----- everything below for manually adjustable values -----
        if (gamepad2.start && gamepad2.dpadUpWasPressed() || gamepad2.start && gamepad2.dpadDownWasPressed()){
            ManuallyAdjustableValues=!ManuallyAdjustableValues;
            HoodAngle = hood.getPosition();
            FlywheelSpeedForTuning = ShooterLogic.TARGET_FLYWHEEL_TPS;
        }

        if (ManuallyAdjustableValues){
            HoodAngle -= gamepad2.left_stick_y / 22;
            hood.SetPosition(HoodAngle);

            if (gamepad2.dpadUpWasPressed()){
                FlywheelSpeedForTuning+=50;
            }
            if (gamepad2.dpadDownWasPressed()){
                FlywheelSpeedForTuning-=50;
            }
            shooter.setFlywheelTPS(FlywheelSpeedForTuning);
        }
        // --------------------------------------------------- /

        else if (UseOdosForSpeedAndDistance){
            double[] turretGoals = FAndV.handleShootingRanges(DistanceFromGoal- FunctionsAndValues.OffsetForShootingAlgorithmRemoveLater);// remove -4 in the future
            hood.SetPosition(turretGoals[0]);
            shooter.setFlywheelTPS(turretGoals[1]);
        }
        else if (camera.getRange()!=-1){
            double[] turretGoals = FAndV.handleShootingRanges(camera.getRange());// remove -4 in the future
            hood.SetPosition(turretGoals[0]);
            shooter.setFlywheelTPS(turretGoals[1]);

            turretRotation.handleBearing(camera.getBearing(),camera.getYaw());
        }
        turretRotation.handleBearing(camera.getBearing(),camera.getYaw());

        if (gamepad2.bWasPressed()) {
            UseOdosForSpeedAndDistance = true;
        }
        if (gamepad2.aWasPressed()) {
            UseOdosForSpeedAndDistance = false;}


        handleDriving();
        handleIntakeAndShootingButtons();
        handleResetPositionFunction();
        TelemetryStatements();
        handleFlywheel();
    }

    private void handleResetPositionFunction(){
//        if (gamepad1.start&&gamepad1.rightBumperWasPressed()){
//            follower.setPose(restartPos);
//        }
    }

    private void TelemetryStatements(){
        if (tuningTelemetry) {
            telemetryM.addData("Turret Finished Rotating?:  ", turretRotation.isTurretFinishedRotating());
            telemetryM.addData("TuningMode?:  ", ManuallyAdjustableValues);
            telemetryM.addData("Ball Count Distance Sensor ", shooter.GetBallsShotCount());
            telemetryM.addData("Turret Rotation Ticks/Sec ", Math.round(turretRotation.GetCurrentVel()));

            telemetryM.addData("Target Angle ", Math.round(turretRotation.GetTargetAngle()));
            telemetryM.addData("Distance Sensor value: ", shooter.GetDistance());
            telemetryM.addData("Bearing " ,camera.getBearing());
            telemetryM.addData("Yaw " ,camera.getYaw());
            telemetryM.addData("Distance (camera) " ,camera.getRange());

        }
        telemetryM.addData("FieldCentricDrive?: ", fieldCentricDrive);
        telemetryM.addData("shooter Goal Speed ", ShooterLogic.TARGET_FLYWHEEL_TPS);
        telemetryM.addData("Is flywheel up to speed?:  ", shooter.IsFlywheelUpToSpeed());
        telemetryM.addData("Distance From Goal ", turretRotation.GetDistanceFromGoal(GoalLocationPoseForDistance));
        telemetryM.addData("Hood Angle", hood.getPosition());
        telemetryM.addData("Flywheel Speed" ,shooter.GetFlywheelSpeed());

        telemetryM.debug("x:" + Math.round(follower.getPose().getX()));
        telemetryM.debug("y:" + Math.round(follower.getPose().getY()));
        telemetryM.debug("heading:" + Math.round(Math.toDegrees(follower.getPose().getHeading())));






        //telemetryM.addData("turret rotation goal degree ", Math.round(turretRotation.GetGoalTrackingAngle()));





        //telemetryM.addData("Turret Rotation Deg ", Math.round(turretRotation.GetCurrentPosDeg()));
        //telemetryM.addData("Turret Offset: ", Math.round(TurretRotation.turret_offset));

        //telemetryM.addData("Heading", Math.toDegrees(follower.getTotalHeading()));

        //telemetryM.addData("Power Of Ball Feeder" ,shooter.GetBallFeederPowerForDebugging()*100);
        //telemetryM.addData("Debugging angle Compensation" ,Math.round(turretRotation.DebugGetAngleCompensation()));

        //telemetryM.addData("IsRed?" ,IsRed);
        //for tuning purposes for auto
        //telemetryM.addData("Is Turret Finished Rotating " ,turretRotation.isTurretFinishedRotating());

        //telemetryM.addData("bearing used in Turret", turretRotation.GetCameraBearingUsedInFile());

        //telemetryM.debug("total heading:" + Math.round(Math.toDegrees(follower.getTotalHeading())));
        telemetryM.update(telemetry);

    }
    private void handleIntakeAndShootingButtons() {
        //BACK button reversing.

        boolean IntakeReversing = gamepad2.left_bumper || gamepad1.left_bumper;

        //intaking



        double IntakePowerValue = Math.abs(gamepad2.left_trigger);
        if (Math.abs(gamepad1.left_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = Math.abs(gamepad1.left_trigger);
        }
        if (Math.abs(gamepad2.right_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = Math.abs(gamepad2.right_trigger);
        }
        if (gamepad2.y){IntakePowerValue = 1;}

        if (IntakeReversing){
            intake.intakeOn(-1,1);
        }
        else if(IntakePowerValue!=0){
            intake.intakeOn(IntakePowerValue,1);
        }
        else{
            intake.intakeOff();
        }

        //shooting

        //boolean Trigger = gamepad2.right_trigger>0;
        boolean Trigger = gamepad2.y;

        if (gamepad2.back){shooter.SpinBallFeeder(-1);}

        else if (Trigger && shooter.IsFlywheelUpToSpeed() && turretRotation.isTurretFinishedRotating()){
            shooter.SpinBallFeeder(1);
        }

        else if (Trigger && gamepad2.right_bumper){
            shooter.SpinBallFeeder(1);
        }

        //so that any button that intakes can spin ball feeder
        else if (IntakePowerValue>0 && !shooter.IsBallDetected()){
            shooter.SpinBallFeeder(FunctionsAndValues.PowerValueForPreloading); // power less so that it doesnt pass the point it needs to go to and get shot
        }

        else{shooter.SpinBallFeeder(0);}

    }

    private void handleDriving() {
        double speed = .5; //
        //if (gamepad1.right_trigger ==1){speed = 1;}
        double speedModifier = gamepad1.right_trigger / 2;
        speed += speedModifier; // trigger makes it slower

        if (SlowMode){speed=.1+speedModifier/1.5;}
        if (gamepad1.leftStickButtonWasPressed()){SlowMode=!SlowMode;}


        double axial = -gamepad1.left_stick_y * speed;
        double lateral = -gamepad1.left_stick_x * speed; // Note: pushing stick forward gives negative value
        double yaw = -gamepad1.right_stick_x * speed;

        if (!automatedDrive) {
            //follower.startTeleopDrive();

            //parking precisly
//            double baseValue = 0.05;
//
//            if (gamepad1.dpad_right) {
//                lateral = -baseValue + speed / 3;
//            }
//            if (gamepad1.dpad_left) {
//                lateral = +baseValue - speed / 3;
//            }
//            if (gamepad1.dpad_up) {
//                axial = +baseValue + speed / 3;
//            }
//            if (gamepad1.dpad_down) {
//                axial = -baseValue - speed / 3;
//            }
//            if (gamepad1.x) {
//                yaw = -baseValue - speed / 3;
//            }
//            if (gamepad1.b) {
//                yaw = +baseValue + speed / 3;
//            }

            //field centric

            if (gamepad1.yWasPressed()) {
                fieldCentricDrive = true;
            }
            if (gamepad1.aWasPressed()) {
                fieldCentricDrive = false;
            }

            //send commands to pedro.

            if (!fieldCentricDrive) {

                follower.setTeleOpDrive(
                        axial,
                        lateral,
                        yaw,
                        true // Robot Centric
                );
            } else if (IsRed == false) {
                follower.setTeleOpDrive(
                        //works i think
                        -axial,
                        -lateral,
                        yaw,
                        false // Robot Centric
                );
            } else if (IsRed == true) {
                follower.setTeleOpDrive(

                        axial,
                        lateral,
                        yaw,
                        false // Robot Centric
                );
            }
        }

        // ----- automated Driving ---
        if (gamepad1.backWasPressed()){

            PathChain GoToParkPos = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), parkPos))
                    .setLinearHeadingInterpolation(follower.getHeading(), parkPos.getHeading())
                    .build();
            follower.followPath(GoToParkPos,1,true);
            automatedDrive = true;

            lastPoseTriggered = parkPos;

        }

        if (gamepad1.dpadUpWasPressed()){

            PathChain GoToFrontShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), shootFrontPos))
                    .setLinearHeadingInterpolation(follower.getHeading(), follower.getHeading())
                    .build();
            follower.followPath(GoToFrontShootPos,1,true);
            automatedDrive = true;

            lastPoseTriggered = shootFrontPos;

        }

        if (gamepad1.dpadDownWasPressed()){

            PathChain GoToBackShootPos = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), shootBackPos))
                    .setLinearHeadingInterpolation(follower.getHeading(),shootBackPos.getHeading())
                    .build();
            follower.followPath(GoToBackShootPos,1,true);
            automatedDrive = true;

            lastPoseTriggered = shootBackPos;
        }
        if (gamepad1.dpadLeftWasPressed()){
            double FinalAngle = Cords.roundToNearest90(follower.getHeading());
            if (!IsRed&&FinalAngle == 180){
                FinalAngle+=90;
            }
            else if (IsRed&&FinalAngle == 0){
                FinalAngle+=90;
            }

            PathChain TriggerClassifierPath = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), TriggerClassifierPos))
                    .setLinearHeadingInterpolation(follower.getHeading(),Math.toRadians(FinalAngle))
                    .build();
            follower.followPath(TriggerClassifierPath,1,true);
            automatedDrive = true;

            lastPoseTriggered = TriggerClassifierPos;
        }


        boolean BreakPaths = false;
        double BreakTolerance=.04;
        if (Math.abs(lateral)>BreakTolerance ||Math.abs(yaw)>BreakTolerance || Math.abs(axial)>BreakTolerance){
            BreakPaths = true;
        }

        if (automatedDrive && (BreakPaths || autoFunctions.isRobotInPositionCustomAmounts(lastPoseTriggered,follower,0,0))) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

    }
    private void handleFlywheel(){
        if (gamepad2.rightStickButtonWasPressed()) {shooter.On();}
        if (gamepad2.leftStickButtonWasPressed()){shooter.Off();}
    }
}
