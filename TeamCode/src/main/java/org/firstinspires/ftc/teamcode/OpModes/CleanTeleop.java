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
import org.firstinspires.ftc.teamcode.Functions.ShootingInterpolation;
import org.firstinspires.ftc.teamcode.Mechanisms.AprilTagVision;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelAndFeederLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.HoodAngle;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp
public class CleanTeleop extends OpMode {
    //timer
    private ElapsedTime runtime = new ElapsedTime();

    // Setup for classes
    private AutoFunctions autoFunctions = new AutoFunctions();
    private Coordinates Cords = new Coordinates();
    private FunctionsAndValues FAndV = new FunctionsAndValues();
    private Intake intake = new Intake();
    private FlywheelAndFeederLogic shooter = new FlywheelAndFeederLogic();
    private HoodAngle hood = new HoodAngle();
    private AprilTagVision camera;
    private TurretRotation turretRotation = new TurretRotation();

    //pedro stuff
    private Follower follower;
    private TelemetryManager telemetryM;

    //other
    private double FlywheelSpeedForTuning;
    private double FlywheelSpeedForTuningOffset;
    private double HoodAngleOffset;
    private double HoodAngle = org.firstinspires.ftc.teamcode.Mechanisms.HoodAngle.START_POINT;

    private boolean ManuallyAdjustableValues = false;
    private boolean fieldCentricDrive = true;
    //private boolean UseOdosForSpeedAndDistance = true;
    private boolean automatedDrive = false;
    private boolean tuningTelemetry = false;
    private boolean IsRed = false;
    private boolean FastMode = true;
    private boolean start_program_witouth_auto_first = true;

    //buttons
    private boolean reset_position_button_pressed = false;
    private boolean WasXPressed;

    //poses
    Pose shootFrontPos, parkPos, shootBackPos, TriggerClassifierPos, lastPoseTriggered;
    private Pose GoalLocationPose, StartingPosition, restartPos;

    @Override
    public void init(){
        // makes it easier in game, less buttons to click so taht u can start right up
        start_program_witouth_auto_first = AutoFunctions.LastPoseRecorded==null;
        AutoFunctions.DidAutoGoToEnd=false;

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
        buildPoses();

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
        HandleColorChooser();
        telemetry.update();
    }

    @Override
    public void loop(){
        follower.update();
        shooter.update();
        camera.update();//if (ManuallyAdjustableValues){camera.update();}
        turretRotation.update(follower, StartingPosition, IsRed);
        turretRotation.handleBearing(camera.getBearing(),camera.getYaw());

        HandleAimingRanges();
        handleDriving();
        handleIntakeAndShootingButtons();
        handleResetPositionFunction();
        TelemetryStatements();
        handleFlywheel();

        WasXPressed=gamepad1.xWasPressed();
    }

    private void buildPoses(){
        shootFrontPos = new Pose(Cords.xFlip(57, IsRed), 95, Math.toRadians(Cords.angleFlip(180, IsRed)));
        shootBackPos = new Pose(Cords.xFlip(57, !IsRed), 18, Math.toRadians(Cords.angleFlip(0, IsRed)));
        parkPos = new Pose(Cords.xFlip(38.71049304677624, !IsRed), 33.29329962073322, Math.toRadians(Cords.angleFlip(180, IsRed)));
        TriggerClassifierPos = new Pose(Cords.xFlip(16.5, IsRed), 70, Math.toRadians(Cords.angleFlip(90, IsRed)));

        restartPos = new Pose(Cords.xFlip(Coordinates.RESTART_X,IsRed), Coordinates.RESTART_Y, Math.toRadians(Cords.angleFlip(0, IsRed)));

        GoalLocationPose = new Pose(Cords.xFlip(Coordinates.GOAL_X,IsRed), Coordinates.GOAL_Y, Math.toRadians(0));
    }
    private void handleResetPositionFunction(){
        //reset_position_button_pressed = gamepad1.right_bumper;
        reset_position_button_pressed = gamepad1.right_bumper&&gamepad1.left_bumper;

        if ((reset_position_button_pressed&&WasXPressed)||(reset_position_button_pressed&&gamepad1.bWasPressed())) {
            if (gamepad1.x) {IsRed = false;} // blue
            else if (gamepad1.b) {IsRed = true;} //red
            turretRotation.resetTotalHeadingForRobotAndTurret(follower.getTotalHeading());
            buildPoses();
            StartingPosition=restartPos;
            follower.setPose(restartPos);

            ManuallyAdjustableValues=false;
            turretRotation.ManualTurretControl(false);
            //UseOdosForSpeedAndDistance = true;
            fieldCentricDrive=true;
        }
    }
    private void TelemetryStatements(){
        if (gamepad2.dpadLeftWasPressed()){tuningTelemetry=!tuningTelemetry;}
        if (tuningTelemetry) {
            telemetryM.addData("Turret Finished Rotating?:  ", turretRotation.isTurretFinishedRotating());
            telemetryM.addData("TuningMode?:  ", ManuallyAdjustableValues);
            telemetryM.addData("Ball Count Distance Sensor ", shooter.GetBallsShotCount());
            telemetryM.addData("Turret Rotation Ticks/Sec ", Math.round(turretRotation.GetCurrentVel()));

            telemetryM.addData("Target Angle ", Math.round(turretRotation.GetTargetAngle()));
            telemetryM.addData("Distance Sensor value: ", shooter.GetDistance());
            telemetryM.addData("Bearing " ,camera.getBearing());
            telemetryM.addData("Yaw " ,camera.getYaw());
            telemetryM.addData("Distance (camera) " ,camera.getRangeEquivalentToOdoRange());

        }

        //telemetryM.addData("GOAL_Y FOR DEBUGGING ", turretRotation.ReturnGoalY());
        telemetryM.addData("FieldCentricDrive?: ", fieldCentricDrive);
        telemetryM.addData("shooter Goal Speed ", FlywheelAndFeederLogic.TARGET_FLYWHEEL_TPS);
        telemetryM.addData("Is flywheel up to speed?:  ", shooter.IsFlywheelUpToSpeed());
        telemetryM.addData("Distance From Goal ", turretRotation.GetDistanceFromGoal(IsRed));
        telemetryM.addData("Hood Angle", hood.getPosition());
        telemetryM.addData("Flywheel Speed" ,shooter.GetFlywheelSpeed());
        if (IsRed){
        telemetryM.addData("RED SIDE OF THE FIELD" ,shooter.GetFlywheelSpeed());}
        if (!IsRed){
            telemetryM.addData("BLUE SIDE OF THE FIELD" ,shooter.GetFlywheelSpeed());}



        telemetryM.debug("x Velocity:" + Math.round(follower.getVelocity().getXComponent()));
        telemetryM.debug("y Velocity:" + Math.round(follower.getVelocity().getYComponent()));
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

        boolean IntakeReversing = gamepad2.left_bumper || (gamepad1.left_bumper&&!reset_position_button_pressed);

        //intaking
        boolean shootButton = false;
        if (gamepad2.y){shootButton=true;}
        if (gamepad1.y){shootButton=true;}



        double IntakePowerValue = Math.abs(gamepad2.left_trigger);
        if (Math.abs(gamepad1.left_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = Math.abs(gamepad1.left_trigger);
        }
        if (Math.abs(gamepad2.right_trigger) > Math.abs(gamepad2.left_trigger)){
            IntakePowerValue = Math.abs(gamepad2.right_trigger);
        }
        if (shootButton){IntakePowerValue = 1;}


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
        boolean Trigger = shootButton;

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
    private void HandleAimingRanges(){

    // ----- everything below for manually adjustable values ----- including turret.
    if (gamepad2.start && gamepad2.dpadUpWasPressed()){
        ManuallyAdjustableValues=!ManuallyAdjustableValues;
        HoodAngle = hood.getPosition();
        FlywheelSpeedForTuning = FlywheelAndFeederLogic.TARGET_FLYWHEEL_TPS;
        FlywheelSpeedForTuningOffset = 0;
        HoodAngleOffset = 0;

        turretRotation.ManualTurretControl(ManuallyAdjustableValues);
        //UseOdosForSpeedAndDistance = !UseOdosForSpeedAndDistance;
        fieldCentricDrive=!fieldCentricDrive;
    }



    if (ManuallyAdjustableValues){
        FAndV.setSpeedTolerance(FunctionsAndValues.SPEED_TOLERANCE_TO_SHOOT_FRONT);
        if (gamepad2.leftStickButtonWasPressed()){HoodAngleOffset = 0;}
        if (gamepad2.rightStickButtonWasPressed()){FlywheelSpeedForTuningOffset = 0;}
        HoodAngleOffset -= gamepad2.left_stick_y / 40;

        if (camera.getRangeEquivalentToOdoRange()!=-1){
            double[] turretGoals = ShootingInterpolation.get(camera.getRangeEquivalentToOdoRange());
            HoodAngle=turretGoals[0];
            FlywheelSpeedForTuning=turretGoals[1];
        }

        HoodAngle=hood.normalize(HoodAngle);

        hood.SetPosition(HoodAngle+HoodAngleOffset);

        if (gamepad2.dpadUpWasPressed()){
            FlywheelSpeedForTuningOffset+=25;
        }
        if (gamepad2.dpadDownWasPressed()){
            FlywheelSpeedForTuningOffset-=25;
        }
        shooter.setFlywheelTPS(FlywheelSpeedForTuning+FlywheelSpeedForTuningOffset);

        turretRotation.updateManualOffset(gamepad2.right_stick_x*5);
    }
    // --------------------------------------------------- /

    else {

        double distance = turretRotation.GetDistanceFromGoal(IsRed);
        double[] turretGoals = ShootingInterpolation.get(distance);
        hood.SetPosition(turretGoals[0]);
        shooter.setFlywheelTPS(turretGoals[1]);
    }
}
    private void handleDriving() {
        double speed =1; //
        double speedModifier = gamepad1.right_trigger / 2;

        //if (SlowMode){speed=.1+speedModifier/1.5;}
        if (FastMode){speed -= speedModifier*1.75;}

        double axial = -gamepad1.left_stick_y * speed;
        double lateral = -gamepad1.left_stick_x * speed; // Note: pushing stick forward gives negative value
        double yaw = -gamepad1.right_stick_x * speed;



        if (!automatedDrive) {

            if (WasXPressed&&!reset_position_button_pressed) {
                fieldCentricDrive = !fieldCentricDrive;
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
        if (gamepad1.backWasPressed()&&gamepad1.start){

            PathChain GoToParkPos = follower.pathBuilder()
                    .addPath(new BezierLine(follower.getPose(), parkPos))
                    .setLinearHeadingInterpolation(follower.getHeading(), parkPos.getHeading())
                    .build();
            follower.followPath(GoToParkPos,1,true);
            automatedDrive = true;

            lastPoseTriggered = parkPos;

        }

//        if (gamepad1.dpadUpWasPressed()){
//
//            PathChain GoToFrontShootPos = follower.pathBuilder()
//                    .addPath(new BezierLine(follower.getPose(), shootFrontPos))
//                    .setLinearHeadingInterpolation(follower.getHeading(), follower.getHeading())
//                    .build();
//            follower.followPath(GoToFrontShootPos,1,true);
//            automatedDrive = true;
//
//            lastPoseTriggered = shootFrontPos;
//
//        }

//        if (gamepad1.dpadDownWasPressed()){
//
//            PathChain GoToBackShootPos = follower.pathBuilder()
//                    .addPath(new BezierLine(follower.getPose(), shootBackPos))
//                    .setLinearHeadingInterpolation(follower.getHeading(),shootBackPos.getHeading())
//                    .build();
//            follower.followPath(GoToBackShootPos,1,true);
//            automatedDrive = true;
//
//            lastPoseTriggered = shootBackPos;
//        }
//        if (gamepad1.dpadLeftWasPressed()){
//            double FinalAngle = Cords.roundToNearest90(follower.getHeading());
//            if (!IsRed&&FinalAngle == 180){
//                FinalAngle+=90;
//            }
//            else if (IsRed&&FinalAngle == 0){
//                FinalAngle+=90;
//            }
//
//            PathChain TriggerClassifierPath = follower.pathBuilder()
//                    .addPath(new BezierLine(follower.getPose(), TriggerClassifierPos))
//                    .setLinearHeadingInterpolation(follower.getHeading(),Math.toRadians(FinalAngle))
//                    .build();
//            follower.followPath(TriggerClassifierPath,1,true);
//            automatedDrive = true;
//
//            lastPoseTriggered = TriggerClassifierPos;
//        }


        boolean BreakPaths = false;
        double BreakTolerance=.05;
        if (Math.abs(lateral)>BreakTolerance ||Math.abs(yaw)>BreakTolerance || Math.abs(axial)>BreakTolerance){
            BreakPaths = true;
        }

        if (automatedDrive && (BreakPaths)) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

    }
    private void handleFlywheel(){
        if (gamepad2.aWasPressed()) {shooter.On();}
        if (gamepad2.bWasPressed()&&!gamepad2.start){shooter.Off();}
    }
    private void HandleColorChooser(){

        // in case you're starting teleop from fresh just for practice
        //telemetry.addData("press 'back' to toggle the start program witouth running auto first.", start_program_witouth_auto_first);
        if (gamepad1.backWasPressed()){
            start_program_witouth_auto_first = !start_program_witouth_auto_first;}
        if (!start_program_witouth_auto_first){telemetry.addData("AFTER AUTO ( 'back' to toggle )", "");}
        if (start_program_witouth_auto_first){telemetry.addData("WITOUTH AUTO FIRST ( 'back' to toggle )", "");}

        if (start_program_witouth_auto_first) {
            //telemetry.addData("Alliance Selection", "X for BLUE, B for RED");
            if (IsRed == false) {telemetry.addData("Color: BLUE (b to switch ) ", "");}
            if (IsRed == true) {telemetry.addData("Color: RED (x to switch )", "");}

            if ((gamepad1.x || gamepad2.x)&&!gamepad1.start) {IsRed = false;} // blue
            if ((gamepad1.b || gamepad2.b)&&!gamepad1.start) {IsRed = true;} //red
        }
        else{
            IsRed = AutoFunctions.IsRed;
        }

    }
}
