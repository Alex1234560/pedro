/* Copyright (c) 2021 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */


package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Mechanisms.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.TurretRotation;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
//
public class CleanTeleop extends LinearOpMode {
    // Hardware Setup Variables
    private Servo ServoShooter1;
    private Servo ReadyToShootServo;
    private static double ShooterAngle = FunctionsAndValues.startPoint;
    //setting up motors and time
    private ElapsedTime runtime = new ElapsedTime();
    //private DcMotorEx IntakeMotor = null;
    //private CRServo StopIntakeServo = null;
    private DcMotorEx ShooterMotor = null;
    private DcMotorEx ShooterMotor2 = null;
    private CRServo BallFeederServo = null;
    private CRServo BallFeederServo2 = null;

    //private Servo ShooterRotatorServo = null;
    private IMU imu;

    //classes
    //Vision !!!
    private double startingAngleRad = Math.toRadians(0);
    private AprilTagVision vision;

    private Intake intake = new Intake();


    //pedro stuff
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this

    //private TelemetryManager telemetryM;

    private FunctionsAndValues FAndV;

    public static boolean fieldCentricDrive = false;

    // angle for hooded shooter
    double HoodAngle = 0;// value from 0 to 1.0

    //april tag stuff
    private double AprilTagBearing = 0;

    private boolean SelfAimToggle = true;
    public static double currentAngle = 90;

    //Variables for statement printing
    private static double ShooterMotorPower = 0;
    public static double GoalShooterMotorTPS = 1200;// rotation ticks per seond

    //variables avobe for testing
    public double shooterTPS = 0; // Ticks per second
    public double range = 0;

    // --- Button Variables For Shooter ---
    private boolean shooterMotorOn = false;      // Tracks if the motor should be on or off

    //declaring button globally
    //private boolean autoAimButton = false;
    public static boolean AutoAim = false;

    private TurretRotation turretRotation = new TurretRotation();

    @Override
    public void runOpMode() {
        currentAngle=90;//center current angle for shooter
        InitializeIMU();
        SetupHardware();

        turretRotation.init(hardwareMap);
        // remove the following once the turret stuff is integrated into the auto, this will go in the auto
        turretRotation.TurretCalibrateToCenter();
        //--------------- ^^ -------------------------

        vision = new AprilTagVision(hardwareMap, "Webcam");
        intake.init(hardwareMap);
        //pedro stuff
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0,0,Math.toRadians(0)));
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

        vision.setManualExposure(AprilTagVision.myExposure, AprilTagVision.myGain);
        imu.resetYaw();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //pedro
            turretRotation.update(Math.toDegrees(follower.getHeading()),follower.getPose().getX(),follower.getPose().getY());
            follower.update();
            //telemetryM.update();

            if (gamepad2.aWasPressed()) {AutoAim = true;}
            if (gamepad2.bWasPressed()) {AutoAim = false;}

            vision.update();
            updateBearing();
            handleDriving();
            handleIntake();
            handleShooterServos();
            handleFlywheel();
            //handleShooterRotation();
            SpeedAndAngleAutoAimUpdate();
            TelemetryStatements();
            handleUserShootingRanges();
            updateReadyToShoot();

            //telemetryM.debug("position", follower.getPose());
            //telemetryM.debug("velocity", follower.getVelocity());

            //autoLock();
        }
    }

    private void updateReadyToShoot(){
        //(Math.abs(GoalShooterMotorTPS - shooterTPS) <= FunctionsAndValues.SpeedToleranceToStartShooting){
        if (vision.isTagVisible()) {
            if (Math.abs(AprilTagBearing) < FunctionsAndValues.AngleToleranceToStartShooting && shooterTPS > FunctionsAndValues.MinimumSpeed) {
                ReadyToShootServo.setPosition(1);
            } else {
                ReadyToShootServo.setPosition(.66666666);
            }
        }
        else{
            ReadyToShootServo.setPosition(.33333);
        }
}

    private void TelemetryStatements(){
        telemetry.addData("FieldCentricDrive?: ", fieldCentricDrive);
        telemetry.addData("EncoderPosition", turretRotation.GetCurrentPos());
        telemetry.addData("Heading", follower.getHeading());

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());

        telemetry.addData("GoalShooterPower= ", GoalShooterMotorTPS);
        telemetry.addData("ShooterMotorTickPerSecond= ", shooterTPS);



        telemetry.update();
    }
    /*
    private void handleShooterRotation(){
        //this function will return current value unless able to adjust it, with autoaim and autoaim activated
        double[] ShooterRotatorServoAngle = FAndV.calculateShooterRotation(AprilTagBearing, AutoAim,currentAngle,false , range);
        ShooterRotatorServo.setPosition(ShooterRotatorServoAngle[0]);
        currentAngle = ShooterRotatorServoAngle[1];
        AprilTagBearing = ShooterRotatorServoAngle[2];

        //if (gamepad2.dpad_left && currentAngle < 180) {currentAngle += 2;}
        //if (gamepad2.dpad_right && currentAngle > 0) {currentAngle -= 2;}
        currentAngle -= gamepad2.right_stick_x*5;
        if (currentAngle > 180) {currentAngle = 180;}
        if (currentAngle < 0) {currentAngle = 0;}

        if (gamepad2.right_stick_button){currentAngle = 90;}

    }*/

    private void SpeedAndAngleAutoAimUpdate(){

        if (vision.getRange()!=-1) {
            range = vision.getRange();
        }


        if (AutoAim){
            double[] shooterGoals = FAndV.handleShootingRanges(range);
            ShooterAngle = shooterGoals[0];
            GoalShooterMotorTPS = shooterGoals[1];

        }
    }

    private void handleUserShootingRanges(){
        if (gamepad2.left_bumper){
            AutoAim = false;
            //ShooterRotatorServo.setPosition(.5);
            currentAngle = 90;
            GoalShooterMotorTPS = 1025;
            ShooterAngle = .15;
        }

        if (gamepad2.dpad_left){

        }
        if (gamepad2.dpad_right){

        }
    }

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
    }

    private void handleIntake() {
        // handle feeding to shooter
        double ShootMechanismPower = 0; //Positive value = shooting, negative value = retract balls and spit them out

        if (!gamepad2.right_bumper && gamepad2.right_trigger > 0 && Math.abs(GoalShooterMotorTPS - shooterTPS) <= FunctionsAndValues.SpeedToleranceToStartShooting) {//(Math.abs(GoalShooterMotorTPS - shooterTPS) <= ToleranceForShooting)
            ShootMechanismPower=1;
//            if (vision.isTagVisible() && Math.abs(AprilTagBearing) > FunctionsAndValues.AngleToleranceToStartShooting){
//                // should make it so that if ur scanning the code and bearing is more than blah blah, it doesnt let u shoot
//                ShootMechanismPower=0;
//            }
        }

        else if (gamepad2.right_bumper&& gamepad2.right_trigger > 0) {
            ShootMechanismPower=1;
        }

        else if(gamepad2.back) {
            ShootMechanismPower=-1;
        }


        //IntakeMotor.setPower(-gamepad2.right_trigger * ShootMechanismPower);
        //StopIntakeMotor.setPower(gamepad2.right_trigger *ShootMechanismPower);
        BallFeederServo.setPower(gamepad2.right_trigger * ShootMechanismPower);
        if (ShootMechanismPower==-1){BallFeederServo.setPower(-1);}
        BallFeederServo2.setPower(BallFeederServo.getPower());


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

        follower.setTeleOpDrive(
                axial,
                lateral,
                yaw,
                !fieldCentricDrive // Robot Centric
        );




    }

    private void handleShooterServos() {

        // Initialize shooterAngle with the servo's current position to start.
        // This ensures it always has a value.

        ShooterAngle-=gamepad2.left_stick_y/22;
        //normalize
        if (ShooterAngle> FunctionsAndValues.endPoint){ShooterAngle= FunctionsAndValues.endPoint;}
        if (ShooterAngle< FunctionsAndValues.startPoint){ShooterAngle= FunctionsAndValues.startPoint;}

        ServoShooter1.setPosition(ShooterAngle);
        telemetry.addData("ServoAngle ", ShooterAngle );

    }

    private void updateBearing(){
        if (vision.isTagVisible()) {
            //telemetry.addData("ID", vision.getID());
            if (vision.getID() == 20 || vision.getID() == 24) {
                telemetry.addData("GOAL TAG?: ", vision.isTagVisible());
                telemetry.addData("Range", "%.2f in", vision.getRange());
                //telemetry.addData("Yaw", "%.2f deg", vision.getYaw());
                //telemetry.addData("X Offset", "%.2f in", vision.getX());
                AprilTagBearing = vision.getBearing();
            }
        }
        telemetry.addData("Bearing", "%.2f Deg", AprilTagBearing);
    }

    private void InitializeIMU() {
        // Drivetrain & IMU
        imu = hardwareMap.get(IMU.class, "imu");

        // Configure the IMU. This is critical for field-centric drive.
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);

    }

    private void SetupHardware(){
        //IntakeMotor = hardwareMap.get(DcMotorEx.class, "INTAKE");
        //StopIntakeServo = hardwareMap.get(DcMotor.class, "StopIntake");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
        BallFeederServo2 = hardwareMap.get(CRServo.class, "BallFeederServo2");
        //ShooterRotatorServo = hardwareMap.get(Servo.class, "ShooterRotatorServo");
        ServoShooter1 = hardwareMap.get(Servo.class, "ServoShooter1");
        ReadyToShootServo = hardwareMap.get(Servo.class, "IndicatorServo");


        //directions
        //BallFeederServo2.setDirection(CRServo.Direction.REVERSE);
        //ServoShooter1.setDirection(Servo.Direction.REVERSE);
        ShooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        // run shooter with encoder
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //IntakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

  }
