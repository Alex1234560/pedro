package org.firstinspires.ftc.teamcode.Mechanisms;




import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Functions.FunctionsAndValues;

@Configurable
public class FlywheelAndFeederLogic {
    private FunctionsAndValues FAndV;
    private DcMotorEx ShooterMotor = null;
    private DcMotorEx ShooterMotor2 = null;
    private CRServo BallFeederServo = null;
    private CRServo BallFeederServo2 = null;

    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN_UP_FLYWHEEL,
        LAUNCH_BALLS,
        RESET
    }

    public static FlywheelState flywheelState;
    private DistanceSensorClass distanceSensor = new DistanceSensorClass();

    // ----------- FEEDER CONSTANTS -------------
    //public static double MAX_SHOOT_BALL_TIME = 7;
    //public static double MAX_SHOOT_BALL_TIME = 2;
    public static double MAX_SHOOT_TIME_WITOUTH_BALLS = .75;
    public static double MIN_SHOOTING_TIME = 1.2;

    //------------- FLYWHEEL CONSTANTS ------------

    // ------------- adjust depending on flywheel accuracy, it will change how much the flywheel slows down, which u can detect.
    public static double FLYWHEEL_AFTER_SHOT_SLOWDOWN = 100;
    private boolean flywheel_on = true;


    private double shotsRemaining = 0;
    private double flywheelVelocity = 0;
    public static double TARGET_FLYWHEEL_TPS = 1300;
    public static double FLYWHEEL_MAX_SPINUP_TIME = 6;

    private boolean IsBallDetectedAutoBool = true;


    public void init(HardwareMap hardwareMap) {

        FAndV = new FunctionsAndValues();
        distanceSensor.init(hardwareMap);

        BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
        BallFeederServo2 = hardwareMap.get(CRServo.class, "BallFeederServo2");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        ShooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        // run shooter with encoder
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelState = FlywheelState.IDLE;

        ShooterMotor.setPower(0);
        ShooterMotor2.setPower(0);
        BallFeederServo.setPower(0);
        BallFeederServo2.setPower(0);

        distanceSensor.SetBallsShotCount(3);

    }

    public void setFlywheelTPS(double TPS) {
        TARGET_FLYWHEEL_TPS = TPS;
    }

    public void SetMotorPowerToTarget() {
        double power = FAndV.handleShooter(flywheelVelocity, true, TARGET_FLYWHEEL_TPS, ShooterMotor.getPower());
        if (power < 0) {
            power = 0;
        }
        //double power = FAndV.handleShooter(flywheelVelocity, true, TARGET_FLYWHEEL_RPM, ShooterMotor.getPower());
        ShooterMotor.setPower(power);
        ShooterMotor2.setPower(power);
    }

    public void TurnFlywheelOff() {
        ShooterMotor.setPower(0);
        ShooterMotor2.setPower(0);
        flywheelState = FlywheelState.IDLE;

        //NOTE REMOVED: //SpinBallFeeder(0) from this

    }

    public void SpinBallFeeder(double power) {

        BallFeederServo.setPower(power);
        BallFeederServo2.setPower(power);

    }

    public boolean IsFlywheelUpToSpeed() {
        return Math.abs(Math.abs(flywheelVelocity) - Math.abs(TARGET_FLYWHEEL_TPS)) < FunctionsAndValues.SpeedToleranceToStartShooting;
    }

    public boolean IsFlywheelSlowedFromShot() {
        return Math.abs(TARGET_FLYWHEEL_TPS) - Math.abs(flywheelVelocity) > FLYWHEEL_AFTER_SHOT_SLOWDOWN;
    }

    public void start() {

        SetMotorPowerToTarget();
    }

    public double GetFlywheelSpeed() {return flywheelVelocity;}

    public void setShooterState(FlywheelState state){
        flywheelState = state;
        stateTimer.reset();
        shootTimer.reset();
    }

    public void updateWithStateMachine(boolean isRobotReadyToShoot){
        update();
        double ball_feeder_servo_power = 0;
        double balls_shoot = distanceSensor.GetBallsShotCount();

        switch (flywheelState) {
            case IDLE:

                if (balls_shoot<3) {
                    setShooterState( FlywheelState.SPIN_UP_FLYWHEEL);
                }
                break;

            case SPIN_UP_FLYWHEEL:
                if (IsFlywheelUpToSpeed() && isRobotReadyToShoot || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    setShooterState(FlywheelState.LAUNCH_BALLS);
                }
                break;
            case LAUNCH_BALLS:
                if (IsFlywheelUpToSpeed()&& isRobotReadyToShoot){
                 ball_feeder_servo_power=1;}

                if (distanceSensor.IsBallDetected()){shootTimer.reset();}

                if (shootTimer.seconds() > MAX_SHOOT_TIME_WITOUTH_BALLS && stateTimer.seconds()>MIN_SHOOTING_TIME) { // change balls_shot to 3 eventually.
                    setShooterState(FlywheelState.RESET);
                }
                break;

            case RESET:
                setShooterState(FlywheelState.IDLE);
                break;
        }


        //preee feeding balls.
        if (ball_feeder_servo_power==0 && !IsBallDetectedAutoBool){
            ball_feeder_servo_power=FunctionsAndValues.PowerValueForPreloading;
        }

        SpinBallFeeder(ball_feeder_servo_power);
    }

    public void update(){
        if(flywheel_on){SetMotorPowerToTarget();}
        else{TurnFlywheelOff();}

        IsBallDetectedAutoBool = distanceSensor.IsBallDetected();
        flywheelVelocity = FAndV.GetSpeedAvgFromTwoMotors(ShooterMotor.getVelocity(), ShooterMotor2.getVelocity());
        distanceSensor.update();
    }

    public double GetShotsRemaining() {
        return shotsRemaining;
    }


    public void Off(){
        flywheel_on=false;
        flywheelState = FlywheelState.IDLE;
        SpinBallFeeder(0);
    }

    public void On(){
        flywheel_on=true;
    }

    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            //shotsRemaining = numberOfShots;
            distanceSensor.SetBallsShotCount(0);
        }
    }

    public String GetState(){return flywheelState.name();}

    public boolean isBusy() {
        return flywheelState != FlywheelState.IDLE;
    }
    public double GetBallsShotCount(){
        return distanceSensor.GetBallsShotCount();
    }
    public double GetDistance(){
        return distanceSensor.GetDistance();
    }
    public boolean IsBallDetected(){
        return distanceSensor.IsBallDetected();
    }

}
