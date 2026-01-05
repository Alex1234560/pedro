package org.firstinspires.ftc.teamcode.Mechanisms;




import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FunctionsAndValues;

@Configurable
public class FlywheelLogic {
    private FunctionsAndValues FAndV;
    private DcMotorEx ShooterMotor = null;
    private DcMotorEx ShooterMotor2 = null;
    private CRServo BallFeederServo = null;
    private CRServo BallFeederServo2 = null;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET
    }

    private FlywheelState flywheelState;

    // ----------- FEEDER CONSTANTS -------------
    public static double FEEDER_ON_TIME = 3;

    //------------- FLYWHEEL CONSTANTS ------------

    // ------------- adjust depending on flywheel accuracy, it will change how much the flywheel slows down, which u can detect.
    public static double FLYWHEEL_AFTER_SHOT_SLOWDOWN = 100;


    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    public static double TARGET_FLYWHEEL_RPM = 1300;
    public static double FLYWHEEL_MAX_SPINUP_TIME = 2.5;
    //so flywheel is stable when starting to spin up, cuz it overshoots, and it allows it to launch but then it exceeds it cuz it overshoots.
    public static double FLYWHEEL_MIN_SPINUP_TIME = .5;

    public void init(HardwareMap hardwareMap){
        FAndV = new FunctionsAndValues();


        BallFeederServo = hardwareMap.get(CRServo.class, "BallFeederServo");
        BallFeederServo2 = hardwareMap.get(CRServo.class, "BallFeederServo2");
        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");

        ShooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        // run shooter with encoder
        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelState= FlywheelState.IDLE;

        ShooterMotor.setPower(0);
        ShooterMotor2.setPower(0);
        BallFeederServo.setPower(0);
        BallFeederServo2.setPower(0);

    }

    public void setFlywheelRPM(double RPM){
        TARGET_FLYWHEEL_RPM = RPM;
    }

    public void SetMotorPowerToTarget(){
        double power = FAndV.handleShooter(flywheelVelocity,true,TARGET_FLYWHEEL_RPM,ShooterMotor.getPower());
        if (power<0){power=0;}
        //double power = FAndV.handleShooter(flywheelVelocity, true, TARGET_FLYWHEEL_RPM, ShooterMotor.getPower());
        ShooterMotor.setPower(power);
        ShooterMotor2.setPower(power);
    }

    public void TurnFlywheelOff() {
        ShooterMotor.setPower(0);
        ShooterMotor2.setPower(0);
        flywheelState = FlywheelState.IDLE;
        SpinBallFeeder(0);
    }

    public void SpinBallFeeder(double power){

        BallFeederServo.setPower(power);
        BallFeederServo2.setPower(power);

    }

    public boolean IsFlywheelUpToSpeed(){
        return Math.abs(Math.abs(flywheelVelocity) - Math.abs(TARGET_FLYWHEEL_RPM)) < FunctionsAndValues.SpeedToleranceToStartShooting;
    }

    public boolean IsFlywheelSlowedFromShot(){
        return Math.abs(TARGET_FLYWHEEL_RPM) - Math.abs(flywheelVelocity)  > FLYWHEEL_AFTER_SHOT_SLOWDOWN;
    }

    public void start(){
        SetMotorPowerToTarget();
    }

    public double GetFlywheelSpeed(){
        return flywheelVelocity;
    }

    public void update(){
        flywheelVelocity = FAndV.GetSpeedAvgFromTwoMotors(ShooterMotor.getVelocity(),ShooterMotor2.getVelocity());

        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    SetMotorPowerToTarget();
                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                SetMotorPowerToTarget();
                if (IsFlywheelUpToSpeed() && stateTimer.seconds() > FLYWHEEL_MIN_SPINUP_TIME|| stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    SpinBallFeeder(1);
                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > FEEDER_ON_TIME || IsFlywheelSlowedFromShot() ){
                    shotsRemaining--;
                    SpinBallFeeder(0);
                    stateTimer.reset();

                    flywheelState = FlywheelState.RESET;
                }
                break;
            case RESET:
                if (shotsRemaining>0){
                    stateTimer.reset();
                    flywheelState= FlywheelState.SPIN_UP;
                }
                else{
                    TurnFlywheelOff();
                    stateTimer.reset();
                }
                break;

        }
    }

    public void fireShots(int numberOfShots){
        if (flywheelState == FlywheelState.IDLE){
            shotsRemaining=numberOfShots;
        }
    }

    public boolean isBusy(){
        return flywheelState != FlywheelState.IDLE;
    }

    public double GetBallFeederPowerForDebugging(){
        return BallFeederServo.getPower();
    }
}
