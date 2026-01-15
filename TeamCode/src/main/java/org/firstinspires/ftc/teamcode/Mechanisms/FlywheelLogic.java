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
        LAUNCH_BALL,
        DETECT_IF_BALL_LAUNCHED,
        RESET
    }

    private FlywheelState flywheelState;

    // ----------- FEEDER CONSTANTS -------------
    //public static double MAX_SHOOT_BALL_TIME = 7;
    public static double MAX_SHOOT_BALL_TIME = 5;

    //------------- FLYWHEEL CONSTANTS ------------

    // ------------- adjust depending on flywheel accuracy, it will change how much the flywheel slows down, which u can detect.
    public static double FLYWHEEL_AFTER_SHOT_SLOWDOWN = 100;
    private boolean flywheel_on = true;


    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    public static double TARGET_FLYWHEEL_TPS = 1300;
    public static double FLYWHEEL_MAX_SPINUP_TIME = 6;

    public void init(HardwareMap hardwareMap) {
        FAndV = new FunctionsAndValues();


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
        SpinBallFeeder(0);
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

    public void updateWithStateMachine(boolean isRobotReadyToShoot){
        update();
        if(flywheel_on){SetMotorPowerToTarget();}
        else{TurnFlywheelOff();}

        if (shotsRemaining>0){flywheel_on=true;}

        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH_BALL;
                }
                break;

            case LAUNCH_BALL:
                if (IsFlywheelUpToSpeed() && isRobotReadyToShoot || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    SpinBallFeeder(1);
                    stateTimer.reset();
                    flywheelState = FlywheelState.DETECT_IF_BALL_LAUNCHED;
                }
                break;
            case DETECT_IF_BALL_LAUNCHED:
                if (stateTimer.seconds() > MAX_SHOOT_BALL_TIME || IsFlywheelSlowedFromShot()) {
                    shotsRemaining -= 1;
                    SpinBallFeeder(0);
                    stateTimer.reset();

                    flywheelState = FlywheelState.RESET;
                }
                break;
            case RESET:
                if (shotsRemaining > 0) {
                    stateTimer.reset();
                    flywheelState = FlywheelState.LAUNCH_BALL;
                } else {
                    stateTimer.reset();
                    flywheelState = FlywheelState.IDLE;
                }
                break;

        }
    }

    public void update() {
        flywheelVelocity = FAndV.GetSpeedAvgFromTwoMotors(ShooterMotor.getVelocity(), ShooterMotor2.getVelocity());



    }

    public double GetShotsRemaining() {
        return shotsRemaining;
    }

    public void Stop(){
        shotsRemaining = 0;
        flywheel_on=false;
        flywheelState = FlywheelState.IDLE;
    }

    public void fireShots(int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    public String GetState(){return flywheelState.name();}

    public boolean isBusy() {
        boolean isStateBusy = flywheelState != FlywheelState.IDLE;
        boolean isShootingDone = shotsRemaining <= 0;
        if (isStateBusy && isShootingDone) {
            return false;
        } else {
            return true;
        }
    }
}
