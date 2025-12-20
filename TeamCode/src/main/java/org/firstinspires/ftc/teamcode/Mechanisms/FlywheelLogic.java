package org.firstinspires.ftc.teamcode.Mechanisms;




import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.FunctionsAndValues;

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
    private double FEEDER_ON_TIME = 1;

    //------------- FLYWHEEL CONSTANTS ------------
    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double TARGET_FLYWHEEL_RPM = 1200;
    private double FLYWHEEL_MAX_SPINUP_TIME = 2;

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

    public void start(){
        ShooterMotor.setPower(FAndV.calculateSpeedForShooter(TARGET_FLYWHEEL_RPM));
        ShooterMotor2.setPower(FAndV.calculateSpeedForShooter(TARGET_FLYWHEEL_RPM));
    }

    public void update(){
        flywheelVelocity= FAndV.GetSpeedAvgFromTwoMotors(ShooterMotor.getVelocity(),ShooterMotor2.getVelocity());

        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {

                    ShooterMotor.setPower(FAndV.calculateSpeedForShooter(TARGET_FLYWHEEL_RPM));
                    ShooterMotor2.setPower(FAndV.calculateSpeedForShooter(TARGET_FLYWHEEL_RPM));

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;

                }
                break;

            case SPIN_UP:

                double power = FAndV.handleShooter(flywheelVelocity, true, TARGET_FLYWHEEL_RPM, ShooterMotor.getPower());
                ShooterMotor.setPower(power);
                ShooterMotor2.setPower(power);

                if (Math.abs(Math.abs(flywheelVelocity) - Math.abs(TARGET_FLYWHEEL_RPM)) < FunctionsAndValues.SpeedToleranceToStartShooting
                        || stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    BallFeederServo.setPower(1);
                    BallFeederServo2.setPower(1);

                    stateTimer.reset();

                    flywheelState = FlywheelState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (stateTimer.seconds() > FEEDER_ON_TIME) {
                    shotsRemaining--;
                    BallFeederServo.setPower(0);
                    BallFeederServo2.setPower(0);

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
                    ShooterMotor.setPower(0);
                    ShooterMotor2.setPower(0);
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




}
