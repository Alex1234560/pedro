package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.CleanTeleop;
import org.firstinspires.ftc.teamcode.FunctionsAndValues;
@Configurable
public class TurretRotation {
    private DcMotorEx TurretRotatorMotor = null;



//    public static double MAX_TICKS_ROTATION = 1000;
//    public static double MIN_TICKS_ROTATION = -1000;


    public static double MotorPowerWitouthAutoDebugging = 0;


    public static boolean AutoRotate = true;
    public static boolean TrackGOAL = false;

    public static boolean LimitVelocitySwitches = false;
    public static double DONT_SWITCH_VALUE = 800;

    public static double FULL_TURN = 1666;// ticks that make a full turn

    public static double GoalAngle = 180;

    public static double SWITCH_ANGLE = 0;

    private double ActualTargetAngle = 0;



    public static double kP = 0.004;
    public static double kI = 0.0005;
    public static double kD = 0.005;
    public static double kF = 0.025;

    SimplePIDF RotationalPIDF = new SimplePIDF(
            kP,  // kP  (start small)
            kI,     // kI  (usually 0)
            kD,     // kD  (often 0)
            kF
    );




    public void init(HardwareMap hardwareMap){
        TurretRotatorMotor = hardwareMap.get(DcMotorEx.class, "TurretRotator");
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double normalizeDeg(double deg){
        double newDeg = deg;
        if (newDeg>360){newDeg-=360;}
        if (newDeg<0){newDeg+=360;}

        return newDeg;

    }
    //public void start(){}

    public void update(double RobotAngleDeg, Pose robotPose, Pose goalPose){


            //i think ti doesnt handle floats well, so im rounding it.
            int IntRobotAngleDeg = (int) Math.round( RobotAngleDeg );//- CleanTeleop.StartingPosition.getHeading());

            double CurrentPos = GetCurrentPos();
            double CurrentVel = GetCurrentVel();

            //telemetry.addData("CurrentPos ", CurrentPos);
            ActualTargetAngle = GoalAngle;

            if (AutoRotate) { ActualTargetAngle +=IntRobotAngleDeg;}


            if (TrackGOAL){
                ActualTargetAngle += (90 - getM(goalPose.getX(), goalPose.getY(), robotPose.getX(), robotPose.getY()));
            }

            //ActualTargetAngle = normalizeDeg(ActualTargetAngle);

            if (ActualTargetAngle > SWITCH_ANGLE) {
                ActualTargetAngle -= 360;
            }
            if (ActualTargetAngle < -SWITCH_ANGLE) {
                ActualTargetAngle += 360;
            }

            double GoalTickPos = (FULL_TURN / 360) * -(180-ActualTargetAngle); // the 180 makes it so that it normalizes the value hopefully so it knows that its being started at the back

            // -------- line below is for tuning values ----------
            RotationalPIDF.updateCoefficients(kP,kI,kD,kF);

            double newPower = RotationalPIDF.calculate(GoalTickPos, CurrentPos);

            // -------------- LOGIC FOR NO SUDDEN DIRECTION CHANGE -------------------  -----

            if (LimitVelocitySwitches){

                double sign = 0;
                if (newPower != 0) {
                    sign = newPower / Math.abs(newPower);
                }
                ;
                double lastSign = 0;
                double TurretLastPower = TurretRotatorMotor.getPower();
                if (TurretLastPower != 0) {
                    lastSign = TurretLastPower / Math.abs(TurretLastPower);
                }

                if (Math.abs(CurrentVel) > DONT_SWITCH_VALUE) {
                    if (sign != lastSign && sign != 0) {
                        newPower = 0;
                    }
                }
            }
            // ---------- setting power to motor -----------

            TurretRotatorMotor.setPower(newPower);






        }





    public void TurretCalibrateToCenter(){
        TurretRotatorMotor.setPower(0);
        TurretRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public double GetTargetAngle(){
        return ActualTargetAngle;
    }

    public double GetCurrentPos(){
        return TurretRotatorMotor.getCurrentPosition();
    }
    public double GetCurrentVel(){
        return TurretRotatorMotor.getVelocity();
    }

    public double getM(double goalPosx, double goalPosy, double curPosx, double curPosy){

        double x = goalPosx - curPosx;
        double y = goalPosy - curPosy;

        double angleRad = Math.atan2(y, x);          // angle from +X axis
        double angleDeg = Math.toDegrees(angleRad);  // convert to degrees

        // normalize to 0–360
        if (angleDeg < 0) {
            angleDeg += 360;
        }

        return angleDeg;
    }


    public static class SimplePIDF {
        public double kP, kI, kD,kF;

        private double integral = 0;
        private double lastError = 0;

        public SimplePIDF(double kP, double kI, double kD,double kF) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.kF = kF;

        }

        public void updateCoefficients(double p, double i, double d, double f) {
            this.kP = p;
            this.kI = i;
            this.kD = d;
            this.kF = f;
        }


        public double calculate(double target, double current) {
            double error = target - current;

            // P
            double p = kP * error;

            // I
            // 2. I Term (with safety)
            // Only accumulate integral if the error is small enough to need a "boost"
            // but large enough that it's not just sensor noise.

            if (Math.abs(error) < 100 && Math.abs(error) > 1) {
                integral += error;
            } else {
                integral = 0; // Reset it if we are far away or already there
            }

            double i = kI * integral;

            // D
            double d = kD * (error - lastError);
            lastError = error;

            // F
            double f = kF * Math.signum(error);  // feedforward based on target RPM

            double output = p + i + d + f;
            return Math.max(-1.0, Math.min(1.0, output));
        }
    }
}