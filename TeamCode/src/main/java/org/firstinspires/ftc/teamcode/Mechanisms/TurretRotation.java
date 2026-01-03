package org.firstinspires.ftc.teamcode.Mechanisms;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

@Configurable
public class TurretRotation {
    private DcMotorEx TurretRotatorMotor = null;

    public static double MotorPowerWitouthAutoDebugging = 0;


    public static boolean AutoRotate = false;
    public static boolean TrackGOAL = true;
    public static boolean MOTOR_ACTIVE = true;

    public static double AUTO_AIMING_TURRET_OFFSET = 90;
    public static double AUTO_AIMING_DIRECTION = -1;

    public static boolean LimitVelocitySwitches = false;
    private boolean LimitMaxSpeed = true;

    public static double DONT_SWITCH_VALUE = 800;
    public static double MAX_MOTOR_POWER = .5;

    public static double FULL_TURN = 1666;// ticks that make a full turn

    public static double GoalAngle = 0;

    public static double SWITCH_ANGLE_POS = 180;
    public static double SWITCH_ANGLE_NEG = -190;

    private double ActualTargetAngle = 0;
    private double ChangeInAngleAccumulation = 0;



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
        //TurretRotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public double normalizeDeg(double deg){
        double newDeg = deg;
        if (newDeg>SWITCH_ANGLE_POS){newDeg-=360;}
        if (newDeg<SWITCH_ANGLE_NEG){newDeg+=360;}

        return newDeg;

    }
    //public void start(){}

    public void update(double RobotAngleDeg, Pose robotPose, Pose goalPose){




            //i think ti doesnt handle floats well, so im rounding it.
            int IntRobotAngleDeg = (int) Math.round(RobotAngleDeg)  + (int) Math.round(ChangeInAngleAccumulation);//- CleanTeleop.StartingPosition.getHeading());


            double CurrentPos = GetCurrentPos();
            double CurrentVel = GetCurrentVel();

            //telemetry.addData("CurrentPos ", CurrentPos);
            ActualTargetAngle = GoalAngle;

            if (AutoRotate) {
                ActualTargetAngle += IntRobotAngleDeg;
            }


            if (TrackGOAL){
                ActualTargetAngle += (getAngleFromTwoPoints(goalPose.getX(), goalPose.getY(), robotPose.getX(), robotPose.getY()))*AUTO_AIMING_DIRECTION;
            }

            while ( ActualTargetAngle > SWITCH_ANGLE_POS || ActualTargetAngle < SWITCH_ANGLE_NEG) { // so if any of  the other functions mess up, this catches it.
                if (ActualTargetAngle > SWITCH_ANGLE_POS) {
                    ActualTargetAngle -= 360;
                    ChangeInAngleAccumulation -= 360;
                }
                if (ActualTargetAngle < SWITCH_ANGLE_NEG) {
                    ActualTargetAngle += 360;
                    ChangeInAngleAccumulation += 360;
                }
            }

            double GoalTickPos = (FULL_TURN / 360) * ActualTargetAngle; // the 180 makes it so that it normalizes the value hopefully so it knows that its being started at the back

            // -------- line below is for tuning values ----------
            RotationalPIDF.updateCoefficients(kP,kI,kD,kF);

            double newPower = RotationalPIDF.calculate(GoalTickPos, CurrentPos);

            // -------------- LOGIC FOR NO SUDDEN DIRECTION CHANGE -------------------  -----

            double TurretLastPower = TurretRotatorMotor.getPower();
            if (LimitVelocitySwitches){

                double sign = 0;
                if (newPower != 0) {
                    sign = newPower / Math.abs(newPower);
                }
                ;
                double lastSign = 0;

                if (TurretLastPower != 0) {
                    lastSign = TurretLastPower / Math.abs(TurretLastPower);
                }

                if (Math.abs(CurrentVel) > DONT_SWITCH_VALUE) {
                    if (sign != lastSign && sign != 0) {
                        newPower = 0;
                    }
                }

            }

            if (LimitMaxSpeed){
                if (Math.abs(newPower)>MAX_MOTOR_POWER)
                    newPower = MAX_MOTOR_POWER * Math.signum(newPower);
            }
            // ---------- setting power to motor -----------
            if (MOTOR_ACTIVE) {
                TurretRotatorMotor.setPower(newPower);
            }
            else{
                TurretRotatorMotor.setPower(0);
            }





        }

    public void TurretCalibrateToCenter(){
        TurretRotatorMotor.setPower(0);
        TurretRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    public double GetTargetAngle(){return ActualTargetAngle;}

    public double GetDistanceFromGoal(Pose robotPose, Pose goalPose){

        return distance (robotPose.getX(),robotPose.getY(),goalPose.getX(), goalPose.getY());


    }
    public double GetAngleThatIsbeingReturnedForAutoAiming(Pose robotPose, Pose goalPose){

        return getAngleFromTwoPoints(goalPose.getX(), goalPose.getY(), robotPose.getX(), robotPose.getY());
    }

    public double GetCurrentPosDeg(){return (GetCurrentPos()/FULL_TURN)*360;}
    public double GetCurrentPos(){return TurretRotatorMotor.getCurrentPosition();}
    public double GetCurrentVel(){return TurretRotatorMotor.getVelocity();}

    //logic for unwrapping angle for turret autoaiming so that it can rotate more than 180 on each side
    double lastAngle;
    double unwrappedAngle;

    public double getContinuousAngle(double curAngleDegrees) {

        double delta = curAngleDegrees - lastAngle;

        // unwrap through ±180 boundary
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        unwrappedAngle += delta;

        lastAngle = curAngleDegrees;

        if (curAngleDegrees<SWITCH_ANGLE_NEG){
            unwrappedAngle += 360;
        }
        if (curAngleDegrees>SWITCH_ANGLE_POS){
            unwrappedAngle -= 360;
        }

        return unwrappedAngle;
    }

    // ^^ ----- ^^

    public double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.hypot(dx, dy);   // safer and avoids overflow
    }

//    public double getAngleFromTwoPoints(double goalPosx, double goalPosy, double curPosx, double curPosy){
//
//        double x = goalPosx - curPosx;
//        double y = goalPosy - curPosy;
//
//        double angleRad = Math.atan2(y, x);          // angle from +X axis
//        double angleDeg = Math.toDegrees(angleRad);  // convert to degrees
//
//        angleDeg+=AUTO_AIMING_TURRET_OFFSET;
//        angleDeg = getContinuousAngle(angleDeg);
//
//        // normalize to 0–360
//
//
//        return angleDeg;
//    }
    public double getAngleFromTwoPoints(double goalPosx, double goalPosy,
                                    double curPosx, double curPosy) {

        double dx = goalPosx - curPosx;
        double dy = goalPosy - curPosy;

        double angleRad = Math.atan2(dy, dx);          // -180..180
        double angleDeg = Math.toDegrees(angleRad);

        angleDeg += AUTO_AIMING_TURRET_OFFSET;         // account for turret mount

        // Normalize into your working range [-180, 180] or using your SWITCH limits
        //angleDeg = normalizeDeg(angleDeg);             // uses your existing normalizeDeg

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