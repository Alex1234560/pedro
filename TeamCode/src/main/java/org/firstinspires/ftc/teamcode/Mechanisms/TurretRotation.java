/* code for autoaiming turret
it features heading compensationa and PIDF as well as trigonometry to aim at the right target.
the angles the turret goes are from SWITCH_ANGLE_POS to SWITCH_ANGLE_NEG which are variables u can see below
 */


package org.firstinspires.ftc.teamcode.Mechanisms;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class TurretRotation {
    private DcMotorEx TurretRotatorMotor = null;

    public static double MotorPowerWitouthAutoDebugging = 0;


    public static boolean AUTO_ROTATE = true; // this is for counter rotating the turret with the heading variable
    public static boolean TRACK_GOAL = true; // this is for activating the trig math that handles aiming at the correct spot
    public static boolean USE_CAMERA_BEARING = false;
    public static boolean MOTOR_ACTIVE = true;// this is for activating the motor, inc ase u want to test something witouth the motor active

    public static int AUTO_AIMING_TURRET_OFFSET = 180; // this is just a base value, that is there to make the turret face the right way


    private static boolean LIMIT_VELOCITY_SWITCHES = false; // this is a prototype function that limits when the motor can switch directions due to speed, it wont be needed in the future
    private static double DONT_SWITCH_VALUE = 800;// this is for the var on top

    private boolean LIMIT_MAX_SPEED = true; // this is a cap on the motor speed so it doesn't skip gears
    public static double MAX_MOTOR_POWER = .3; // for the var avobe

    public static double FULL_TURN = 1666;// ticks that make a full turn


    // ----- this are the limits that makes teh turret rotate in the opposite direction to not cross any cables -----
    public static double SWITCH_ANGLE_POS = 180;
    public static double SWITCH_ANGLE_NEG = -190;

    private double double_robot_angle_deg;
    private double actual_target_angle = 0;// these is the variable used to tell the turret what angle we want.
    private double angle_calculated_for_tracking_goal = 0;// this angle comes from the function getAngleFromTwoPoints
    private double camera_bearing = 0;


    // ---------- PIDF values for turret ---------------
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


    public void update(double RobotAngleDeg, Pose robotPose, Pose goalPose, Pose initPose){


            // note that the function below allows the robotAngleDeg to get really high if u turn a lot, which could slow down the loop time
            // beacuse of the while loop below, so u could add an acumullating value that counters it.
            double_robot_angle_deg =  RobotAngleDeg;


            double current_position = GetCurrentPos();// telemetry
            double current_velocity = GetCurrentVel();// telemetry

            actual_target_angle = 0;// start the target angle as zero, add values to make it aim in the right dir

            if (AUTO_ROTATE) {
                actual_target_angle -= double_robot_angle_deg + Math.toDegrees(initPose.getHeading());
            }

            if (USE_CAMERA_BEARING){
                actual_target_angle+=camera_bearing;
            }


            angle_calculated_for_tracking_goal = getAngleFromTwoPoints(goalPose.getX(), goalPose.getY(), robotPose.getX(), robotPose.getY());
            if (TRACK_GOAL){
                actual_target_angle += angle_calculated_for_tracking_goal;
            }

            // ------------ limit handler -------------
            while ( actual_target_angle > SWITCH_ANGLE_POS || actual_target_angle < SWITCH_ANGLE_NEG) { // so if any of  the other functions mess up, this catches it.
                if (actual_target_angle > SWITCH_ANGLE_POS) {
                    actual_target_angle -= 360;

                }
                if (actual_target_angle < SWITCH_ANGLE_NEG) {
                    actual_target_angle += 360;

                }
            }

            double goal_tick_pos = (FULL_TURN / 360) * actual_target_angle; // gives us our target ticks value off of values we have

            // -------- line below is for tuning values ----------
            RotationalPIDF.updateCoefficients(kP,kI,kD,kF);

            // ----- getting the power the motor needs----
            double newPower = RotationalPIDF.calculate(goal_tick_pos, current_position);

            // -------------- LOGIC FOR NO SUDDEN DIRECTION CHANGE -------------------  -----

            double TurretLastPower = TurretRotatorMotor.getPower();
            if (LIMIT_VELOCITY_SWITCHES){

                double sign = 0;
                if (newPower != 0) {
                    sign = newPower / Math.abs(newPower);
                }
                ;
                double lastSign = 0;

                if (TurretLastPower != 0) {
                    lastSign = TurretLastPower / Math.abs(TurretLastPower);
                }

                if (Math.abs(current_velocity) > DONT_SWITCH_VALUE) {
                    if (sign != lastSign && sign != 0) {
                        newPower = 0;
                    }
                }

            }

            if (LIMIT_MAX_SPEED){
                if (Math.abs(newPower)>MAX_MOTOR_POWER)
                    newPower = MAX_MOTOR_POWER * Math.signum(newPower);
            }

            // ---------- setting power to motor -----------
            if (MOTOR_ACTIVE) {TurretRotatorMotor.setPower(newPower);}
            else{TurretRotatorMotor.setPower(0);}
        }

    // --------------- functions to return simple values ----------
    public double GetTargetAngle(){return actual_target_angle;}
    public double DebugGetAngleCompensation(){return double_robot_angle_deg;}
    public double GetCurrentPosDeg(){return (GetCurrentPos()/FULL_TURN)*360;}
    public double GetCurrentPos(){return TurretRotatorMotor.getCurrentPosition();}
    public double GetCurrentVel(){return TurretRotatorMotor.getVelocity();}
    public double GetGoalTrackingAngle(){return angle_calculated_for_tracking_goal;}

    // -------------- complicated functions ------------------

    // this function i think won't work beacuse it will wobble the shooter too much, but we can try.
    public void handleBearing(double bearing){
        if (bearing != 999){
            camera_bearing=bearing;
        }
        else{camera_bearing=0;}
    }

    public double GetDistanceFromGoal(Pose robotPose, Pose goalPose){

        return distance (robotPose.getX(),robotPose.getY(),goalPose.getX(), goalPose.getY());


    }

    public void CalibrateTurretToCenter(){
        TurretRotatorMotor.setPower(0);
        TurretRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.hypot(dx, dy);   // safer and avoids overflow
    }

    public double getAngleFromTwoPoints(double goalPosx, double goalPosy, double curPosx, double curPosy) {

        double dx = goalPosx - curPosx;
        double dy = goalPosy - curPosy;

        double angleRad = Math.atan2(dy, dx);          // -180..180
        double angleDeg = Math.toDegrees(angleRad);

        angleDeg += AUTO_AIMING_TURRET_OFFSET;         // account for turret mount
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