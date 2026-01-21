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

import org.firstinspires.ftc.teamcode.Functions.Coordinates;
import org.firstinspires.ftc.teamcode.Functions.FunctionsAndValues;

@Configurable
public class TurretRotation {
    private FunctionsAndValues FAndV = new FunctionsAndValues();
    private Coordinates Cords = new Coordinates();

    private DcMotorEx TurretRotatorMotor = null;


    public static double CAMERA_MULTIPLIER_FOR_TURRET_CHANGE = .09;
    public static double CAMERA_STARTING_CHANGE = .003;

    public static double TURRET_AIMING_ALLOWED_ERROR = 4.5;
    //public static double LARGE_VALUE_TO_CHANGE = 1;


    public static boolean AUTO_ROTATE = true; // this is for counter rotating the turret with the heading variable
    public static boolean TRACK_GOAL = true; // this is for activating the trig math that handles aiming at the correct spot
    public static boolean USE_CAMERA_BEARING = true;
    public static boolean MOTOR_ACTIVE = true;// this is for activating the motor, inc ase u want to test something witouth the motor active

    private static boolean LIMIT_VELOCITY_SWITCHES = false; // this is a prototype function that limits when the motor can switch directions due to speed, it wont be needed in the future
    private static double DONT_SWITCH_VALUE = 800;// this is for the var on top

    private boolean LIMIT_MAX_SPEED = true; // this is a cap on the motor speed so it doesn't skip gears
    public static double MAX_MOTOR_POWER = 1; // for the var avobe

    public static double FULL_TURN = 1666;// ticks that make a full turn

    // ----- this are the limits that makes teh turret rotate in the opposite direction to not cross any cables -----
    public static double SWITCH_ANGLE_POS = 10;
    public static double SWITCH_ANGLE_NEG = -360;

    private double double_robot_angle_deg;
    private double actual_target_angle = 0;// these is the variable used to tell the turret what angle we want.
    private double angle_calculated_for_tracking_goal = 0;// this angle comes from the function getAngleFromTwoPoints
    private double camera_bearing_offset = 0;
    public static double turret_offset = 0;

    private boolean is_turret_being_centered;

    public static boolean COMPENSATE_FOR_TURRET_OFFSET = true;
    //public static double[] TurretOffsetINCHESXY = {2.85244094,-2.536};
    public static double[] TurretOffsetINCHESXY = {-2.85244094,2.536};
    public static double SIGN_MULTIPLIER_ROBOT_ANGLE_OFFSET = 1;

    private double turret_x;
    private double turret_y;






    // ---------- PIDF values for turret ---------------
    public static double kP = .01;
    public static double kI = 0.0001;
    public static double kD = 0.002;
    public static double kF = 0.032;
    //    public static double kP = 0.004;
//    public static double kI = 0.001;//.0005
//    public static double kD = 0.005;
//    public static double kF = 0.05;//.025

    SimplePIDF RotationalPIDF = new SimplePIDF(
            kP,  // kP  (start small)
            kI,     // kI  (usually 0)
            kD,     // kD  (often 0)
            kF
    );


    public void init(HardwareMap hardwareMap){

        //added line to see if i thelps
        //turret_offset=0;


        is_turret_being_centered=false;
        TurretRotatorMotor = hardwareMap.get(DcMotorEx.class, "TurretRotator");
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TurretRotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(double TotalRotation, Pose robotPose, Pose goalPose, Pose initPose, boolean IsRed){


            double_robot_angle_deg =  TotalRotation + Math.toDegrees(initPose.getHeading());

            double current_position = GetCurrentPosTicks();// telemetry
            double current_velocity = GetCurrentVel();// telemetry

            actual_target_angle = 0;

            actual_target_angle+=turret_offset;

            if (AUTO_ROTATE) {
                actual_target_angle -= double_robot_angle_deg;
            }

            if (USE_CAMERA_BEARING){
                actual_target_angle+= camera_bearing_offset;
            }

            turret_x = robotPose.getX();
            turret_y = robotPose.getY();

            if (COMPENSATE_FOR_TURRET_OFFSET) {

                double BeforeRotX= turret_x+TurretOffsetINCHESXY[0];
                double BeforeRotY= turret_y+TurretOffsetINCHESXY[1];

                double[] RotatedCords = Cords.rotatePoint(BeforeRotX, BeforeRotY, turret_x, turret_y, Math.toRadians(double_robot_angle_deg*SIGN_MULTIPLIER_ROBOT_ANGLE_OFFSET));

                turret_x = RotatedCords[0];
                turret_y = RotatedCords[1];
            }

            angle_calculated_for_tracking_goal = getAngleFromTwoPoints(goalPose.getX(), goalPose.getY(), turret_x, turret_y, IsRed);

            if (TRACK_GOAL){
                actual_target_angle += angle_calculated_for_tracking_goal;
            }


            boolean is_turret_past_angle_pos = IsTurretPastAnglePos();
            boolean is_turret_past_angle_neg = IsTurretPastAngleNeg();

            if (is_turret_past_angle_pos){
                  turret_offset-=(360);
            }
            if (is_turret_past_angle_neg){
                turret_offset+=(360);
            }

            // this is for cenetring at the end of auto, and passing onto teleop
            if (is_turret_being_centered){
                actual_target_angle=0;
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
                if (newPower != 0) {sign = newPower / Math.abs(newPower);}
                double lastSign = 0;
                if (TurretLastPower != 0) {lastSign = TurretLastPower / Math.abs(TurretLastPower);}

                if (Math.abs(current_velocity) > DONT_SWITCH_VALUE) {if (sign != lastSign && sign != 0) {newPower = 0;}}
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
    //public double DebugGetAngleCompensation(){return double_robot_angle_deg;}
    public double GetCurrentPosDeg(){return (GetCurrentPosTicks()/FULL_TURN)*360;}
    public double GetCameraBearingUsedInFile(){return camera_bearing_offset;}
    public double GetCurrentPosTicks(){return TurretRotatorMotor.getCurrentPosition();}
    public double GetCurrentVel(){return TurretRotatorMotor.getVelocity();}
    public double GetGoalTrackingAngle(){return angle_calculated_for_tracking_goal;}

    // -------------- complicated functions ------------------

    public boolean isTurretFinishedRotating(){
        double difference = Math.abs(GetCurrentPosDeg()-actual_target_angle);
        return difference < TURRET_AIMING_ALLOWED_ERROR;
    }
    public boolean IsTurretPastAnglePos(){
        if (actual_target_angle>SWITCH_ANGLE_POS){
            return true;
        }
        else{
            return false;
        }
    }
    public boolean IsTurretPastAngleNeg(){
        if (actual_target_angle<SWITCH_ANGLE_NEG){
            return true;
        }
        else{
            return false;
        }
    }

    public void TurretTo0Deg(boolean bool){
        is_turret_being_centered = bool;
    }

    public void handleBearing(double bearing, double yaw){
        //this modifies bearing so it aims at behind the code and not at the code
        double bearing_with_yaw = ((bearing - ((-0.0893957*yaw)+0.0540481)) );
        //double bearing_with_yaw = bearing; // rn no bearing with yaw is going to be used.

        double changeValue = (CAMERA_MULTIPLIER_FOR_TURRET_CHANGE *Math.abs(bearing_with_yaw))+ CAMERA_STARTING_CHANGE;
        double ERROR_MARGIN = 1;
            if (bearing != 999){


                if (bearing_with_yaw > ERROR_MARGIN){
                    camera_bearing_offset += changeValue;
                }
                if (bearing_with_yaw < -ERROR_MARGIN){
                    camera_bearing_offset -=changeValue ;
                }

            }
            else{
                if (Math.abs(camera_bearing_offset) <= ERROR_MARGIN ){
                    camera_bearing_offset=0;
                }
                else {
                    //redefining change value to use in getting the offset to normal.
                    changeValue = (CAMERA_MULTIPLIER_FOR_TURRET_CHANGE *Math.abs(camera_bearing_offset))+ CAMERA_STARTING_CHANGE;
                    camera_bearing_offset -= (changeValue * Math.signum(camera_bearing_offset));
                }
            }
    }

    public double GetDistanceFromGoal(Pose goalPose){
        return FAndV.distance(turret_x, turret_y, goalPose.getX(), goalPose.getY());
    }

    public void CalibrateTurretToCenter(){
        turret_offset=0;
        TurretRotatorMotor.setPower(0);
        TurretRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double getAngleFromTwoPoints(double goalPosx, double goalPosy, double curPosx, double curPosy, boolean IsRed) {

        double dx = goalPosx - curPosx;
        double dy = goalPosy - curPosy;

        double angleRad = Math.atan2(dy, dx);          // -180..180
        double angleDeg = Math.toDegrees(angleRad);

        // this code below is to account for the trig limits, witouth this code, in blue side when it crosses certain line, the angle goes from 180 to -180 making the turret vibrate.
        if (!IsRed){ angleDeg = ((angleDeg + 360) % 360);}

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