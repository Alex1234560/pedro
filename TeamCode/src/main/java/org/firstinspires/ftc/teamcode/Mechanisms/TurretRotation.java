/* code for autoaiming turret
it features heading compensationa and PIDF as well as trigonometry to aim at the right target.
the angles the turret goes are from SWITCH_ANGLE_POS to SWITCH_ANGLE_NEG which are variables u can see below
 */


package org.firstinspires.ftc.teamcode.Mechanisms;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Functions.Coordinates;
import org.firstinspires.ftc.teamcode.Functions.FunctionsAndValues;
import org.firstinspires.ftc.teamcode.Functions.VelocityInterpolation;

@Configurable
public class TurretRotation {
    private FunctionsAndValues FAndV = new FunctionsAndValues();
    private Coordinates Cords = new Coordinates();

    private DcMotorEx TurretRotatorMotor = null;

    public static double CAMERA_MULTIPLIER_FOR_TURRET_CHANGE = .12;
    public static double CAMERA_STARTING_CHANGE = .005;

    public static double TURRET_AIMING_ALLOWED_ERROR = 2.5;

//    public static double BASE_FOR_VELOCITY = .003;
//    public static double MULTIPLIER_FOR_VELOCITY = 0.012;
    public static boolean IS_TUNING_VELOCITY_MODE_ON;
    public static double TUNABLE_VELOCITY_VALUE =0;


    private static boolean AUTO_ROTATE = true; // this is for counter rotating the turret with the heading variable
    private static boolean TRACK_GOAL = true; // this is for activating the trig math that handles aiming at the correct spot
    private static boolean USE_CAMERA_BEARING = true;

    private static double FULL_TURN = 1666;// ticks that make a full turn

    // ----- this are the limits that makes teh turret rotate in the opposite direction to not cross any cables -----
    private static double SWITCH_ANGLE_POS = 10;
    private static double SWITCH_ANGLE_NEG = -360;

    private double double_robot_angle_deg;
    private double actual_target_angle = 0;// these is the variable used to tell the turret what angle we want.
    private double angle_calculated_for_tracking_goal = 0;// this angle comes from the function getAngleFromTwoPoints
    private double camera_bearing_offset = 0;
    private static double turret_offset = 0;

    private boolean is_turret_being_centered;
    private boolean is_turret_being_manually_controlled;
    private double manual_control_offset;

    private static boolean COMPENSATE_FOR_TURRET_OFFSET = true;
    //public static double[] TurretOffsetINCHESXY = {2.85244094,-2.536};
    private static double[] TurretOffsetINCHESXY = {-2.85244094,2.536};
    private static double SIGN_MULTIPLIER_ROBOT_ANGLE_OFFSET = 1;

    private double turret_x;
    private double turret_y;
    private double goal_x=0;
    private double goal_y=0;

    private double Turret_Offset_For_When_Heading_Is_Reset;




    // ---------- PIDF values for turret ---------------
    public static double kP = .005;
    public static double kI = 0.0008;
    public static double kD = 0.008;
    public static double kF = 0.02;
//    public static double kP = .01;
//    public static double kI = 0.0001;
//    public static double kD = 0.002;
//    public static double kF = 0.032;
    SimplePIDF RotationalPIDF = new SimplePIDF(
            kP,  // kP  (start small)
            kI,     // kI  (usually 0)
            kD,     // kD  (often 0)
            kF
    );


    public void init(HardwareMap hardwareMap){
        IS_TUNING_VELOCITY_MODE_ON = false;


        //added line to see if i thelps
        //turret_offset=0;
        Turret_Offset_For_When_Heading_Is_Reset=0;

        is_turret_being_manually_controlled=false;
        is_turret_being_centered=false;
        TurretRotatorMotor = hardwareMap.get(DcMotorEx.class, "TurretRotator");
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TurretRotatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void update(Follower follower, Pose initPose, boolean IsRed){
            //getting valeus from follower.
            Pose robotPose = follower.getPose();
            //double TotalRotation = Math.toDegrees(follower.getHeading());
            double TotalRotation = Math.toDegrees(follower.getTotalHeading())- Turret_Offset_For_When_Heading_Is_Reset; //;

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


            turret_x = robotPose.getX();// +acceleration.getXComponent() * TUNING_VALUE_FOR_ACCELERATION;
            turret_y = robotPose.getY();// +  acceleration.getYComponent() * TUNING_VALUE_FOR_ACCELERATION;

            if (COMPENSATE_FOR_TURRET_OFFSET) {

                double BeforeRotX= turret_x+TurretOffsetINCHESXY[0];
                double BeforeRotY= turret_y+TurretOffsetINCHESXY[1];

                double[] RotatedCords = Cords.rotatePoint(BeforeRotX, BeforeRotY, turret_x, turret_y, Math.toRadians(double_robot_angle_deg*SIGN_MULTIPLIER_ROBOT_ANGLE_OFFSET));

                turret_x = RotatedCords[0];
                turret_y = RotatedCords[1];
            }

            //moving goal now?, maybe itll work better.

            goal_x=Cords.xFlip(Coordinates.GOAL_X,IsRed);
            goal_y=Coordinates.GOAL_Y;

            /// for changing goal position dinamically

            double difference = Math.abs(Coordinates.GOAL_Y-Coordinates.GOAL_Y_WHEN_AT_TOP_OF_FIELD);
            double transition = 2;


            if (turret_y>= (Coordinates.Y_LEVEL_TO_CHANGE_TO_TOP_SHOOTING-transition)){
                double progression = (turret_y-(Coordinates.Y_LEVEL_TO_CHANGE_TO_TOP_SHOOTING-transition))/transition;
                if (progression>1){progression=1;}
                if (progression<0){progression=0;}

                goal_y=Coordinates.GOAL_Y-(difference*progression);
            }

            /// ---------

            /// Accounting for velocity when aiming

            Vector velocity = follower.getVelocity();

            double distance = GetDistanceFromGoal(IsRed);
            double multiplyValue;
            //changed this so it can be tuned with a computer with panels
            if (IS_TUNING_VELOCITY_MODE_ON){multiplyValue=TUNABLE_VELOCITY_VALUE;}
            else{
                //multiplyValue = BASE_FOR_VELOCITY + (MULTIPLIER_FOR_VELOCITY*distance);
                multiplyValue= VelocityInterpolation.get(distance);
            }

            goal_x-=(velocity.getXComponent() * multiplyValue);
            goal_y-=(velocity.getYComponent() * multiplyValue);

            /// ------------------

            angle_calculated_for_tracking_goal = getAngleFromTwoPoints(goal_x,goal_y, turret_x, turret_y, IsRed);

            if (TRACK_GOAL && !is_turret_being_manually_controlled){
                actual_target_angle += angle_calculated_for_tracking_goal;
                manual_control_offset=angle_calculated_for_tracking_goal;
            }

            if (is_turret_being_manually_controlled){
                //manual_control_offset-=camera_bearing_offset;
                actual_target_angle+=manual_control_offset;
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

            TurretRotatorMotor.setPower(newPower);


        }

    // --------------- functions to return simple values ----------
    public double GetTargetAngle(){return actual_target_angle;}
    //public double DebugGetAngleCompensation(){return double_robot_angle_deg;}
    public double GetCurrentPosDeg(){return (GetCurrentPosTicks()/FULL_TURN)*360;}
    public double GetCameraBearingUsedInFile(){return camera_bearing_offset;}
    public double GetCurrentPosTicks(){return TurretRotatorMotor.getCurrentPosition();}
    public double GetCurrentVel(){return TurretRotatorMotor.getVelocity();}
    //public double ReturnGoalY(){return goal_y;}

    // -------------- complicated functions ------------------

//    public void resetTotalHeadingForRobotAndTurret(double totalHeadingInRad){
//        Turret_Offset_For_When_Heading_Is_Reset = Math.toDegrees(totalHeadingInRad);
//    }

    public boolean isTurretFinishedRotating(){
        double difference = Math.abs(Math.abs(GetCurrentPosDeg())-Math.abs(actual_target_angle));
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

    public void ManualTurretControl(boolean bool){
        is_turret_being_manually_controlled = bool;

    }
    public void updateManualOffset(double offset){
        manual_control_offset+=offset;
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

    public double GetDistanceFromGoal(boolean IsRed){
        //return FAndV.distance(turret_x, turret_y, (Cords.xFlip(Coordinates.GOAL_X_FOR_DISTANCE,IsRed)), Coordinates.GOAL_Y_FOR_DISTANCE);
        double distance = FAndV.distance(turret_x, turret_y, goal_x, goal_y);
        if (!is_turret_being_manually_controlled) {
            FAndV.updateSpeedTolerance(distance);
        }
        return distance;
    }



    public void resetTotalHeadingForRobotAndTurret(double TotalRotationRad){
        Turret_Offset_For_When_Heading_Is_Reset = Math.toDegrees(TotalRotationRad);
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