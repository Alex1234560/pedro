package org.firstinspires.ftc.teamcode.Functions; // Make sure this matches your team's package name

//import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;

import org.firstinspires.ftc.teamcode.Mechanisms.ShooterAngle;

/**
 * This class encapsulates all the logic for initializing and using the AprilTag processor.
 * It simplifies OpModes by hiding the complex setup of the VisionPortal and providing
 * easy-to-use methods for accessing detection data.
 */
@Configurable
public class FunctionsAndValues {

    private static double BackRangeStart = 100;
    private static double FrontRangeStart = 15;


    //public static double After90ChangeInAngle = 0; // was -3

    // swiched it from 60 to 300 cuz i believe the time it takes for the ball to reach flywheeel it goes up to speed enough to be accurate
    public static double SPEED_TOLERANCE_TO_SHOOT_BACK = 20;
    public static double SPEED_TOLERANCE_TO_SHOOT_FRONT = 50;
    public static double SpeedToleranceToStartShooting = SPEED_TOLERANCE_TO_SHOOT_BACK;


    public static double PowerValueForPreloading = 0.12;


    //public static double AngleToleranceToStartShooting = 2;

    public static double MinimumSpeed = 600;

    //public static double tuningMultiplier = 3.5;

    //public static boolean disableBearingPrediction = false;


    //public static double rotationTolerance = .5;


    //public static double GearRatio = 3;

    public static double kF = 0.000415873;
    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;
//    public static double kF = .0008;
//    public static double kP = 0.001;
//    public static double kI = 0;
//    public static double kD = 0;

    SimplePIDF flywheelPIDF = new SimplePIDF(
            kP,  // kP  (start small)
            kI,     // kI  (usually 0)
            kD,     // kD  (often 0)
            kF
    );

    public FunctionsAndValues() {

    }

    public double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.hypot(dx, dy);   // safer and avoids overflow
    }

    public double[] handleShootingRanges(double range) {
        double[] turretGoals = new double[2];
        double targAngle = 0;
        double targSpeed = 0;

        range= range+16.7;


        if (range<BackRangeStart) {
            //targAngle = (0.00779122 * range) + .121448;
            //targSpeed = (5.64501 * range) + 925.3;
            targAngle = (.00890654 * range) + .0542134;
            targSpeed = (6.62105 * range) + 827.79659;

            SpeedToleranceToStartShooting = SPEED_TOLERANCE_TO_SHOOT_FRONT;
        }
        else{
            targAngle=.9;
            //targSpeed = 11.12353*range+288.91;
            targSpeed = 4.07072*range+1077.60987;
            //targSpeed = 4.78571*range+1009.28571; // Alexs house measurement
            //targSpeed = (6.15554*range)+839.5422; One used in odometry pod class before.
            SpeedToleranceToStartShooting = SPEED_TOLERANCE_TO_SHOOT_BACK;
        }

        if (range<FrontRangeStart){targAngle = ShooterAngle.START_POINT;}

        //normalize
        if (targSpeed>2500){targSpeed=2500;}
        if (targSpeed<0){targSpeed=0;}


        turretGoals[0] = targAngle;
        turretGoals[1] = targSpeed;
        return turretGoals;
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

        public double calculate(double target, double current) {
            double error = target - current;

            // P
            double p = kP * error;

            // I
            integral += error;
            double i = kI * integral;

            // D
            double d = kD * (error - lastError);
            lastError = error;

            // F
            double f = kF * target;  // feedforward based on target RPM

            return p + i + d + f;
        }
    }

    public double handleShooter(double shooterTPS, boolean isMotorOn, double GoalSpeedTPS, double currentPower) {
        double newPower = flywheelPIDF.calculate(GoalSpeedTPS,shooterTPS);

        //if (Math.abs(GoalSpeedTPS-shooterTPS)<ToleranceForShooting){newPower=currentPower;}
        if (!isMotorOn){newPower=0;}
        if (newPower>1){newPower=1;}
        if (newPower<0){newPower=0;}
        return newPower;

    }

    public double[] getGoalOffset(double parallelx, double parallely, double perpx, double perpy){
    
        double StrafeOffset = 0.5;
        double RangeOffset = 0.5;
        
        double StrafeoffsetX = perpx * StrafeOffset;
        double StrafeoffsetY = perpy * StrafeOffset;
        
        double RangeoffsetX = parallelx * RangeOffset;
        double RangeoffsetY = parallely * RangeOffset;
        

        return new double[]{StrafeoffsetX, StrafeoffsetY, RangeoffsetX, RangeoffsetY};
    }


    public static double getDotProduct(double ax, double ay, double bx, double by) {
        return (ax * bx) + (ay * by);
    }
    public double[] GetTempGoalPos(double goalPoseX, double goalPoseY, double velocityVectorX, double velocityVectorY, double robotPoseX, double robotPoseY){

        // Returns parallel x,y and perp x, y
        // Parallel is the forward and back
        // Perp is the left and right

        // Solving for parallel: ((velx * goalx + vely * goaly) / (goalx * goalx + goaly * goaly)) * goal
        // In end it will be parallelx = goalx * scalar, y is same but Y, just needs 2 components


        // Solving for perpendicular: v - vparallel
        // Repeat the perpendicular for both x and y to get the vector


        //Gets Distances to goal
        double goalVectorX = goalPoseX - robotPoseX;
        double goalVectorY = goalPoseY - robotPoseY;

        //Gets dot product
        double dotProduct = getDotProduct(velocityVectorX, velocityVectorY, goalVectorX, goalVectorY);


        double squaredMagnitude = (goalVectorX * goalVectorX) + (goalVectorY * goalVectorY);

        double scalar = dotProduct / squaredMagnitude;

        double parallelX = goalVectorX * scalar;
        double parallelY = goalVectorY * scalar;

        double perpendicularX = velocityVectorX - parallelX;
        double perpendicularY= velocityVectorY - parallelY;
        
        return getGoalOffset(parallelX, parallelY, perpendicularX, perpendicularY);
    }

    
    public double GetSpeedAvgFromTwoMotors(double Motor1Speed, double Motor2Speed ){
        double Speed;
        double Motor1Vel = Math.abs(Motor1Speed);
        double Motor2Vel = Math.abs(Motor2Speed);
        //Speed = (Motor2Vel +Motor1Vel) /2;

        //Check if difference in speed is enough that its an issue. If its not, speed = average. If it is, speed = working motor
        //Determined by taking slower motor, and if its too slow using the faster one.
        if (Math.abs(Motor1Vel - Motor2Vel) > 100) {
            Speed = Math.min(Motor1Vel, Motor2Vel);
            //Final check to make sure its the correct shooting speed
            if (Speed < MinimumSpeed){
                Speed = Math.max(Motor1Vel, Motor2Vel);
            }
        } else {
            Speed = (Motor2Vel + Motor1Vel) / 2;
        }



        return Speed;
    }
    }








