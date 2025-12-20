package org.firstinspires.ftc.teamcode; // Make sure this matches your team's package name

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.Range;

/**
 * This class encapsulates all the logic for initializing and using the AprilTag processor.
 * It simplifies OpModes by hiding the complex setup of the VisionPortal and providing
 * easy-to-use methods for accessing detection data.
 */

public class FunctionsAndValues {

    private static double BackRangeStart = 90;

    public static double After90ChangeInAngle = 0; // was -3

    public static double SpeedToleranceToStartShooting = 60;
    public static double AngleToleranceToStartShooting = 2;

    public static double MinimumSpeed = 600;

    public static double tuningMultiplier = 3.5;

    //public static boolean disableBearingPrediction = false;

    public static double startPoint = .15;
    public static double endPoint = .9;//.7


    //public static double rotationTolerance = .5;


    public static double GearRatio = 3;


    public static double kP = 0.005;
    public static double kI = 0;
    public static double kD = 0;

    SimplePIDF flywheelPIDF = new SimplePIDF(
            kP,  // kP  (start small)
            kI,     // kI  (usually 0)
            kD     // kD  (often 0)
    );

    public FunctionsAndValues() {

    }

    public double ReCalibrateShooterSpeed(double GoalTPS) {
        return calculateSpeedForShooter(GoalTPS);
    }

    public double[] calculateShooterRotation(double bearing, boolean autorotate, double currentAngle, boolean auto, double range) {
        double rotationCompensation = 0;
        if (auto){
            rotationCompensation = .1 * tuningMultiplier;
        }
        else{
            rotationCompensation = .5* tuningMultiplier;
        }

        double[] ValuesForAngleAndCurrentAngleAndNewBearing = new double[3];

        double newCurrentAngle = currentAngle;
        double newBearing = bearing;

        if (range>BackRangeStart){
            if (bearing > 0){
                bearing += After90ChangeInAngle;
            }
            if (bearing < 0){
                bearing -= After90ChangeInAngle;
            }
        }

        if ( Math.abs(bearing) <= 30 && autorotate && bearing != 999) {


            double angleChangeAmount = (bearing/GearRatio) * rotationCompensation;
            double bearingChangeAmount = bearing * rotationCompensation;
            //if (disableBearingPrediction){bearingChangeAmount=0;}
            if (Math.abs(bearing)<6){
                angleChangeAmount=angleChangeAmount/2;
                bearingChangeAmount=angleChangeAmount/2;
            }
            if (Math.abs(bearing)<3){
                angleChangeAmount=angleChangeAmount/3;
                bearingChangeAmount=angleChangeAmount/3;
            }


            //newCurrentAngle += (bearing/GearRatio) * rotationCompensation;
            newCurrentAngle +=angleChangeAmount;
            newBearing -= bearingChangeAmount;

            newCurrentAngle = Math.min(180, Math.max(0, newCurrentAngle));
        }


        //returning value that is mapped from degrees to 0-1.
        double valueForShooterServo = Range.scale(
                newCurrentAngle,   // value you want to map
                0, 180,        // input range
                0,  // output start
                1     // output end
        );

        ValuesForAngleAndCurrentAngleAndNewBearing[0] = valueForShooterServo;
        ValuesForAngleAndCurrentAngleAndNewBearing[1] = newCurrentAngle;
        ValuesForAngleAndCurrentAngleAndNewBearing[2] = newBearing;

        return ValuesForAngleAndCurrentAngleAndNewBearing;
    }


    //range to rpm and angle
    /*public double[] handleShootingRanges(double range) {

        double[] turretGoals = new double[2];

        //double targAngle = (0.00493055 * range) + 0.243814;
        double targAngle = (0.00729122*range) +0.0887001;
        double targSpeed = (6.94554 * range) + 850.3396;

        //normalize
        if (targAngle>endPoint){targAngle=endPoint;}
        if (targAngle<startPoint){targAngle=startPoint;}
        if (targSpeed>2500){targSpeed=2500;}
        if (targSpeed<0){targSpeed=0;}

        if (range>BackRangeStart){
            targSpeed+=After90ChangeInSpeed;
        }


        turretGoals[0] = targAngle;
        turretGoals[1] = targSpeed;
        return turretGoals;
    }*/
    public double[] handleShootingRanges(double range) {

        double[] turretGoals = new double[2];
        double targAngle = 0;
        double targSpeed = 0;

        //double targAngle = (0.00493055 * range) + 0.243814;
        if (range<BackRangeStart) {
             targAngle = (0.00729122 * range) + 0.0887001;
             targSpeed = (6.94554 * range) + 850.3396;
        }else{
            targAngle=.9;
            //targSpeed =(-.071246 * (range*range)) + 22.62838*(range) - 137.62456; Joannas House measurements
            targSpeed = 4.78571*range+1009.28571; // Alexs house measurement
        }

        //normalize
        if (targAngle>endPoint){targAngle=endPoint;}
        if (targAngle<startPoint){targAngle=startPoint;}
        if (targSpeed>2500){targSpeed=2500;}
        if (targSpeed<0){targSpeed=0;}


        turretGoals[0] = targAngle;
        turretGoals[1] = targSpeed;
        return turretGoals;
    }
    //rpm to power
    public double calculateSpeedForShooter(double GoalTPS) {
        // tweak these numbers if you need to recalibrate
        double MotorSpeed = 0.000415873 * GoalTPS + 0.0117401;
        return MotorSpeed;
    }

    public class SimplePIDF {
        public double kP, kI, kD;

        private double integral = 0;
        private double lastError = 0;

        public SimplePIDF(double kP, double kI, double kD) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;

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
            double f =0.000415873 * target + 0.0117401;  // feedforward based on target RPM

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
    public double GetSpeedAvgFromTwoMotors(double Motor1Speed, double Motor2Speed ){
        double Speed = 0;
        double Motor1Vel = Math.abs(Motor1Speed);
        double Motor2Vel =Math.abs(Motor2Speed);
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








