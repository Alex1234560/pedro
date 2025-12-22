package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.FunctionsAndValues;
@Configurable
public class TurretRotation {
    private DcMotorEx TurretRotatorMotor = null;



//    public static double MAX_TICKS_ROTATION = 1000;
//    public static double MIN_TICKS_ROTATION = -1000;


    public static double MotorPowerWitouthAutoDebugging = 0;


    public static boolean AutoRotate = true;

    public static double FULL_TURN = 2725;// ticks that make a full turn

    public static double GoalAngle = 0;

    public static double SWITCH_ANGLE = 135;

    public static double kP = 0.0025;
    public static double kI = 0.0001;
    public static double kD = 0.007;
    public static double kF = .05;

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
    //public void start(){}

    public void update(double RobotAngleDeg, double RobotX, double RobotY){
        if (AutoRotate) {

            //i think ti doesnt handle floats well, so im rounding it.
            int IntRobotAngleDeg = (int) Math.round( RobotAngleDeg);
            double CurrentPos = TurretRotatorMotor.getCurrentPosition();

            //telemetry.addData("CurrentPos ", CurrentPos);

            double ActualTargetAngle = GoalAngle+IntRobotAngleDeg;

            if (ActualTargetAngle > SWITCH_ANGLE) {
                ActualTargetAngle -= 360;
            }
            if (ActualTargetAngle < -SWITCH_ANGLE) {
                ActualTargetAngle += 360;
            }

            double GoalTickPos = (FULL_TURN / 360) * ActualTargetAngle;

            // -------- line below is for tuning values ----------
            RotationalPIDF.updateCoefficients(kP,kI,kD,kF);

            double newPower = RotationalPIDF.calculate(GoalTickPos, CurrentPos);

            TurretRotatorMotor.setPower(newPower);


        }
        else{
            TurretRotatorMotor.setPower(MotorPowerWitouthAutoDebugging);
        }


    }

    public void TurretCalibrateToCenter(){
        TurretRotatorMotor.setPower(0);
        TurretRotatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public double GetCurrentPos(){
        return TurretRotatorMotor.getCurrentPosition();
    }

    public double getM (double goalPosx, double goalPosy, double curPosx, double curPosy){


        double x = goalPosx - curPosx;
        double y = goalPosy - curPosy;

        return Math.atan(y/x);
        //if doesnt work can try this: return Math.atan2(x, y); // angle from Y-axis
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
