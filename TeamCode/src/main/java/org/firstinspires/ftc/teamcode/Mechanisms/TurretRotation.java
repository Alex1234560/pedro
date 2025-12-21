package org.firstinspires.ftc.teamcode.Mechanisms;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.FunctionsAndValues;
@Configurable
public class TurretRotation {
    private DcMotorEx TurretRotatorMotor = null;


    private boolean AutoRotate = true;

//    public static double MAX_TICKS_ROTATION = 1000;
//    public static double MIN_TICKS_ROTATION = -1000;

    public static double FULL_TURN = 2400;// ticks that make a full turn

    public static double GoalAngle = 10;

    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0;

    SimplePIDF RotationalPIDF = new SimplePIDF(
            kP,  // kP  (start small)
            kI,     // kI  (usually 0)
            kD     // kD  (often 0)
    );

    public void init(HardwareMap hardwareMap){
        TurretRotatorMotor = hardwareMap.get(DcMotorEx.class, "TurretRotator");
        TurretRotatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void start(){


    }

    public void update(double RobotAngleDeg){
        if (AutoRotate) {
            telemetry.addData("Heading (DEG): ", RobotAngleDeg);
            double CurrentPos = TurretRotatorMotor.getCurrentPosition();

            telemetry.addData("CurrentPos ", CurrentPos);

            if (GoalAngle > 180) {
                GoalAngle -= 360;
            }
            if (GoalAngle < -180) {
                GoalAngle += 360;
            }

            double GoalTickPos = (FULL_TURN / 360) * GoalAngle;
            double newPower = RotationalPIDF.calculate(GoalTickPos, CurrentPos);

            TurretRotatorMotor.setPower(newPower);


        }

    }


    public static class SimplePIDF {
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
            double f = 0;  // feedforward based on target RPM

            return p + i + d + f;
        }
    }
}
