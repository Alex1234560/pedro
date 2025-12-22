//package org.firstinspires.ftc.teamcode.Mechanisms;
//
//import com.bylazar.configurables.annotations.Configurable;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//
//@TeleOp
//
//@Disabled
//public class FlywheelPIDF extends OpMode {
//    private DcMotorEx ShooterMotor = null;
//    private DcMotorEx ShooterMotor2 = null;
//
//    public static double TARGET_VELOCITY = 900;
//
//    public static boolean MOTOR_ON = false;
//
//    public static double P = 0;
//    public static double F = 0;
//    private double kP;
//
//
//    @Override
//    public void init(){
//
//
//        ShooterMotor = hardwareMap.get(DcMotorEx.class, "Shooter");
//        ShooterMotor2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
//
//        ShooterMotor2.setDirection(DcMotorEx.Direction.REVERSE);
//
//        ShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        ShooterMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        // run shooter with encoder
//
//
//        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0, F);
//
////        ShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
////        ShooterMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
//
//    }
//    @Override
//    public void loop(){
//        double flywheelSpeed = ShooterMotor.getVelocity();
//        double flywheelPower = 0;
//        double error = TARGET_VELOCITY-flywheelSpeed;
//
//
//        if (MOTOR_ON){
//
//            ShooterMotor.setPower(flywheelPower);
//            ShooterMotor2.setVelocity(flywheelPower);
//        }
//        else {
//            ShooterMotor.setPower(0);
//            ShooterMotor2.setPower(0);
//        }
//
//
//
//        telemetry.addData("error: ", error);
//        telemetry.addData("current Velocity : ", flywheelSpeed);
//        telemetry.addData("target Velocity : ", TARGET_VELOCITY);
//        telemetry.update();
//
//
//
//    }
//}
//
