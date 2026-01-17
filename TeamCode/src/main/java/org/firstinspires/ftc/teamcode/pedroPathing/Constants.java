package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(15)
            .forwardZeroPowerAcceleration(-54.38182942200218)
            .lateralZeroPowerAcceleration(-70.86148762045916)
            //.translationalPIDFCoefficients(new PIDFCoefficients(.1, 0.0001,.0025, .025))
            .translationalPIDFCoefficients(new PIDFCoefficients(.15, 0.00005,.005, .01))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,.0045, .0275))
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0,0.0001, .06,0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.025, 0,0.001, .06,0.01))

            .centripetalScaling(.0005)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(62.1326)
            .yVelocity(50.524395319)
            .maxPower(1)
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            .8,
            1)

            ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            //the value commented out was the one i used to make circle reallly small
//            .forwardPodY(-.1)// im trying this one to make auto work well.
//            .strafePodX(.9)
            //actual measurements
//            .forwardPodY(1.4)
//            .strafePodX(0)

            //smallest ive gotten so far.
            .forwardPodY(-1)
            .strafePodX(.5)

            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .pinpointLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
