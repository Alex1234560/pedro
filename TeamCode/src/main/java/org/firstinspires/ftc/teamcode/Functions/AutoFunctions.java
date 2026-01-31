package org.firstinspires.ftc.teamcode.Functions;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;


@Configurable
public class AutoFunctions {

    public static Pose LastPoseRecorded;
    public static boolean DidAutoGoToEnd;
    public static boolean IsRed;


    //for intaking balls where positiion needs to be precise
    private static double ALLOWED_ERROR_POSITION = 6;
    private static double ALLOWED_ERROR_VELOCITY = 15;

    public static boolean isRobotInPosition(Pose GoalPose, Follower follower) {
        return isRobotInPositionCustomAmounts(GoalPose,follower,ALLOWED_ERROR_VELOCITY,ALLOWED_ERROR_POSITION);
    }
    public static boolean isRobotInPositionCustomAmounts(Pose GoalPose, Follower follower,double ALLOWED_ERROR_VEL, double ALLOWED_ERROR_POS) {
        Vector VelocityVector = follower.getVelocity();
        double vX = VelocityVector.getXComponent();
        double vY = VelocityVector.getYComponent();
        double Velocity = Math.hypot(vX, vY);

        boolean isVelocityAcceptable = Velocity <= ALLOWED_ERROR_VEL;

        double dx = GoalPose.getX() - follower.getPose().getX();
        double dy = GoalPose.getY() - follower.getPose().getY();
        double difference = Math.hypot(dx, dy);

        boolean isPositionAcceptable = difference <= ALLOWED_ERROR_POS;

        if (isPositionAcceptable && isVelocityAcceptable) {
            return true;
        } else {
            return false;

        }
    }

}
