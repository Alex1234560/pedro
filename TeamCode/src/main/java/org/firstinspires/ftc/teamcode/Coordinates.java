package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class Coordinates {

    public static Pose LastPoseRecorded=null;

    // BLUE POSES ARE DEFAULT, THESE ARE BLUE side POSES
    public static final double StartingRobotAngleDeg = 144;

    public static final double GOAL_X = 8;
    public static final double GOAL_Y = 136;

    public static final double GOAL_X_FOR_DISTANCE = 16;
    public static final double GOAL_Y_FOR_DISTANCE = 131;

    public static final double START_X = 17.914;
    public static final double START_Y = 121.168;


    public double xFlip(double oPos, boolean Red){
        double switcher;
        if (Red){switcher=144;
            return switcher-oPos; // Alex
        }
        else{switcher=0;
            return oPos;
        }

    }

    public double angleFlip(double oAng, boolean Red) {
        double flipVal;
        if (Red){flipVal = 180;
            return flipVal-oAng;}
        else{flipVal = 0;
            return oAng;
        }
    }

    public double[] rotatePoint(double px, double py, double pivotX, double pivotY, double angleRadians) {
        double dx = px - pivotX;
        double dy = py - pivotY;

        double cosA = Math.cos(angleRadians);
        double sinA = Math.sin(angleRadians);

        double rx = dx * cosA - dy * sinA;
        double ry = dx * sinA + dy * cosA;

        return new double[] {rx+pivotX, ry+pivotY};

  }

}
