package org.firstinspires.ftc.teamcode.Functions;

import com.bylazar.configurables.annotations.Configurable;

@Configurable

public class Coordinates {

    // BLUE POSES ARE DEFAULT, THESE ARE BLUE side POSES
    public static final double StartingRobotAngleDeg = 144;

    public static final double GOAL_X = 0;
    public static final double GOAL_Y = 141;

    public static final double GOAL_Y_WHEN_AT_TOP_OF_FIELD = 135;
    //public static final double GOAL_X_WHEN_AT_BOTTOM_OF_FIELD = 137;

    public static double Y_LEVEL_TO_CHANGE_TO_TOP_SHOOTING = 130;

    public static final double GOAL_X_FOR_CAMERA = 16;
    public static final double GOAL_Y_FOR_CAMERA = 131;

    public static final double FRONT_START_X = 17.914;
    public static final double FRONT_START_Y = 121.168;

    public static final double RESTART_X = 134;
    public static final double RESTART_Y = 9.4;


    public static double xFlip(double oPos, boolean Red){
        double switcher;
        if (Red){switcher=144;
            return switcher-oPos; // Alex
        }
        else{switcher=0;
            return oPos;
        }

    }

    public static double angleFlip(double oAng, boolean Red) {
        double flipVal;
        if (Red){flipVal = 180;
            return flipVal-oAng;}
        else{flipVal = 0;
            return oAng;
        }
    }

    public static double[] rotatePoint(double px, double py, double pivotX, double pivotY, double angleRadians) {
        double dx = px - pivotX;
        double dy = py - pivotY;

        double cosA = Math.cos(angleRadians);
        double sinA = Math.sin(angleRadians);

        double rx = dx * cosA - dy * sinA;
        double ry = dx * sinA + dy * cosA;

        return new double[] {rx+pivotX, ry+pivotY};

  }

//    public double roundToNearest90(double angleDeg) {
//        return Math.round(angleDeg / 90.0) * 90.0;
//    }

}
