package org.firstinspires.ftc.teamcode;


public class Coordinates {

    // BLUE POSES ARE DEFAULT, THESE ARE BLUE side POSES
    public static final double StartingRobotAngleDeg = 144;
    public static final double GOAL_X = 12.5;
    public static final double GOAL_Y = 136.5;
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

}
