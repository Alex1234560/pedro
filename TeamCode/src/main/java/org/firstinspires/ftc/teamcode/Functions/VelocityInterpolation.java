package org.firstinspires.ftc.teamcode.Functions; // Make sure this matches your team's package name

//import com.acmerobotics.dashboard.config.Config;


public class VelocityInterpolation {
    private static final Point[] TABLE = {
            //front
            new Point(0, .003),
            new Point(15, .183 ),
            new Point(200, 2.403),


    };

    // safety checks
    static {

        if (TABLE.length < 2) {
            throw new IllegalStateException("VelocityInterpolation arrays must have >= 2 points.");
        }

        for (int i = 1; i < TABLE.length; i++) {
            if (TABLE[i].dist <= TABLE[i - 1].dist) {
                throw new IllegalStateException("VelocityInterpolation table must be sorted by distance.");
            }
        }

    }

    private VelocityInterpolation() {}

    private static final class Point {
        final double dist;
        final double velocity_multiplier;

        Point(double dist, double velocity_multiplier) {
            this.dist = dist;
            this.velocity_multiplier = velocity_multiplier;

        }
    }


    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double get(double distance){
        int n = TABLE.length;


        if (distance <= TABLE[0].dist) {
            return TABLE[0].velocity_multiplier;}

        if (distance >= TABLE[n - 1].dist) {
            return TABLE[n-1].velocity_multiplier;
        }

        int i = 0;
        while (i < n - 2 && distance > TABLE[i + 1].dist) {
            i++;
        }

        Point a = TABLE[i];
        Point b = TABLE[i + 1];

        double t = (distance - a.dist) / (b.dist - a.dist);
        if (t > 1) {
            t = 1;
        }
        if (t < 0) {
            t = 0;
        }

        double velocity_multiplier = lerp(a.velocity_multiplier, b.velocity_multiplier, t);

        return velocity_multiplier;

    }
}








