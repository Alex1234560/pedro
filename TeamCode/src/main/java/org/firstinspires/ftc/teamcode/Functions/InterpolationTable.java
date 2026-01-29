package org.firstinspires.ftc.teamcode.Functions; // Make sure this matches your team's package name

//import com.acmerobotics.dashboard.config.Config;


public class InterpolationTable {
    private static final Point[] TABLE = {
            //front
            new Point(16.7, .15, 828 ),
            new Point(31.7, .15,  927),
            new Point(56.7, .41047, 1092 ),
            new Point(86.7, .67767,  1291),
            new Point(96.7, .76674,  1375.5),
            //back
            new Point(107.7, .9,  1400),
            new Point(143.7, .9,  1620),
            new Point(154.7, 0.9,  1690)

    };


    private InterpolationTable() {}

    private static final class Point {
        final double dist;
        final double angle;
        final double tps;   // you called it TPS (ticks per second?) keep same meaning

        Point(double dist, double angle, double tps) {
            this.dist = dist;
            this.angle = angle;
            this.tps = tps;
        }
    }


    private static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    public static double[] get(double distance){
        int n = TABLE.length;
        if (n < 2) {
            throw new IllegalStateException("ShotTable arrays must have >= 2 points.");
            //return new double[]{0,0};
        }

        else {
            if (distance <= TABLE[0].dist) {
                return new double[]{TABLE[0].angle, TABLE[0].tps};
            }
            if (distance >= TABLE[n - 1].dist) {
                return new double[]{TABLE[n - 1].angle, TABLE[n - 1].tps};
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

            double angle = lerp(a.angle, b.angle, t);
            double tps   = lerp(a.tps,   b.tps,   t);

            return new double[]{angle, tps};
        }
    }
}








