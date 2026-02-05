package org.firstinspires.ftc.teamcode.Functions; // Make sure this matches your team's package name

// TO DO: Check interpolation for bugs, & make it run more efficiently.


public class ShootingInterpolation {
    private static final Point[] TABLE = {
            //front
            new Point(16.7, .15, 828 ),
            new Point(31.7, .15,  927),
            new Point(38, .2189,  970),
            new Point(67, .5,  1061),
            new Point(82.75, .68,  1249),
            new Point(84.8, .673,  1186),
            new Point(101.75, .9,  1340),

            //back
            new Point(124.88, .9,  1442),
            new Point(139.277, .9,  1493),
            new Point(143.786, .9,  1520),
            new Point(154.18, .9,  1586.4)


    };

    // safety checks
    static {

        if (TABLE.length < 2) {
            throw new IllegalStateException("ShootingInterpolation arrays must have >= 2 points.");
        }

        for (int i = 1; i < TABLE.length; i++) {
            if (TABLE[i].dist <= TABLE[i - 1].dist) {
                throw new IllegalStateException("ShootingInterpolation table must be sorted by distance.");
            }
        }
    }

    private ShootingInterpolation() {}

    private static final class Point {
        final double dist;
        final double angle;
        final double tps;

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








